Final code for micromouse  #include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// =====================================================
// ================= PIN DEFINITIONS ===================
// =====================================================

// ---- TB6612FNG Motor Driver ----
#define PWMA 5
#define AIN1 6
#define AIN2 7
#define PWMB 11
#define BIN1 9
#define BIN2 10

// ---- Encoder Interrupt Pins ----
#define R_ENC_A 2
#define L_ENC_A 3

// ---- VL53L0X XSHUT Pins ----
#define XSHUT_L 12
#define XSHUT_F 13
#define XSHUT_R A1

// ---- Indicators ----
#define LED 8
#define BUZZER A3

// =====================================================
// ================= ROBOT CONSTANTS ===================
// =====================================================
#define FRONT_LIMIT 90

#define BASE_SPEED 255
#define TURN_SPEED 120

// ---- STRAIGHT PWM (FROM MOTOR TEST) ----
#define RIGHT_PWM_MAX 255
#define LEFT_PWM_MAX  165

// ---- TURN PWM (BALANCED ROTATION) ----
#define RIGHT_TURN_PWM 140
#define LEFT_TURN_PWM  255

// =====================================================
// ================= OBJECTS ===========================
// =====================================================
Adafruit_SSD1306 oled(128, 64, &Wire);
VL53L0X sensorL, sensorF, sensorR;
MPU6050 mpu;

// =====================================================
// ================= ENCODERS ==========================
// =====================================================
volatile long rCount = 0;
volatile long lCount = 0;

void ISR_R() { rCount++; }
void ISR_L() { lCount++; }

void resetEncoders() {
  noInterrupts();
  rCount = 0;
  lCount = 0;
  interrupts();
}

// =====================================================
// ================= MOTOR CONTROL =====================
// =====================================================

// ---- STRAIGHT MOTION ----
void motorLeft(int spd) {
  spd = constrain(spd, -LEFT_PWM_MAX, LEFT_PWM_MAX);
  digitalWrite(BIN1, spd >= 0);
  digitalWrite(BIN2, spd < 0);
  analogWrite(PWMB, abs(spd));
}

void motorRight(int spd) {
  spd = constrain(spd, -RIGHT_PWM_MAX, RIGHT_PWM_MAX);
  digitalWrite(AIN1, spd >= 0);
  digitalWrite(AIN2, spd < 0);
  analogWrite(PWMA, abs(spd));
}

// ---- TURN MOTION ----
void motorLeftTurn(int spd) {
  spd = constrain(spd, -LEFT_TURN_PWM, LEFT_TURN_PWM);
  digitalWrite(BIN1, spd >= 0);
  digitalWrite(BIN2, spd < 0);
  analogWrite(PWMB, abs(spd));
}

void motorRightTurn(int spd) {
  spd = constrain(spd, -RIGHT_TURN_PWM, RIGHT_TURN_PWM);
  digitalWrite(AIN1, spd >= 0);
  digitalWrite(AIN2, spd < 0);
  analogWrite(PWMA, abs(spd));
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// =====================================================
// ============ STRAIGHT PID (ENCODERS) =================
// =====================================================
float Kp_straight = 0.45;
float Ki_straight = 0.0;
float Kd_straight = 0.25;

long lastError = 0;
long integral = 0;

void moveStraightPID(int speed) {
  long error = lCount - rCount;

  if (abs(error) <= 1) error = 0;
  error = constrain(error, -10, 10);

  integral += error;
  integral = constrain(integral, -200, 200);

  long derivative = error - lastError;
  lastError = error;

  long correction =
    Kp_straight * error +
    Ki_straight * integral +
    Kd_straight * derivative;

  correction = constrain(correction, -30, 30);

  motorLeft(speed - correction);
  motorRight(speed + correction);
}

// =====================================================
// ================= MPU TURN ==========================
// =====================================================
float Kp_turn = 1.1;
float Kd_turn = 0.4;

float gyroZOffset = 0;
float angleZ = 0;
float lastTurnError = 0;
unsigned long lastMicros = 0;

void calibrateMPU() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZOffset = sum / 500.0;
}

void updateAngle() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float gyroZ = (gz - gyroZOffset) / 131.0;

  angleZ += gyroZ * dt;
}

void turnMPU(float targetAngle) {
  angleZ = 0;
  lastTurnError = 0;
  lastMicros = micros();
  resetEncoders();

  unsigned long startTime = millis();
  unsigned long blinkTimer = 0;
  bool blinkState = false;

  while (true) {
    updateAngle();

    // ---- LED + BUZZER BLINK ----
    if (millis() - blinkTimer >= 100) {
      blinkTimer = millis();
      blinkState = !blinkState;
      digitalWrite(LED, blinkState);
      if (blinkState) tone(BUZZER, 2000);
      else noTone(BUZZER);
    }

    float error = targetAngle - angleZ;
    float control =
      Kp_turn * error +
      Kd_turn * (error - lastTurnError);

    lastTurnError = error;
    control = constrain(control, -TURN_SPEED, TURN_SPEED);

    motorLeftTurn(-control);
    motorRightTurn(control);

    if (abs(error) < 1.0) break;
    if (millis() - startTime > 800) break;
  }

  stopMotors();

  // ---- TURN COMPLETE SIGNAL ----
  digitalWrite(LED, HIGH);
  tone(BUZZER, 3000);
  delay(150);
  digitalWrite(LED, LOW);
  noTone(BUZZER);

  delay(120);
}

// =====================================================
// ================= SENSOR SETUP ======================
// =====================================================
void setupSensors() {
  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);

  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(50);

  digitalWrite(XSHUT_L, HIGH);
  delay(10);
  sensorL.init();
  sensorL.setAddress(0x30);

  digitalWrite(XSHUT_F, HIGH);
  delay(10);
  sensorF.init();
  sensorF.setAddress(0x31);

  digitalWrite(XSHUT_R, HIGH);
  delay(10);
  sensorR.init();
  sensorR.setAddress(0x32);

  sensorL.startContinuous();
  sensorF.startContinuous();
  sensorR.startContinuous();
}

// =====================================================
// ===================== SETUP =========================
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(R_ENC_A), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), ISR_L, RISING);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(20, 25);
  oled.print("READY");
  oled.display();

  setupSensors();
  mpu.initialize();
  calibrateMPU();
  resetEncoders();
}

// =====================================================
// ====================== LOOP =========================
// =====================================================
void loop() {
  int dF = sensorF.readRangeContinuousMillimeters();
  int dL = sensorL.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();

  if (dF < FRONT_LIMIT) {
    stopMotors();
    delay(100);

    if (dL > dR) turnMPU(90);
    else         turnMPU(-90);

    resetEncoders();
    return;
  }

  moveStraightPID(BASE_SPEED);
}

