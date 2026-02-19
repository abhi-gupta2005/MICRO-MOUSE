#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// ================= PINS =================
#define PWMA 5
#define AIN1 6
#define AIN2 7
#define PWMB 11
#define BIN1 9
#define BIN2 10

#define XSHUT_L 12
#define XSHUT_F 13
#define XSHUT_R A1

#define R_ENC_A 2
#define L_ENC_A A2

#define LED 8

// ================= LIMITS =================
#define FRONT_LIMIT 90
#define BASE_SPEED 255

// ================= OBJECTS =================
Adafruit_SSD1306 oled(128, 64, &Wire);
VL53L0X sensorL, sensorF, sensorR;
MPU6050 mpu;

// ================= ENCODERS =================
volatile long rCount = 0;
volatile long lCount = 0;

void ISR_R() { rCount++; }
void ISR_L() { lCount--; }   // change sign only if direction is wrong

// ================= MOTOR =================
void motorLeft(int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(BIN1, spd > 0);
  digitalWrite(BIN2, spd <= 0);
  analogWrite(PWMB, abs(spd));
}

void motorRight(int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(AIN1, spd > 0);
  digitalWrite(AIN2, spd <= 0);
  analogWrite(PWMA, abs(spd));
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// =================================================
// ============ STRAIGHT PID (ENCODER) ===============
// =================================================

// 🔧 TUNE HERE
float Kp_straight = 0.7;
float Kd_straight = 0.4;

long lastStraightError = 0;

void moveStraightPID(int speed) {
  long error = lCount - rCount;

  long correction =
    Kp_straight * error +
    Kd_straight * (error - lastStraightError);

  lastStraightError = error;

  correction = constrain(correction, -40, 40);

  motorLeft(speed - correction);
  motorRight(speed + correction);
}

// =================================================
// ============== MPU TURN PID ======================
// =================================================

float Kp_turn = 1.0;
float Kd_turn = 0.5;

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

  digitalWrite(LED, HIGH);
  unsigned long startTime = millis();

  while (true) {
    updateAngle();

    float error = targetAngle - angleZ;
    float output =
      Kp_turn * error +
      Kd_turn * (error - lastTurnError);

    lastTurnError = error;

    output = constrain(output, -120, 120);

    motorLeft(-output);
    motorRight(output);

    if (abs(error) < 1.0) break;
    if (millis() - startTime > 800) break;
  }

  stopMotors();
  digitalWrite(LED, LOW);
  delay(100);
}

// ================= SENSOR SETUP =================
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

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(R_ENC_A), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), ISR_L, RISING);

oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
oled.clearDisplay();
oled.setTextSize(2);
oled.setTextColor(WHITE);

// First line
oled.setCursor(0, 10);
oled.print("LORD");

// Second line
oled.setCursor(0, 30);
oled.print("ABHISHEK");

oled.display();

  setupSensors();

  mpu.initialize();
  calibrateMPU();
}

// ================= LOOP =================
void loop() {
  int dF = sensorF.readRangeContinuousMillimeters();
  int dL = sensorL.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();

  Serial.print("L:");
  Serial.print(dL);
  Serial.print(" F:");
  Serial.print(dF);
  Serial.print(" R:");
  Serial.println(dR);

  if (dF <= FRONT_LIMIT) {
    stopMotors();
    delay(120);

    if (dL > dR) turnMPU(90);
    else         turnMPU(-90);

    return;
  }

  moveStraightPID(BASE_SPEED);
}
