// Merged PID yaw control into wall-following code
#include <Wire.h>
#include <MPU6050.h>

// --- MPU6050 ---
MPU6050 mpu;
float yaw = 0;
unsigned long lastTime;

// PID parameters for yaw control
float setpointYaw = 0.0;
float Kp = 4.0, Ki = 0.1, Kd = 1.0;
float integral = 0.0, lastError = 0.0;
int rightBias = 10;      // optional bias for right motor

// --- Ultrasonic sensors ---
int trig1 = 6, echo1 = 7;    // left
int trig2 = 2, echo2 = 4;    // middle
int trig3 = 8, echo3 = 9;    // right

// --- Motor driver pins ---
#define AIN1 A0
#define AIN2 A1
#define PWMA 3
#define BIN1 A2
#define BIN2 A3
#define PWMB 5

int speed = 100;     // base PWM speed
int DIS = 10;        // distance threshold (cm)
float speedSound = 0.0343;

// flag to detect entering straight segment
bool prevStraight = false;

// --- Function prototypes ---
long measureDistance(int trig, int echo);
void updateYaw();
void driveMotors(int lspeed, int rspeed);
void moveStop();
void turnLeft90();
void turnRight90();
void moveForwardL();
void moveForwardR();
void moveForward();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 lỗi kết nối!");
    while (1);
  }
  lastTime = millis();
  yaw = 0;

  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  moveStop();
  Serial.println(">> CHỜ TƯỜNG 2 BÊN...");
}

void loop() {
  // Wait for two walls detected
  while (true) {
    long lD = measureDistance(trig1, echo1);
    long rD = measureDistance(trig3, echo3);
    if (lD < 25 && rD < 25) break;
    delay(200);
  }
  Serial.println(">> BẮT ĐẦU DI CHUYỂN!");

  // Main control loop
  while (true) {
    updateYaw();

    long leftDistance = measureDistance(trig1, echo1);
    delay(10);
    long middleDistance = measureDistance(trig2, echo2);
    delay(10);
    long rightDistance = measureDistance(trig3, echo3);
    delay(10);

    Serial.print("L:"); Serial.print(leftDistance);
    Serial.print(" M:"); Serial.print(middleDistance);
    Serial.print(" R:"); Serial.println(rightDistance);
    int wall_left = (leftDistance < DIS) ? 1 : 0;
    int wall_front = (middleDistance < DIS) ? 1 : 0;
    int wall_right = (rightDistance < DIS) ? 1 : 0;

    bool isStraight = (middleDistance > DIS);
    if (isStraight) {
      // entering straight: update setpoint at current yaw
      if (!prevStraight) {
        setpointYaw = yaw;
        integral = 0;
        lastError = 0;
      }
      // PID correction
      float error = setpointYaw - yaw;
      integral = constrain(integral + error, -50, 50);
      float derivative = error - lastError;
      float corr = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;
      int ls = constrain(speed + corr, 0, 255);
      int rs = constrain(speed - corr, 0, 255);
      driveMotors(ls, rs);

    } else {
      // prioritize turns and dead-end
      if (leftDistance > DIS && middleDistance < DIS && rightDistance < DIS) {
        moveForward();delay(200);
        moveStop(); delay(200);
        turnLeft90();
        driveMotors(speed, speed);
      } else if (leftDistance < DIS && middleDistance < DIS && rightDistance > DIS) {
        moveForward();delay(200);
        moveStop(); delay(200);
        turnRight90();than
        driveMotors(speed, speed);
      } else if (leftDistance < DIS && middleDistance < DIS && rightDistance < DIS) {
        // dead end
        moveStop();
      } else if (leftDistance < DIS && middleDistance > DIS && rightDistance > DIS) {
        // dead end
        moveStop();
      }
    }
    prevStraight = isStraight;
  }
}

// --- Functions ---
long measureDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH);
  return (d == 0 ? 30000 : d) / 2.0 * speedSound;
}

void updateYaw() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  yaw += (gz / 131.0) * dt;
}

void driveMotors(int lspeed, int rspeed) {
  rspeed = constrain(rspeed + rightBias, 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, lspeed);
  analogWrite(PWMB, rspeed);
}

void moveStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void turnLeft90() {
  float start = yaw;
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);
  while (abs(yaw - start) < 75) updateYaw();
  moveStop(); delay(200);
}

void turnRight90() {
  float start = yaw;
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);
  while (abs(yaw - start) < 75) updateYaw();
  moveStop(); delay(200);
}

// Di chuyển thẳng
void moveForward() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed);
}
