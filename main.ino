#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Cảm biến siêu âm
int trig1 = 6, echo1 = 7;
int trig2 = 2, echo2 = 4;
int trig3 = 8, echo3 = 9;

#define buttonR 10
#define buttonL A6
// Motor driver
#define AIN1 A0
#define AIN2 A1
#define PWMA 3
#define BIN1 A2
#define BIN2 A3
#define PWMB 5

// Ngưỡng & tốc độ
float speedSound = 0.0343;
int DIS = 15;
int speed = 100;

// Khoảng cách đo được
long leftDistance, middleDistance, rightDistance;

// Biến yaw
float yaw = 0;
unsigned long lastTime;

// Đọc khoảng cách siêu âm
long measureDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return (pulseIn(echo, HIGH) / 2.0) * speedSound;
}

// Cập nhật yaw bằng tích phân gyroZ
void updateYaw() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  float gyroZ = gz / 131.0;      // độ/giây
  yaw += gyroZ * dt;            // tích phân thành góc
}

// Quay trái chính xác 90°
void turnLeft90() {
  updateYaw();
  float start = yaw;
  // bắt đầu quay trái
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  // quay cho đến khi đạt ~90°
  while (abs(yaw - start) < 85) {
    updateYaw();
  }
  // dừng
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(200);
}

// Quay phải chính xác 90°
void turnRight90() {
  updateYaw();
  float start = yaw;
  // bắt đầu quay phải
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  // quay cho đến khi đạt ~90°
  while (abs(yaw - start) < 85) {
    updateYaw();
  }
  // dừng
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(200);
}
// Quay phải chính xác 90°
void turnRight180() {
  updateYaw();
  float start = yaw;
  // bắt đầu quay phải
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  // quay cho đến khi đạt ~90°
  while (abs(yaw - start) < 180) {
    updateYaw();
  }
  // dừng
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(200);
}

// Di chuyển tiến thẳng
void moveForward() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

// Dừng
void moveStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Điều chỉnh song song tường phải
void moveForwardR() {
  long r = measureDistance(trig3, echo3);
  if (r < 6)        analogWrite(PWMB, speed), analogWrite(PWMA, speed/2);
  else if (r > 7)   analogWrite(PWMA, speed), analogWrite(PWMB, speed/2);
  else              analogWrite(PWMA, speed), analogWrite(PWMB, speed);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
}

// Điều chỉnh song song tường trái
void moveForwardL() {
  long l = measureDistance(trig1, echo1);
  if (l < 7)        analogWrite(PWMB, speed/2), analogWrite(PWMA, speed);
  else if (l > 8)   analogWrite(PWMA, speed/2), analogWrite(PWMB, speed);
  else              analogWrite(PWMA, speed),   analogWrite(PWMB, speed);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 lỗi kết nối!");
    while (1);
  }
  lastTime = millis();
  yaw = 0; // reset góc ban đầu
  pinMode(buttonR, INPUT_PULLUP);   // Chân D10 dùng kéo trở trong
  pinMode(buttonL, INPUT_PULLUP);    // Chân A6 cũng dùng kéo trở trong
  // khởi tạo siêu âm
  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);

  // khởi tạo motor
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  moveStop();
}

void loop() {
  moveStop();
  delay(10);

  // Chờ 2 cảm biến trái+phải <20 mới bắt đầu
  while (true) {
    leftDistance  = measureDistance(trig1, echo1);
    rightDistance = measureDistance(trig3, echo3);
    if (leftDistance < 25 && rightDistance < 25) break;
    delay(200);
  }

  // Vòng chạy chính
  while (true) {
    // cập nhật yaw liên tục
    updateYaw();

    // đo khoảng cách
    leftDistance   = measureDistance(trig1, echo1);    delay(10);
    middleDistance = measureDistance(trig2, echo2);    delay(10);
    rightDistance  = measureDistance(trig3, echo3);    delay(10);

    Serial.print("L:"); Serial.print(leftDistance);
    Serial.print(" M:"); Serial.print(middleDistance);
    Serial.print(" R:"); Serial.println(rightDistance);

    if (leftDistance < DIS && middleDistance > DIS && rightDistance < DIS) {
      // đi thẳng + tự chỉnh                               
      moveForward();
      if (leftDistance < rightDistance && leftDistance < 8) {
        analogWrite(PWMA, 100); analogWrite(PWMB, 80);
      } else if (rightDistance < leftDistance && rightDistance < 8) {
        analogWrite(PWMA, 80); analogWrite(PWMB, 100);
      }
    }
    else if (leftDistance > DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForwardR();
    }
    else if (leftDistance < DIS && middleDistance > DIS && rightDistance > DIS) {
      moveForwardL();
    }
    else if (leftDistance > DIS && middleDistance < DIS && rightDistance > DIS) {
      // quẹo phải 90°
      moveForward(); delay(300);
      moveStop();    delay(200);
      turnRight90();
      moveForward();
    }
    else if (leftDistance > DIS && middleDistance < DIS && rightDistance < DIS) {
      // quẹo trái 90°
      moveForward(); delay(300);
      moveStop();    delay(200);
      turnLeft90();
      moveForward();
    }
    else if (leftDistance < DIS && middleDistance < DIS && rightDistance > DIS) {
      // quẹo phải 90°
      moveForward(); delay(300);
      moveStop();    delay(200);
      turnRight90();
      moveForward();
    }
    else if (leftDistance < DIS && middleDistance < DIS && rightDistance < DIS) {
      // chặn 3 phía → dừng và quẹo trái 90°
      moveStop();
      turnRight180();
    }

    delay(10);
  }
}
