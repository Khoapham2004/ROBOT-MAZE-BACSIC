#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
bool justTurned180 = false;

// Cảm biến siêu âm
int trig1 = 6, echo1 = 7; 
int trig2 = 2, echo2 = 4;
int trig3 = 8, echo3 = 9;

#define BTN_LEFT 10  // D10: nút chọn bám trái
#define BTN_RIGHT A6   // A6: nút chọn bám phải
int mode = 0;         // 0: chưa chọn, 1: bám trái, 2: bám phải

// Motor driver
#define AIN1 A0
#define AIN2 A1
#define PWMA 3
#define BIN1 A2
#define BIN2 A3
#define PWMB 5

float speedSound = 0.0343;
int DIS = 10;///////////////////////////
int speed = 100;////////////////////////

long leftDistance, middleDistance, rightDistance;
float yaw = 0;
unsigned long lastTime;

// Đọc khoảng cách
long measureDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return (pulseIn(echo, HIGH) / 2.0) * speedSound;
}
// Dừng
void moveStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}
// Cập nhật góc
void updateYaw() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  yaw += (gz / 131.0) * dt;
}

// Quay trái 90°
void turnLeft90() {
  updateYaw();
  float start = yaw;
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  while (abs(yaw - start) < 80) updateYaw();//////////////////////////////
  moveStop(); delay(200);
}

// Quay phải 90°
void turnRight90() {
  updateYaw();
  float start = yaw;
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  while (abs(yaw - start) < 80) updateYaw();////////////////////////////////
  moveStop(); delay(200);
}
// Quay phải 90°
void turnRight180() {
  updateYaw();
  float start = yaw;
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  while (abs(yaw - start) < 170) updateYaw();////////////////////////////////
  moveStop(); delay(200);
}

// Di chuyển thẳng
void moveForward() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed);
}

// Bám tường phải
void moveForwardR() {
long r = measureDistance(trig3, echo3);
if (r < 7) {
  analogWrite(PWMA, speed / 2); analogWrite(PWMB, speed);
} else if (r > 8) {
  analogWrite(PWMA, speed); analogWrite(PWMB, speed / 2);
} else {
  analogWrite(PWMA, speed); analogWrite(PWMB, speed);
}
digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

// Bám tường trái
void moveForwardL() {
long l = measureDistance(trig1, echo1);
if (l < 7) {
  analogWrite(PWMA, speed); analogWrite(PWMB, speed / 2);
} else if (l > 8) {
  analogWrite(PWMA, speed / 2); analogWrite(PWMB, speed);
} else {
  analogWrite(PWMA, speed); analogWrite(PWMB, speed);
}
digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
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
yaw = 0;

pinMode(BTN_RIGHT, INPUT_PULLUP);
pinMode(BTN_LEFT, INPUT_PULLUP);

pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);

pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

moveStop();
Serial.println(">> ĐANG CHỜ CHỌN HƯỚNG BÁM...");
}

void loop() {

while (mode == 0) {
if (digitalRead(BTN_LEFT) == LOW) {
  mode = 1;  // bám trái
  Serial.println(">> Đã chọn: BÁM TRÁI");
  delay(500);
} else if (analogRead(BTN_RIGHT) < 255/2) {
  // giá trị < ~300 khi nhấn nếu dùng trở kéo xuống đất
  mode = 2;  // bám phải
  Serial.println(">> Đã chọn: BÁM PHẢI");
  delay(500);
}
}


// Chờ phát hiện tường 2 bên
Serial.println(">> ĐÃ CHỌN – CHỜ TƯỜNG 2 BÊN...");
while (true) {
  leftDistance = measureDistance(trig1, echo1);
  rightDistance = measureDistance(trig3, echo3);
  if (leftDistance < 25 && rightDistance < 25) break;
  delay(200);
}

Serial.println(">> BẮT ĐẦU DI CHUYỂN!");

// Vòng điều khiển chính
while (true) {
  updateYaw();

  leftDistance = measureDistance(trig1, echo1);
  delay(10);
  middleDistance = measureDistance(trig2, echo2);
  delay(10);
  rightDistance = measureDistance(trig3, echo3);
  delay(10);

  Serial.print("L:"); Serial.print(leftDistance);
  Serial.print(" M:"); Serial.print(middleDistance);
  Serial.print(" R:"); Serial.println(rightDistance);
  if (justTurned180) {
    if (leftDistance < DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForward();
      if (leftDistance < 8) {
        analogWrite(PWMA, 100); analogWrite(PWMB, 80);
      } else if ( rightDistance < 8) {
        analogWrite(PWMA, 80); analogWrite(PWMB, 100);
      }
    // Sau khi quay đầu, chỉ xét quẹo trái/phải khi một bên thông thoáng
     } else if (leftDistance > DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnLeft90(); moveForward();
      justTurned180 = false; // reset cờ sau khi đã quẹo
    } else if (leftDistance < DIS && middleDistance > DIS && rightDistance > DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnRight90(); moveForward();
      justTurned180 = false;
    } else {
      moveForward(); // nếu chưa rõ, vẫn tiếp tục tiến
    }
  
  } else {
    // Các điều kiện bình thường
    if (leftDistance < DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForward();
      if (leftDistance < rightDistance && leftDistance < 8) {
        analogWrite(PWMA, 100); analogWrite(PWMB, 80);
      } else if (rightDistance < leftDistance && rightDistance < 8) {
        analogWrite(PWMA, 80); analogWrite(PWMB, 100);
      }
    } else if (leftDistance > DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForwardR();
    } else if (leftDistance < DIS && middleDistance > DIS && rightDistance > DIS) {
      moveForwardL();
  
    } else if (leftDistance > DIS && middleDistance > DIS && rightDistance < DIS) {
      moveForward();
      delay(200); moveStop(); delay(200);
      turnLeft90(); moveForward();
  
    } else if (leftDistance < DIS && middleDistance > DIS && rightDistance > DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnRight90(); moveForward();
  
    } else if (mode == 2 && leftDistance > DIS && middleDistance < DIS && rightDistance > DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnRight90(); moveForward();
  
    } else if (mode == 1 && leftDistance > DIS && middleDistance < DIS && rightDistance > DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnLeft90(); moveForward();
      justTurned180 = true;
  
    } else if (leftDistance > DIS && middleDistance < DIS && rightDistance < DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnLeft90(); moveForward();
      justTurned180 = true;
  
    } else if (leftDistance < DIS && middleDistance < DIS && rightDistance > DIS) {
      moveForward();
      delay(400); moveStop(); delay(200);
      turnRight90(); moveForward();
  
    } else if (leftDistance < DIS && middleDistance < DIS && rightDistance < DIS) {
      moveStop(); delay(200);
      turnRight180();
      justTurned180 = true;  // Cờ được bật
    }
  }  
 }
}

