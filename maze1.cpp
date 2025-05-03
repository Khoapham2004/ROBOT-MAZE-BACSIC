// Thư viện cần thiết
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <limits.h>  // INT_MAX

// Kích thước ma trận con
const int SUB_R = 4;
const int SUB_C = 4;

// Khoảng cách chèn (scale)
const int SCALE = 2;

// Kích thước ma trận tổng
const int ROWS = SUB_R * SCALE + 1;
const int COLS = SUB_C * SCALE + 1;

// Vị trí chèn góc trên‑trái trong ma trận tổng
const int startRow = 1;
const int startCol = 1;

// Tọa độ đích và start trong ma trận con
const int goalSR = 2, goalSC = 2;
const int startSR = 3, startSC = 0;

// Cell cho queue flood-fill submatrix
typedef struct { int r, c; } Cell;

// Ma trận nền và hiển thị
int matrix[ROWS][COLS] = {0};
char display[ROWS][COLS];

// HC-SR04 pins
typedef struct { int trig, echo; } HCSR04;
HCSR04 sensors[3] = {
  {6, 7},  // Left
  {2, 4},  // Mid
  {8, 9}   // Right
};

// Motor driver pins
#define AIN1 A0
#define AIN2 A1
#define PWMA 3
#define BIN1 A2
#define BIN2 A3
#define PWMB 5
const int speed = 120;

// Hướng và vị trí robot
enum Direction { NORTH=0, EAST, SOUTH, WEST };
int robotDir = NORTH;
int robotR = startRow + startSR * SCALE;
int robotC = startCol + startSC * SCALE;

// Ngưỡng chướng ngại vật (cm)
const float obstacleThreshold = 15.0;

// Di chuyển vector
const int dr[4] = { -1, 0, 1,  0 };
const int dc[4] = {  0, 1, 0, -1 };

// MPU6050 để đo yaw
MPU6050 mpu;
float yaw = 0;
unsigned long lastTime = 0;

// Prototype
float getAverageDistance(int trigPin, int echoPin);
void moveStop();
void updateObstacles();
void updateYaw();
void turnLeft90();
void turnRight90();
void moveForward();
void printMatrix();
void floodFillSubmatrix(int srSub, int scSub);
void floodFillUpdate(int r, int c);

void floodFillSubmatrix(int srSub, int scSub) {
  static Cell q[SUB_R * SUB_C];
  bool visited[SUB_R][SUB_C] = {false};
  int head = 0, tail = 0;
  q[tail++] = { srSub, scSub };
  visited[srSub][scSub] = true;
  while (head < tail) {
    Cell cur = q[head++];
    int rS = cur.r, cS = cur.c;
    int bR = startRow + rS * SCALE;
    int bC = startCol + cS * SCALE;
    int minv = INT_MAX;
    int maxv = 0;
    int accessibleCount = 0;
    // Đếm số hàng xóm có thể đi tới và tìm giá trị min/max
    for (int d = 0; d < 4; d++) {
      int nrS = rS + (d==0?-1:d==2?1:0);
      int ncS = cS + (d==1?1:d==3?-1:0);
      if (nrS >= 0 && nrS < SUB_R && ncS >= 0 && ncS < SUB_C) {
        int nbR = startRow + nrS * SCALE;
        int nbC = startCol + ncS * SCALE;
        if (display[nbR][nbC] != '#') {
          accessibleCount++;
          minv = min(minv, matrix[nbR][nbC]);
          maxv = max(maxv, matrix[nbR][nbC]);
        }
      }
    }
    // Cập nhật giá trị ô nếu bị chặn 3 hướng hoặc cần điều chỉnh
    if (accessibleCount <= 1 || matrix[bR][bC] <= minv) { // <= 1 nghĩa là bị chặn 3 hướng trở lên
      if (accessibleCount <= 1 && accessibleCount > 0) {
        matrix[bR][bC] = maxv - 1; // Đặt giá trị là max của hàng xóm trừ 1
      } else if (accessibleCount == 0) {
        matrix[bR][bC] = 1; // Đặt lại thành 1 nếu bị chặn hoàn toàn
      } else {
        matrix[bR][bC] = minv + 1;
      }
      for (int d = 0; d < 4; d++) {
        int nrS = rS + (d==0?-1:d==2?1:0);
        int ncS = cS + (d==1?1:d==3?-1:0);
        if (nrS >= 0 && nrS < SUB_R && ncS >= 0 && ncS < SUB_C && !visited[nrS][ncS]) {
          int nbR = startRow + nrS * SCALE;
          int nbC = startCol + ncS * SCALE;
          if (display[nbR][nbC] != '#') {
            visited[nrS][ncS] = true;
            q[tail++] = { nrS, ncS };
          }
        }
      }
    }
  }
}
void floodFillUpdate(int r, int c) {
  static Cell q[ROWS * COLS];
  bool visited[ROWS][COLS] = {false};
  int head = 0, tail = 0;
  q[tail++] = { r, c };
  visited[r][c] = true;

  while (head < tail) {
    Cell cur = q[head++];
    int curR = cur.r, curC = cur.c;

    // Check accessible neighboring cells
    int accessibleCount = 0;
    int maxNeighborValue = 0;
    for (int d = 0; d < 4; d++) {
      int nr = curR + (d==0?-1:d==2?1:0);
      int nc = curC + (d==1?1:d==3?-1:0);
      if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && display[nr][nc] != '#') {
        accessibleCount++;
        maxNeighborValue = max(maxNeighborValue, matrix[nr][nc]);
      }
    }

    // Update the current cell if blocked or needs adjustment
    if (accessibleCount <= 1) {
      if (accessibleCount == 0) {
        matrix[curR][curC] = INT_MAX; // Mark as unreachable
      } else {
        matrix[curR][curC] = maxNeighborValue + 1; // Update based on max neighbor
      }

      // Propagate changes to neighbors
      for (int d = 0; d < 4; d++) {
        int nr = curR + (d==0?-1:d==2?1:0);
        int nc = curC + (d==1?1:d==3?-1:0);
        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !visited[nr][nc] && display[nr][nc] != '#') {
          q[tail++] = { nr, nc };
          visited[nr][nc] = true;
        }
      }
    }
  }
}
// Đo khoảng cách HC-SR04
float getAverageDistance(int trigPin, int echoPin) {
  float total = 0;
  const int samples = 3;
  for (int i=0; i<samples; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH);
    total += (duration * 0.0343f) / 2.0f;
    delay(10);
  }
  return total / samples;
}

// Dừng motor
void moveStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Cập nhật chướng ngại vật vào display
void updateObstacles() {
  float distLeft  = getAverageDistance(sensors[0].trig, sensors[0].echo);
  float distMid   = getAverageDistance(sensors[1].trig, sensors[1].echo);
  float distRight = getAverageDistance(sensors[2].trig, sensors[2].echo);
  int mR = robotR + dr[robotDir];
  int mC = robotC + dc[robotDir];
  if (distMid < obstacleThreshold && mR>=0 && mR<ROWS && mC>=0 && mC<COLS)
    display[mR][mC] = '#';
  int lD = (robotDir+3)%4;
  int lR = robotR + dr[lD]; int lC = robotC + dc[lD];
  if (distLeft < obstacleThreshold && lR>=0 && lR<ROWS && lC>=0 && lC<COLS)
    display[lR][lC] = '#';
  int rD = (robotDir+1)%4;
  int rR = robotR + dr[rD]; int rC = robotC + dc[rD];
  if (distRight < obstacleThreshold && rR>=0 && rR<ROWS && rC>=0 && rC<COLS)
    display[rR][rC] = '#';
}

// Cập nhật yaw từ MPU6050
void updateYaw() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  yaw += (gz / 131.0f) * dt;
}

// Quay trái 90°
void turnLeft90() {
  updateYaw(); float s=yaw;
  digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH);
  analogWrite(PWMA,speed); analogWrite(PWMB,speed);
  while(abs(yaw-s)<80.0f) updateYaw();
  moveStop(); robotDir=(robotDir+3)%4;
}

// Quay phải 90°
void turnRight90() {
  updateYaw(); float s=yaw;
  digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
  analogWrite(PWMA,speed); analogWrite(PWMB,speed);
  while(abs(yaw-s)<80.0f) updateYaw();
  moveStop(); robotDir=(robotDir+1)%4;
}

// Di chuyển tiến một ô
void moveForward() {
  display[robotR][robotC] = char('0' + matrix[robotR][robotC]);
  digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH);
  analogWrite(PWMA,speed); analogWrite(PWMB,speed);
  delay(600); moveStop(); delay(200);
  robotR += dr[robotDir] * SCALE;
  robotC += dc[robotDir] * SCALE;
  display[robotR][robotC] = 'S';
  updateObstacles();
}

// In ma trận lên Serial
void printMatrix() {
  Serial.println();
  for(int i=0; i<(COLS*2-5)/2; i++) Serial.print(' ');
  Serial.println("North");
  for(int i=0; i<ROWS; i++) {
    if(i==ROWS/2) Serial.print("West   "); else Serial.print("       ");
    for(int j=0; j<COLS; j++) {
      Serial.print(display[i][j]); Serial.print(' ');
    }
    if(i==ROWS/2) Serial.println("  East"); else Serial.println();
  }
  for(int i=0; i<(COLS*2-5)/2; i++) Serial.print(' ');
  Serial.println("South");
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Setup started...");
  Wire.begin(); mpu.initialize(); lastTime=millis();
  for(auto &s: sensors) pinMode(s.trig, OUTPUT), pinMode(s.echo, INPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  for(int i=0; i<SUB_R; i++) for(int j=0; j<SUB_C; j++)
    matrix[startRow+i*SCALE][startCol+j*SCALE] = abs(goalSR-i)+abs(goalSC-j);
  for(int i=0; i<ROWS; i++) for(int j=0; j<COLS; j++)
    display[i][j] = (matrix[i][j]==0?'.':char('0'+matrix[i][j]));
  display[startRow+goalSR*SCALE][startCol+goalSC*SCALE] = 'G';
  display[robotR][robotC] = 'S';
  updateObstacles();
  floodFillSubmatrix(goalSR, goalSC);
  printMatrix();
  Serial.println("Setup completed.");
}

void loop() {
  int goalR = startRow + goalSR * SCALE;
  int goalC = startCol + goalSC * SCALE;
  if (robotR == goalR && robotC == goalC) {
    moveStop();
    Serial.println("Goal Reached!");
    while (true);
  }

  int minVal = INT_MAX;
  int bestDir = -1;
  int backtrackDir = -1;
  for (int d = 0; d < 4; d++) {
    int nr = robotR + dr[d] * SCALE;
    int nc = robotC + dc[d] * SCALE;
    if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && display[nr][nc] != '#') {
      if (matrix[nr][nc] < minVal) {
        int goalDir = (d + 2) % 4;
        int backR = robotR + dr[goalDir] * SCALE;
        int backC = robotC + dc[goalDir] * SCALE;
        if (backR == goalR && backC == goalC) {
          backtrackDir = d;
        } else {
          minVal = matrix[nr][nc];
          bestDir = d;
        }
      }
    }
  }

  if (bestDir == -1 && backtrackDir != -1) {
    bestDir = backtrackDir;
  }

  if (bestDir == -1) {
    Serial.println("No path found, updating flood-fill...");
    floodFillSubmatrix((robotR - startRow) / SCALE, (robotC - startCol) / SCALE);
    printMatrix();
    return;
  }

  Serial.print("Turning to direction: ");
  Serial.println(bestDir);
  while (robotDir != bestDir) {
    if ((robotDir + 1) % 4 == bestDir) {
      turnRight90();
    } else if ((robotDir + 3) % 4 == bestDir) {
      turnLeft90();
    } else {
      turnRight90();
    }
  }

  Serial.println("Moving forward...");
  moveForward();
  floodFillSubmatrix((robotR - startRow) / SCALE, (robotC - startCol) / SCALE);
  printMatrix();
  delay(500);
}
