#include <Arduino.h>

#define ROWS 3
#define COLS 3
#define MAX_QUEUE 200

int matrix[ROWS][COLS];

struct Point {
  int row;
  int col;
};

Point queue[MAX_QUEUE];
int front = 0, rear = 0;

void enqueue(Point p) {
  if ((rear + 1) % MAX_QUEUE != front) {
    queue[rear] = p;
    rear = (rear + 1) % MAX_QUEUE;
  }
}

Point dequeue() {
  Point p = queue[front];
  front = (front + 1) % MAX_QUEUE;
  return p;
}

bool isQueueEmpty() {
  return front == rear;
}

void bfs(int destRow, int destCol) {
  for (int i = 0; i < ROWS; i++)
    for (int j = 0; j < COLS; j++)
      matrix[i][j] = -1;

  matrix[destRow][destCol] = 0;
  enqueue({destRow, destCol});

  int dRow[] = {-1, 0, 1, 0};
  int dCol[] = {0, 1, 0, -1};

  while (!isQueueEmpty()) {
    Point p = dequeue();
    for (int i = 0; i < 4; i++) {
      int newRow = p.row + dRow[i];
      int newCol = p.col + dCol[i];
      if (newRow >= 0 && newRow < ROWS && newCol >= 0 && newCol < COLS && matrix[newRow][newCol] == -1) {
        matrix[newRow][newCol] = matrix[p.row][p.col] + 1;
        enqueue({newRow, newCol});
      }
    }
  }
}

void printMatrix() {
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      Serial.print(matrix[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println("---------------------------------");
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  // TỰ CHỌN tọa độ đích và xuất phát
  int goalRow = 1;
  int goalCol = 2;

  int startRow = 2;
  int startCol = 0;

  bfs(goalRow, goalCol);

  Serial.print("Đích (goal): (");
  Serial.print(goalRow);
  Serial.print(", ");
  Serial.print(goalCol);
  Serial.println(") = 0");

  Serial.print("Xuất phát: (");
  Serial.print(startRow);
  Serial.print(", ");
  Serial.print(startCol);
  Serial.print(") = ");
  Serial.println(matrix[startRow][startCol]);

  printMatrix();
}

void loop() {
}
