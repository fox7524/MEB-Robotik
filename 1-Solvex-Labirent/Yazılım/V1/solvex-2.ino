 #include <Arduino.h>
 
 static const int HCSR_TRIG = 6;
 static const int HCSR_FRONT_ECHO = 2;
 static const int HCSR_LEFT_ECHO = 3;
 static const int HCSR_RIGHT_ECHO = 4;
 
 static const int R_IN1 = 15;
 static const int R_IN2 = 16;
 static const int R_PWM = 17;
 static const int L_IN1 = 18;
 static const int L_IN2 = 19;
 static const int L_PWM = 20;
 
 static const int START_BTN = 7;
 static const int TURN_90_MS = 350;
 static const int CELL_TRAVEL_MS = 700;
 
 static const unsigned long SENSOR_TIMEOUT_US = 25000;
 static const int WALL_THRESHOLD_CM = 18;
 
 static void motorStop() {
   digitalWrite(L_IN1, LOW);
   digitalWrite(L_IN2, LOW);
   digitalWrite(R_IN1, LOW);
   digitalWrite(R_IN2, LOW);
   analogWrite(L_PWM, 0);
   analogWrite(R_PWM, 0);
 }
 
 void ileri(){
 digitalWrite(L_IN1, HIGH);
 digitalWrite(R_IN1, HIGH);
 
 
 digitalWrite(L_IN1, LOW);
 digitalWrite(R_IN1, LOW);
 
 
 analogWrite(L_PWM, 255);
 analogWrite(R_PWM, 255);
 
 
 }
 
 void geri(){
 digitalWrite(L_IN1, LOW);
 digitalWrite(R_IN1, LOW);
 
 
 digitalWrite(L_IN1, HIGH);
 digitalWrite(R_IN1, HIGH);
 
 
 analogWrite(L_PWM, 255);
 analogWrite(R_PWM, 255);
 
 
 }
 void sag360(){
 digitalWrite(L_IN1, HIGH);
 digitalWrite(R_IN1, LOW);
 
 digitalWrite(L_IN1, LOW);
 digitalWrite(R_IN1, HIGH);
 
 analogWrite(L_PWM, 255);
 analogWrite(R_PWM, 255);
 
     
 }
 void sol360(){
 
 digitalWrite(L_IN1, LOW);
 digitalWrite(R_IN1, HIGH);
 
 digitalWrite(L_IN1, HIGH);
 digitalWrite(R_IN1, LOW);
 
 analogWrite(L_PWM, 255);
 analogWrite(R_PWM, 255);
     
 }
 
 void sag(){
 digitalWrite(R_IN1, HIGH);
 digitalWrite(R_IN1, HIGH);
 
 analogWrite(R_PWM, 255);
     
 }
 void sol(){
 digitalWrite(L_IN1, HIGH);
 digitalWrite(L_IN1, HIGH);
 
 analogWrite(L_PWM, 255);
     
 }
 
 static unsigned long fireAndReadEchoUs(int echoPin) {
   digitalWrite(HCSR_TRIG, LOW);
   delayMicroseconds(2);
   digitalWrite(HCSR_TRIG, HIGH);
   delayMicroseconds(10);
   digitalWrite(HCSR_TRIG, LOW);
   return pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
 }
 
 static int readCm(int echoPin) {
   unsigned long us = fireAndReadEchoUs(echoPin);
   if (us == 0) return 999;
   return (int)(us / 58);
 }
 
 static bool wallFront() { return readCm(HCSR_FRONT_ECHO) < WALL_THRESHOLD_CM; }
 static bool wallLeft() { return readCm(HCSR_LEFT_ECHO) < WALL_THRESHOLD_CM; }
 static bool wallRight() { return readCm(HCSR_RIGHT_ECHO) < WALL_THRESHOLD_CM; }
 
 static const int MAZE_W = 8;
 static const int MAZE_H = 16;
 
 static const uint8_t NORTH = 1;
 static const uint8_t EAST = 2;
 static const uint8_t SOUTH = 4;
 static const uint8_t WEST = 8;
 
 static uint8_t distances[MAZE_W][MAZE_H];
 static uint8_t walls[MAZE_W][MAZE_H];
 
 static int currentX = 0;
 static int currentY = 0;
 static int currentHeading = 0;
 
 static int goalX[4] = {3, 3, 4, 4};
 static int goalY[4] = {7, 8, 7, 8};
 
 static int qx[MAZE_W * MAZE_H];
 static int qy[MAZE_W * MAZE_H];
 
 static void setWall(int x, int y, uint8_t dir) {
   walls[x][y] |= dir;
 
   int nx = x, ny = y;
   uint8_t opp = 0;
   if (dir == NORTH) { ny++; opp = SOUTH; }
   else if (dir == EAST) { nx++; opp = WEST; }
   else if (dir == SOUTH) { ny--; opp = NORTH; }
   else { nx--; opp = EAST; }
 
   if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
     walls[nx][ny] |= opp;
   }
 }
 
 static void updateWallsMap(bool f, bool l, bool r) {
   static const uint8_t card[4] = {NORTH, EAST, SOUTH, WEST};
   uint8_t fdir = card[currentHeading];
   uint8_t ldir = card[(currentHeading + 3) & 3];
   uint8_t rdir = card[(currentHeading + 1) & 3];
   if (f) setWall(currentX, currentY, fdir);
   if (l) setWall(currentX, currentY, ldir);
   if (r) setWall(currentX, currentY, rdir);
 }
 
 static void updateDistances() {
   memset(distances, 255, sizeof(distances));
   int head = 0, tail = 0;
 
   for (int i = 0; i < 4; i++) {
     int gx = goalX[i];
     int gy = goalY[i];
     if (gx < 0 || gx >= MAZE_W || gy < 0 || gy >= MAZE_H) continue;
     distances[gx][gy] = 0;
     qx[tail] = gx;
     qy[tail] = gy;
     tail++;
   }
 
   while (head < tail) {
     int x = qx[head];
     int y = qy[head];
     head++;
     uint8_t d = distances[x][y];
 
     if (y + 1 < MAZE_H && !(walls[x][y] & NORTH) && distances[x][y + 1] > (uint8_t)(d + 1)) {
       distances[x][y + 1] = d + 1;
       qx[tail] = x;
       qy[tail] = y + 1;
       tail++;
     }
     if (x + 1 < MAZE_W && !(walls[x][y] & EAST) && distances[x + 1][y] > (uint8_t)(d + 1)) {
       distances[x + 1][y] = d + 1;
       qx[tail] = x + 1;
       qy[tail] = y;
       tail++;
     }
     if (y - 1 >= 0 && !(walls[x][y] & SOUTH) && distances[x][y - 1] > (uint8_t)(d + 1)) {
       distances[x][y - 1] = d + 1;
       qx[tail] = x;
       qy[tail] = y - 1;
       tail++;
     }
     if (x - 1 >= 0 && !(walls[x][y] & WEST) && distances[x - 1][y] > (uint8_t)(d + 1)) {
       distances[x - 1][y] = d + 1;
       qx[tail] = x - 1;
       qy[tail] = y;
       tail++;
     }
   }
 }
 
 static int findLowestNeighbor() {
   static const int dx[4] = {0, 1, 0, -1};
   static const int dy[4] = {1, 0, -1, 0};
   static const uint8_t wb[4] = {NORTH, EAST, SOUTH, WEST};
 
   int bestDir = -1;
   uint8_t best = 255;
 
   for (int dir = 0; dir < 4; dir++) {
     if (walls[currentX][currentY] & wb[dir]) continue;
     int nx = currentX + dx[dir];
     int ny = currentY + dy[dir];
     if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
     uint8_t nd = distances[nx][ny];
     if (nd < best) {
       best = nd;
       bestDir = dir;
     }
   }
   return bestDir;
 }
 
 static void turnToHeading(int nextHeading) {
   int diff = (nextHeading - currentHeading + 4) & 3;
  if (diff == 1) {
    sag360();
    delay(TURN_90_MS);
    motorStop();
  } else if (diff == 3) {
    sol360();
    delay(TURN_90_MS);
    motorStop();
  } else if (diff == 2) {
    sag360();
    delay(TURN_90_MS);
    motorStop();
    delay(30);
    sag360();
    delay(TURN_90_MS);
    motorStop();
  }
   currentHeading = nextHeading;
 }
 
 static void updatePositionAfterForward() {
   static const int dx[4] = {0, 1, 0, -1};
   static const int dy[4] = {1, 0, -1, 0};
   currentX += dx[currentHeading];
   currentY += dy[currentHeading];
 }
 
 static bool isAtGoal() {
   for (int i = 0; i < 4; i++) {
     if (currentX == goalX[i] && currentY == goalY[i]) return true;
   }
   return false;
 }
 
 static void search_algorithm() {
   bool f = wallFront();
   bool l = wallLeft();
   bool r = wallRight();
 
   updateWallsMap(f, l, r);
   updateDistances();
 
   if (isAtGoal()) {
     motorStop();
     while (true) delay(50);
   }
 
   int nextH = findLowestNeighbor();
   if (nextH < 0) {
     motorStop();
     while (true) delay(50);
   }
 
   turnToHeading(nextH);
   ileri();
  delay(CELL_TRAVEL_MS);
  motorStop();
   updatePositionAfterForward();
 
   Serial.print(currentX);
   Serial.print(',');
   Serial.print(currentY);
   Serial.print(" d=");
   Serial.println(distances[currentX][currentY]);
 }
 
 void setup() {
   Serial.begin(115200);
 
   pinMode(HCSR_TRIG, OUTPUT);
   digitalWrite(HCSR_TRIG, LOW);
   pinMode(HCSR_FRONT_ECHO, INPUT);
   pinMode(HCSR_LEFT_ECHO, INPUT);
   pinMode(HCSR_RIGHT_ECHO, INPUT);
 
   pinMode(R_IN1, OUTPUT);
   pinMode(R_IN2, OUTPUT);
   pinMode(R_PWM, OUTPUT);
   pinMode(L_IN1, OUTPUT);
   pinMode(L_IN2, OUTPUT);
   pinMode(L_PWM, OUTPUT);
 
   pinMode(START_BTN, INPUT_PULLUP);
 
   motorStop();
 
   memset(walls, 0, sizeof(walls));
   updateDistances();
 
   while (digitalRead(START_BTN) == HIGH) delay(10);
   delay(300);
 }
 
 void loop() {
   search_algorithm();
 }
