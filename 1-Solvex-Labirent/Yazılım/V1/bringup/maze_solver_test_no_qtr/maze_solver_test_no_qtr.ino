#include <Arduino.h>
#include <Wire.h>

static const uint32_t SERIAL_BAUD = 115200;

static const int R_IN1 = PB12;
static const int R_IN2 = PB13;
static const int L_IN1 = PB14;
static const int L_IN2 = PB15;

static const int HCSR_FRONT_TRIG = PA1;
static const int HCSR_LEFT_TRIG  = PA2;
static const int HCSR_RIGHT_TRIG = PA3;
static const int HCSR_FRONT_ECHO = PA4;
static const int HCSR_LEFT_ECHO  = PA5;
static const int HCSR_RIGHT_ECHO = PA6;

static const unsigned long HCSR_ECHO_TIMEOUT_US = 15000;
static const unsigned long HCSR_INTER_PING_MS = 18;
static const int WALL_THRESHOLD_CM = 18;

static const unsigned long TURN_90_MS = 350;
static const unsigned long CELL_MS = 700;

static const uint8_t MPU_ADDR = 0x68;
static bool g_imuOk = false;
static float g_gyroBiasZ = 0.0f;
static float g_yawDeg = 0.0f;
static unsigned long g_lastYawUs = 0;
static const float TURN_TOL_DEG = 4.0f;
static const unsigned long TURN_TIMEOUT_MS = 1200;

static const int MAZE_W = 8;
static const int MAZE_H = 16;
static const uint8_t NORTH = 1;
static const uint8_t EAST  = 2;
static const uint8_t SOUTH = 4;
static const uint8_t WEST  = 8;

static uint8_t g_walls[MAZE_W][MAZE_H];
static uint8_t g_known[MAZE_W][MAZE_H];
static uint16_t g_visit[MAZE_W][MAZE_H];
static uint8_t g_dist[MAZE_W][MAZE_H];
static int g_qx[MAZE_W * MAZE_H];
static int g_qy[MAZE_W * MAZE_H];

static int g_x = 0;
static int g_y = 0;
static int g_heading = 0;

static void motorStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

static void motorFwd() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
}

static void motorRev() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
}

static void motorPivotRight() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
}

static void motorPivotLeft() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t* out, int n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)addr, n);
  if (got != n) return false;
  for (int i = 0; i < n; i++) out[i] = (uint8_t)Wire.read();
  return true;
}

static bool imuInit() {
  if (!i2cWrite8(MPU_ADDR, 0x6B, 0x00)) return false;
  if (!i2cWrite8(MPU_ADDR, 0x1B, 0x00)) return false;
  delay(50);
  float sum = 0.0f;
  const int n = 400;
  for (int i = 0; i < n; i++) {
    uint8_t b[2];
    if (!i2cReadN(MPU_ADDR, 0x47, b, 2)) return false;
    int16_t gz = (int16_t)((b[0] << 8) | b[1]);
    sum += (float)gz;
    delay(2);
  }
  g_gyroBiasZ = sum / (float)n;
  g_yawDeg = 0.0f;
  g_lastYawUs = micros();
  return true;
}

static void imuUpdateYaw() {
  if (!g_imuOk) return;
  unsigned long now = micros();
  unsigned long dtUs = now - g_lastYawUs;
  if (dtUs == 0) return;
  g_lastYawUs = now;
  uint8_t b[2];
  if (!i2cReadN(MPU_ADDR, 0x47, b, 2)) return;
  int16_t gz = (int16_t)((b[0] << 8) | b[1]);
  float gzDps = ((float)gz - g_gyroBiasZ) / 131.0f;
  g_yawDeg += gzDps * ((float)dtUs / 1000000.0f);
}

static float yawDeg() {
  imuUpdateYaw();
  return g_yawDeg;
}

static unsigned long hcsrEchoUs(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, HCSR_ECHO_TIMEOUT_US);
}

static int hcsrCmOnce(int trigPin, int echoPin) {
  unsigned long us = hcsrEchoUs(trigPin, echoPin);
  if (us == 0) return -1;
  return (int)(us / 58);
}

static int hcsrCmFast2(int trigPin, int echoPin) {
  int a = hcsrCmOnce(trigPin, echoPin);
  if (a < 0) a = 500;
  delay(HCSR_INTER_PING_MS);
  int b = hcsrCmOnce(trigPin, echoPin);
  if (b < 0) b = 500;
  return (a < b) ? a : b;
}

static bool wallFromCm(int cm) {
  if (cm <= 0) return true;
  return cm < WALL_THRESHOLD_CM;
}

static void markKnownEdge(int x, int y, uint8_t dir) {
  if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) return;
  g_known[x][y] |= dir;

  int nx = x;
  int ny = y;
  uint8_t opp = 0;
  if (dir == NORTH) { ny++; opp = SOUTH; }
  else if (dir == EAST) { nx++; opp = WEST; }
  else if (dir == SOUTH) { ny--; opp = NORTH; }
  else { nx--; opp = EAST; }

  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) return;
  g_known[nx][ny] |= opp;
}

static void setWall(int x, int y, uint8_t dir) {
  if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) return;
  g_walls[x][y] |= dir;

  int nx = x;
  int ny = y;
  uint8_t opp = 0;
  if (dir == NORTH) { ny++; opp = SOUTH; }
  else if (dir == EAST) { nx++; opp = WEST; }
  else if (dir == SOUTH) { ny--; opp = NORTH; }
  else { nx--; opp = EAST; }

  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) return;
  g_walls[nx][ny] |= opp;
}

static void updateWallsFromSensors(bool front, bool left, bool right) {
  static const uint8_t card[4] = {NORTH, EAST, SOUTH, WEST};
  uint8_t f = card[g_heading & 3];
  uint8_t l = card[(g_heading + 3) & 3];
  uint8_t r = card[(g_heading + 1) & 3];

  markKnownEdge(g_x, g_y, f);
  markKnownEdge(g_x, g_y, l);
  markKnownEdge(g_x, g_y, r);

  if (front) setWall(g_x, g_y, f);
  if (left) setWall(g_x, g_y, l);
  if (right) setWall(g_x, g_y, r);
}

static bool edgeIsUsable(int x, int y, uint8_t dir) {
  if ((g_known[x][y] & dir) == 0) return false;
  if (g_walls[x][y] & dir) return false;
  return true;
}

static bool cellIsFrontier(int x, int y) {
  if ((g_known[x][y] & NORTH) == 0) return true;
  if ((g_known[x][y] & EAST) == 0) return true;
  if ((g_known[x][y] & SOUTH) == 0) return true;
  if ((g_known[x][y] & WEST) == 0) return true;
  return false;
}

static void bfsFromTargets(const int* tx, const int* ty, int tcount) {
  memset(g_dist, 255, sizeof(g_dist));
  int head = 0, tail = 0;
  for (int i = 0; i < tcount; i++) {
    int x = tx[i], y = ty[i];
    if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) continue;
    g_dist[x][y] = 0;
    g_qx[tail] = x;
    g_qy[tail] = y;
    tail++;
  }
  while (head < tail) {
    int x = g_qx[head];
    int y = g_qy[head];
    head++;
    uint8_t d = g_dist[x][y];
    if (d == 255) continue;
    if (y + 1 < MAZE_H && edgeIsUsable(x, y, NORTH) && g_dist[x][y + 1] > (uint8_t)(d + 1)) {
      g_dist[x][y + 1] = d + 1;
      g_qx[tail] = x;
      g_qy[tail] = y + 1;
      tail++;
    }
    if (x + 1 < MAZE_W && edgeIsUsable(x, y, EAST) && g_dist[x + 1][y] > (uint8_t)(d + 1)) {
      g_dist[x + 1][y] = d + 1;
      g_qx[tail] = x + 1;
      g_qy[tail] = y;
      tail++;
    }
    if (y - 1 >= 0 && edgeIsUsable(x, y, SOUTH) && g_dist[x][y - 1] > (uint8_t)(d + 1)) {
      g_dist[x][y - 1] = d + 1;
      g_qx[tail] = x;
      g_qy[tail] = y - 1;
      tail++;
    }
    if (x - 1 >= 0 && edgeIsUsable(x, y, WEST) && g_dist[x - 1][y] > (uint8_t)(d + 1)) {
      g_dist[x - 1][y] = d + 1;
      g_qx[tail] = x - 1;
      g_qy[tail] = y;
      tail++;
    }
  }
}

static bool pickExplorationTarget(int& outTx, int& outTy) {
  static uint8_t distFromCur[MAZE_W][MAZE_H];
  memset(distFromCur, 255, sizeof(distFromCur));
  int head = 0, tail = 0;
  distFromCur[g_x][g_y] = 0;
  g_qx[tail] = g_x;
  g_qy[tail] = g_y;
  tail++;
  while (head < tail) {
    int x = g_qx[head];
    int y = g_qy[head];
    head++;
    uint8_t d = distFromCur[x][y];
    if (d == 255) continue;
    if (y + 1 < MAZE_H && edgeIsUsable(x, y, NORTH) && distFromCur[x][y + 1] > (uint8_t)(d + 1)) {
      distFromCur[x][y + 1] = d + 1;
      g_qx[tail] = x;
      g_qy[tail] = y + 1;
      tail++;
    }
    if (x + 1 < MAZE_W && edgeIsUsable(x, y, EAST) && distFromCur[x + 1][y] > (uint8_t)(d + 1)) {
      distFromCur[x + 1][y] = d + 1;
      g_qx[tail] = x + 1;
      g_qy[tail] = y;
      tail++;
    }
    if (y - 1 >= 0 && edgeIsUsable(x, y, SOUTH) && distFromCur[x][y - 1] > (uint8_t)(d + 1)) {
      distFromCur[x][y - 1] = d + 1;
      g_qx[tail] = x;
      g_qy[tail] = y - 1;
      tail++;
    }
    if (x - 1 >= 0 && edgeIsUsable(x, y, WEST) && distFromCur[x - 1][y] > (uint8_t)(d + 1)) {
      distFromCur[x - 1][y] = d + 1;
      g_qx[tail] = x - 1;
      g_qy[tail] = y;
      tail++;
    }
  }

  int bestX = -1, bestY = -1;
  int bestScore = 1000000;
  for (int y = 0; y < MAZE_H; y++) {
    for (int x = 0; x < MAZE_W; x++) {
      uint8_t d = distFromCur[x][y];
      if (d == 255) continue;
      if (!cellIsFrontier(x, y)) continue;
      int score = (int)d * 50 + (int)g_visit[x][y];
      if (score < bestScore) { bestScore = score; bestX = x; bestY = y; }
    }
  }

  if (bestX < 0) return false;
  outTx = bestX;
  outTy = bestY;
  return true;
}

static int chooseNextHeadingToTarget(int tx, int ty) {
  int txx[1] = {tx};
  int tyy[1] = {ty};
  bfsFromTargets(txx, tyy, 1);

  static const int dx[4] = {0, 1, 0, -1};
  static const int dy[4] = {1, 0, -1, 0};
  static const uint8_t wb[4] = {NORTH, EAST, SOUTH, WEST};

  int bestDir = -1;
  int bestScore = 1000000;
  for (int dir = 0; dir < 4; dir++) {
    uint8_t e = wb[dir];
    if (!edgeIsUsable(g_x, g_y, e)) continue;
    int nx = g_x + dx[dir];
    int ny = g_y + dy[dir];
    if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
    uint8_t d = g_dist[nx][ny];
    if (d == 255) continue;
    int score = (int)d * 50 + (int)g_visit[nx][ny] * 5;
    int diff = (dir - (g_heading & 3) + 4) & 3;
    if (diff == 2) score += 20;
    if (diff == 1 || diff == 3) score += 10;
    if (score < bestScore) { bestScore = score; bestDir = dir; }
  }
  return bestDir;
}

static void turnRelativeDeg(float deg) {
  if (!g_imuOk) {
    if (deg > 0) motorPivotRight(); else motorPivotLeft();
    delay((deg < 0) ? TURN_90_MS : TURN_90_MS);
    motorStop();
    return;
  }

  float start = yawDeg();
  float target = start + deg;
  unsigned long t0 = millis();
  if (deg > 0) motorPivotRight(); else motorPivotLeft();
  while (true) {
    float y = yawDeg();
    float err = target - y;
    if (err < 0) err = -err;
    if (err <= TURN_TOL_DEG) break;
    if ((millis() - t0) > TURN_TIMEOUT_MS) break;
    delay(2);
  }
  motorStop();
}

static void faceHeading(int next) {
  int diff = (next - (g_heading & 3) + 4) & 3;
  if (diff == 0) return;
  if (diff == 1) turnRelativeDeg(+90.0f);
  else if (diff == 3) turnRelativeDeg(-90.0f);
  else turnRelativeDeg(+180.0f);
  g_heading = next & 3;
}

static void markBackEdgeOpen() {
  static const uint8_t wb[4] = {NORTH, EAST, SOUTH, WEST};
  int back = (g_heading + 2) & 3;
  markKnownEdge(g_x, g_y, wb[back]);
}

static void driveOneCell() {
  motorFwd();
  delay(CELL_MS);
  motorStop();

  static const int dx[4] = {0, 1, 0, -1};
  static const int dy[4] = {1, 0, -1, 0};
  int nx = g_x + dx[g_heading & 3];
  int ny = g_y + dy[g_heading & 3];
  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) {
    motorStop();
    motorRev();
    delay(300);
    motorStop();
    return;
  }
  g_x = nx;
  g_y = ny;
  markBackEdgeOpen();
}

static void senseAndMap(bool& wF, bool& wL, bool& wR) {
  int cmF = hcsrCmFast2(HCSR_FRONT_TRIG, HCSR_FRONT_ECHO);
  delay(HCSR_INTER_PING_MS);
  int cmL = hcsrCmFast2(HCSR_LEFT_TRIG, HCSR_LEFT_ECHO);
  delay(HCSR_INTER_PING_MS);
  int cmR = hcsrCmFast2(HCSR_RIGHT_TRIG, HCSR_RIGHT_ECHO);

  wF = wallFromCm(cmF);
  wL = wallFromCm(cmL);
  wR = wallFromCm(cmR);

  updateWallsFromSensors(wF, wL, wR);
}

static void stepExplore() {
  g_visit[g_x][g_y]++;

  bool wF = true, wL = true, wR = true;
  senseAndMap(wF, wL, wR);

  int tx = -1, ty = -1;
  if (!pickExplorationTarget(tx, ty)) {
    motorStop();
    delay(50);
    return;
  }

  int next = chooseNextHeadingToTarget(tx, ty);
  if (next < 0) {
    motorPivotRight();
    delay(120);
    motorStop();
    return;
  }

  for (int tries = 0; tries < 3; tries++) {
    faceHeading(next);
    bool fw = true, lw = true, rw = true;
    senseAndMap(fw, lw, rw);
    if (!fw) break;
    if (!pickExplorationTarget(tx, ty)) break;
    next = chooseNextHeadingToTarget(tx, ty);
    if (next < 0) break;
  }

  bool fw2 = true, lw2 = true, rw2 = true;
  senseAndMap(fw2, lw2, rw2);
  if (fw2) {
    motorPivotRight();
    delay(120);
    motorStop();
    return;
  }

  driveOneCell();
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  motorStop();

  pinMode(HCSR_FRONT_ECHO, INPUT);
  pinMode(HCSR_LEFT_ECHO, INPUT);
  pinMode(HCSR_RIGHT_ECHO, INPUT);

  pinMode(HCSR_FRONT_TRIG, OUTPUT);
  digitalWrite(HCSR_FRONT_TRIG, LOW);
  pinMode(HCSR_LEFT_TRIG, OUTPUT);
  digitalWrite(HCSR_LEFT_TRIG, LOW);
  pinMode(HCSR_RIGHT_TRIG, OUTPUT);
  digitalWrite(HCSR_RIGHT_TRIG, LOW);

  Wire.begin();
  Wire.setClock(400000);
  g_imuOk = imuInit();

  memset(g_walls, 0, sizeof(g_walls));
  memset(g_known, 0, sizeof(g_known));
  memset(g_visit, 0, sizeof(g_visit));

  delay(500);
  motorFwd();
  delay(200);
  motorStop();
}

void loop() {
  stepExplore();
}

