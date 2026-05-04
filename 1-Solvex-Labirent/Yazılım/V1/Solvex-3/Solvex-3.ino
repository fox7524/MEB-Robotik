#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static const uint32_t SERIAL_BAUD = 115200;

#if defined(PA0) && defined(PB5)
static const int PIN_START_BTN = PB5;
static const int PIN_QTR1A = PA0;

static const int HCSR_FRONT_ECHO = PA4;
static const int HCSR_LEFT_ECHO  = PA5;
static const int HCSR_RIGHT_ECHO = PA6;

static const int HCSR_TRIG = -1;
static const int HCSR_FRONT_TRIG = PA1;
static const int HCSR_LEFT_TRIG  = PA2;
static const int HCSR_RIGHT_TRIG = PA3;

static const int R_IN1 = PB12;
static const int R_IN2 = PB13;
static const int L_IN1 = PB14;
static const int L_IN2 = PB15;

static const int PIN_ENC_L_A = PB10;
static const int PIN_ENC_R_A = PB11;
static const int PIN_ENC_L_B = PB8;
static const int PIN_ENC_R_B = PB9;
#else
static const int PIN_START_BTN = 7;
static const int PIN_QTR1A = A0;

static const int HCSR_FRONT_ECHO = 2;
static const int HCSR_LEFT_ECHO  = 3;
static const int HCSR_RIGHT_ECHO = 4;

static const int HCSR_TRIG = 6;
static const int HCSR_FRONT_TRIG = HCSR_TRIG;
static const int HCSR_LEFT_TRIG  = HCSR_TRIG;
static const int HCSR_RIGHT_TRIG = HCSR_TRIG;

static const int R_IN1 = 15;
static const int R_IN2 = 16;
static const int L_IN1 = 18;
static const int L_IN2 = 19;

static const int PIN_ENC_L_A = -1;
static const int PIN_ENC_R_A = -1;
static const int PIN_ENC_L_B = -1;
static const int PIN_ENC_R_B = -1;
#endif

static const int QTR_MARGIN = 250;

static const unsigned long HCSR_ECHO_TIMEOUT_US = 15000;
static const unsigned long HCSR_INTER_PING_MS = 18;
static const int WALL_THRESHOLD_CM = 18;

static const int R_PWM = -1;
static const int L_PWM = -1;
static const long CELL_TICKS = 0;
static const unsigned long CELL_TRAVEL_MS_FALLBACK = 700;

static const unsigned long TURN_TIMEOUT_MS = 1500;
static const int TURN_90_MS_FALLBACK = 350;
static const float TURN_TOL_DEG = 3.0f;

static const uint8_t MPU_ADDR = 0x68;
static bool g_imuOk = false;
static float g_gyroBiasZ = 0.0f;
static float g_yawDeg = 0.0f;
static unsigned long g_lastYawUs = 0;

static volatile long g_encL = 0;
static volatile long g_encR = 0;

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
static int g_prevX = 0;
static int g_prevY = 0;

static int g_qtrBaseline = 0;
static int g_startIdleLevel = HIGH;
static bool g_started = false;
static long g_cellTicks = CELL_TICKS;
static uint8_t g_finishStreak = 0;
static const bool ENABLE_QTR_FINISH = true;
static const unsigned long FINISH_CREEP_MS = 250;

void ileri();
void geri();
void sag360();
void sol360();
void sag();
void sol();

static void motorStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  if (L_PWM >= 0) analogWrite(L_PWM, 0);
  if (R_PWM >= 0) analogWrite(R_PWM, 0);
}

static void isrEncL() {
  if (PIN_ENC_L_B >= 0) {
    g_encL += (digitalRead(PIN_ENC_L_B) == HIGH) ? 1 : -1;
  } else {
    g_encL++;
  }
}

static void isrEncR() {
  if (PIN_ENC_R_B >= 0) {
    g_encR += (digitalRead(PIN_ENC_R_B) == HIGH) ? 1 : -1;
  } else {
    g_encR++;
  }
}

static void resetEncoders() {
  noInterrupts();
  g_encL = 0;
  g_encR = 0;
  interrupts();
}

static long encL() {
  noInterrupts();
  long v = g_encL;
  interrupts();
  return v;
}

static long encR() {
  noInterrupts();
  long v = g_encR;
  interrupts();
  return v;
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
  const int n = 600;
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

static unsigned long hcsrReadEchoUs(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, HCSR_ECHO_TIMEOUT_US);
}

static int hcsrReadCmOnce(int trigPin, int echoPin) {
  unsigned long us = hcsrReadEchoUs(trigPin, echoPin);
  if (us == 0) return -1;
  return (int)(us / 58);
}

static int hcsrReadCmFast2(int trigPin, int echoPin) {
  int a = hcsrReadCmOnce(trigPin, echoPin);
  delay(HCSR_INTER_PING_MS);
  int b = hcsrReadCmOnce(trigPin, echoPin);
  if (a < 0 || b < 0) return -1;
  return (a < b) ? a : b;
}

static bool wallFromCm(int cm) {
  if (cm <= 0) return true;
  return cm < WALL_THRESHOLD_CM;
}

static bool isFinish() {
  if (!ENABLE_QTR_FINISH) return false;
  int v = analogRead(PIN_QTR1A);
  bool white = v > (g_qtrBaseline + QTR_MARGIN);
  if (white) {
    if (g_finishStreak < 255) g_finishStreak++;
  } else {
    g_finishStreak = 0;
  }
  return g_finishStreak >= 3;
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
      if (score < bestScore) {
        bestScore = score;
        bestX = x;
        bestY = y;
      }
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
    if (diff == 1 || diff == 3) score += 10;
    if (diff == 2) score += 20;
    if (score < bestScore) {
      bestScore = score;
      bestDir = dir;
    }
  }
  return bestDir;
}

static void turnRelativeDeg(float deg) {
  if (!g_imuOk) {
    if (deg > 0) { sag360(); delay(TURN_90_MS_FALLBACK); motorStop(); }
    else { sol360(); delay(TURN_90_MS_FALLBACK); motorStop(); }
    return;
  }
  float start = yawDeg();
  float target = start + deg;
  unsigned long t0 = millis();
  if (deg > 0) sag360(); else sol360();
  while (true) {
    float y = yawDeg();
    float err = target - y;
    if (fabsf(err) <= TURN_TOL_DEG) break;
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
  else { turnRelativeDeg(+180.0f); }
  g_heading = next & 3;
}

static void markBackEdgeOpen() {
  static const uint8_t wb[4] = {NORTH, EAST, SOUTH, WEST};
  int back = (g_heading + 2) & 3;
  markKnownEdge(g_x, g_y, wb[back]);
}

static void driveOneCell() {
  g_prevX = g_x;
  g_prevY = g_y;

  long useTicks = g_cellTicks;
  if (useTicks > 0 && PIN_ENC_L_A >= 0 && PIN_ENC_R_A >= 0) {
    resetEncoders();
    ileri();
    unsigned long t0 = millis();
    while ((millis() - t0) <= 2500) {
      long l = labs(encL());
      long r = labs(encR());
      if (l >= useTicks && r >= useTicks) break;
      delay(2);
    }
    motorStop();
  } else {
    long l0 = labs(encL());
    long r0 = labs(encR());
    ileri();
    delay(CELL_TRAVEL_MS_FALLBACK);
    motorStop();
    long l1 = labs(encL());
    long r1 = labs(encR());
    long dl = l1 - l0;
    long dr = r1 - r0;
    long est = (dl > 0 && dr > 0) ? ((dl + dr) / 2) : 0;
    if (est > 0 && g_cellTicks == 0) g_cellTicks = est;
  }

  static const int dx[4] = {0, 1, 0, -1};
  static const int dy[4] = {1, 0, -1, 0};
  int nx = g_x + dx[g_heading & 3];
  int ny = g_y + dy[g_heading & 3];
  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) {
    motorStop();
    geri();
    delay(250);
    motorStop();
    g_heading = (g_heading + 2) & 3;
    return;
  }
  g_x = nx;
  g_y = ny;
  markBackEdgeOpen();
}

static void senseAndMap(bool& wF, bool& wL, bool& wR, int& cmF) {
  cmF = hcsrReadCmFast2(HCSR_FRONT_TRIG, HCSR_FRONT_ECHO);
  delay(HCSR_INTER_PING_MS);
  int cmL = hcsrReadCmFast2(HCSR_LEFT_TRIG, HCSR_LEFT_ECHO);
  delay(HCSR_INTER_PING_MS);
  int cmR = hcsrReadCmFast2(HCSR_RIGHT_TRIG, HCSR_RIGHT_ECHO);

  wF = wallFromCm(cmF);
  wL = wallFromCm(cmL);
  wR = wallFromCm(cmR);
  updateWallsFromSensors(wF, wL, wR);
}

static bool stepExplore() {
  g_visit[g_x][g_y]++;

  bool wF = true, wL = true, wR = true;
  int cmF = 500;
  senseAndMap(wF, wL, wR, cmF);

  if (isFinish()) return true;

  int tx = -1, ty = -1;
  if (!pickExplorationTarget(tx, ty)) {
    motorStop();
    geri();
    delay(200);
    motorStop();
    sag360();
    delay(120);
    motorStop();
    return false;
  }
  int next = chooseNextHeadingToTarget(tx, ty);
  if (next < 0) {
    motorStop();
    sag360();
    delay(120);
    motorStop();
    return false;
  }

  bool lastFrontWall = wF;
  for (int tries = 0; tries < 3; tries++) {
    faceHeading(next);
    bool fw = true, lw = true, rw = true;
    int cmTmp = 500;
    senseAndMap(fw, lw, rw, cmTmp);
    lastFrontWall = fw;
    if (isFinish()) return true;
    if (!fw) break;
    if (!pickExplorationTarget(tx, ty)) break;
    next = chooseNextHeadingToTarget(tx, ty);
    if (next < 0) break;
  }

  if (lastFrontWall) {
    motorStop();
    sag360();
    delay(120);
    motorStop();
    return false;
  }

  driveOneCell();
  return false;
}

static void calibrateQtrBaseline() {
  long sum = 0;
  const int n = 200;
  for (int i = 0; i < n; i++) {
    sum += analogRead(PIN_QTR1A);
    delay(5);
  }
  g_qtrBaseline = (int)(sum / n);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_START_BTN, INPUT_PULLUP);
  {
    int ones = 0;
    const int n = 40;
    for (int i = 0; i < n; i++) {
      ones += (digitalRead(PIN_START_BTN) == HIGH) ? 1 : 0;
      delay(2);
    }
    g_startIdleLevel = (ones > (n / 2)) ? HIGH : LOW;
  }

  pinMode(PIN_QTR1A, INPUT);

  pinMode(HCSR_FRONT_ECHO, INPUT);
  pinMode(HCSR_LEFT_ECHO, INPUT);
  pinMode(HCSR_RIGHT_ECHO, INPUT);

  pinMode(HCSR_FRONT_TRIG, OUTPUT);
  digitalWrite(HCSR_FRONT_TRIG, LOW);
  if (HCSR_LEFT_TRIG != HCSR_FRONT_TRIG) {
    pinMode(HCSR_LEFT_TRIG, OUTPUT);
    digitalWrite(HCSR_LEFT_TRIG, LOW);
  }
  if (HCSR_RIGHT_TRIG != HCSR_FRONT_TRIG && HCSR_RIGHT_TRIG != HCSR_LEFT_TRIG) {
    pinMode(HCSR_RIGHT_TRIG, OUTPUT);
    digitalWrite(HCSR_RIGHT_TRIG, LOW);
  }

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  if (R_PWM >= 0) pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  if (L_PWM >= 0) pinMode(L_PWM, OUTPUT);
  motorStop();

  if (PIN_ENC_L_A >= 0 && PIN_ENC_R_A >= 0) {
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    if (PIN_ENC_L_B >= 0) pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    if (PIN_ENC_R_B >= 0) pinMode(PIN_ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);
  }

  Wire.begin();
  Wire.setClock(400000);
  g_imuOk = imuInit();

  memset(g_walls, 0, sizeof(g_walls));
  memset(g_known, 0, sizeof(g_known));
  memset(g_visit, 0, sizeof(g_visit));

  calibrateQtrBaseline();

  while (digitalRead(PIN_START_BTN) == g_startIdleLevel) {
    motorStop();
    delay(5);
  }
  delay(250);

  motorSet(200, 200);
  delay(250);
  motorStop();

  g_started = true;
}

void loop() {
  if (!g_started) {
    motorStop();
    delay(10);
    return;
  }

  bool done = stepExplore();
  if (done) {
    ileri();
    delay(FINISH_CREEP_MS);
    motorStop();
    unsigned long t0 = millis();
    while ((millis() - t0) < 5000) {
      motorStop();
      delay(10);
    }
    while (true) {
      motorStop();
      delay(50);
    }
  }
}

static const int PWM_MAX = 255;
static const int DRIVE_PWM = 220;
static const int TURN_PWM = 200;

static void motorSet(int left, int right) {
  left = constrain(left, -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

  if (left == 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    if (L_PWM >= 0) analogWrite(L_PWM, 0);
  } else if (left > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    if (L_PWM >= 0) analogWrite(L_PWM, left);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    if (L_PWM >= 0) analogWrite(L_PWM, -left);
  }

  if (right == 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    if (R_PWM >= 0) analogWrite(R_PWM, 0);
  } else if (right > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    if (R_PWM >= 0) analogWrite(R_PWM, right);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    if (R_PWM >= 0) analogWrite(R_PWM, -right);
  }
}

void ileri() { motorSet(DRIVE_PWM, DRIVE_PWM); }
void geri() { motorSet(-DRIVE_PWM, -DRIVE_PWM); }
void sag360() { motorSet(TURN_PWM, -TURN_PWM); }
void sol360() { motorSet(-TURN_PWM, TURN_PWM); }
void sag() { motorSet(0, DRIVE_PWM); }
void sol() { motorSet(DRIVE_PWM, 0); }
