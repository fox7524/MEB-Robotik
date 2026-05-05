// --- Solvex-3.ino pinout (PA/PB ile bağlanan sistem) ---

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>



const int PIN_START_BTN = PB5;

const int PIN_QTR1A = PA0;

const int HFT = PA1;
const int HLT  = PA2;
const int HRT = PA3;

const int H_F_E = PA4;
const int H_L_E  = PA5;
const int H_R_E = PA6;


const int R_PWM = PB0;
const int L_PWM = PB1;
const int R_IN1 = PB12;
const int R_IN2 = PB13;
const int L_IN1 = PB14;
const int L_IN2 = PB15;

// Encoder (C1/C2 = A/B)
const int PIN_ENC_L_A = PB10; // C1
const int PIN_ENC_R_A = PB11; // C1
const int PIN_ENC_L_B = PB8;  // C2
const int PIN_ENC_R_B = PB9;  // C2

static constexpr uint8_t MAZE_W = 8;
static constexpr uint8_t MAZE_H = 16;

enum Dir : uint8_t { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };
static constexpr uint8_t DIR_BIT[4] = { 1u << 0, 1u << 1, 1u << 2, 1u << 3 };
static constexpr int8_t DX[4] = { 0, 1, 0, -1 };
static constexpr int8_t DY[4] = { -1, 0, 1, 0 };

struct Cell {
  uint8_t walls;
  uint8_t known;
  uint8_t visited;
};

static Cell g_maze[MAZE_H][MAZE_W];
static uint8_t g_x = 0;
static uint8_t g_y = 0;
static uint8_t g_dir = DIR_E;

static uint16_t g_front_cm = 0;
static uint16_t g_left_cm = 0;
static uint16_t g_right_cm = 0;
static uint8_t g_wall_front = 0;
static uint8_t g_wall_left = 0;
static uint8_t g_wall_right = 0;

static float g_yaw_deg = 0.0f;
static bool g_mpu_ok = false;
static bool g_search_inited = false;
static uint32_t g_last_sensor_ms = 0;
static uint8_t g_prev_x = 0;
static uint8_t g_prev_y = 0;
static bool g_have_prev = false;

static constexpr bool ENABLE_DIAGNOSTICS = false;
static constexpr bool ENABLE_SELFTEST = false;
static constexpr bool ENABLE_LED_DEBUG = true;
static constexpr bool ENABLE_MOTOR_SANITY = false;

static constexpr bool BYPASS_QTR1A = true;
static constexpr uint16_t QTR_WHITE_THRESHOLD = 2500;
static constexpr uint16_t WALL_THRESHOLD_CM = 14;

static constexpr int PIN_LED = PC13;

void ledInit() {
  if (!ENABLE_LED_DEBUG) return;
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

void ledOn() {
  if (!ENABLE_LED_DEBUG) return;
  digitalWrite(PIN_LED, LOW);
}

void ledOff() {
  if (!ENABLE_LED_DEBUG) return;
  digitalWrite(PIN_LED, HIGH);
}

void ledBlink(uint8_t n) {
  if (!ENABLE_LED_DEBUG) return;
  for (uint8_t i = 0; i < n; i++) {
    ledOn();
    delay(120);
    ledOff();
    delay(120);
  }
  delay(300);
}

static void selfTestOnce() {
  static bool done = false;
  if (done) return;
  done = true;

  if (!ENABLE_DIAGNOSTICS) return;

  Serial.println();
  Serial.println("SELFTEST START");

  uint16_t qtr = analogRead(PIN_QTR1A);
  Serial.print("QTR_RAW=");
  Serial.println(qtr);

  getmpu();
  Serial.print("YAW_DEG=");
  Serial.println(g_yaw_deg, 2);

  gethcsrf();
  gethcsrl();
  gethcsrr();
  Serial.print("HC_F_CM=");
  Serial.print(g_front_cm);
  Serial.print(" HC_L_CM=");
  Serial.print(g_left_cm);
  Serial.print(" HC_R_CM=");
  Serial.println(g_right_cm);

  bool okH = (g_front_cm > 0 || g_left_cm > 0 || g_right_cm > 0);
  Serial.print("HC_STATUS=");
  Serial.println(okH ? "OK" : "FAIL");

  Serial.println("SELFTEST END");
  Serial.println();
}

void setup() {
#if ENABLE_DIAGNOSTICS
Serial.begin(115200);
Serial.print("merheaba");
#endif
ledInit();
pinMode(H_F_E, INPUT);
pinMode(H_L_E, INPUT);
pinMode(H_R_E, INPUT);
pinMode(PIN_START_BTN, INPUT);
pinMode(PIN_QTR1A, INPUT);
pinMode(PIN_ENC_L_A, INPUT);
pinMode(PIN_ENC_R_A, INPUT);
pinMode(PIN_ENC_L_B, INPUT);
pinMode(PIN_ENC_R_B, INPUT);
pinMode(HFT, OUTPUT);
pinMode(HLT, OUTPUT);
pinMode(HRT, OUTPUT);
pinMode(R_IN1, OUTPUT);
pinMode(R_IN2, OUTPUT);
pinMode(L_IN1, OUTPUT);
pinMode(L_IN2, OUTPUT);
pinMode(L_PWM, OUTPUT);
pinMode(R_PWM, OUTPUT);

digitalWrite(HFT, LOW);
digitalWrite(HLT, LOW);
digitalWrite(HRT, LOW);

digitalWrite(L_IN1, LOW);
digitalWrite(L_IN2, LOW);
digitalWrite(R_IN1, LOW);
digitalWrite(R_IN2, LOW);
analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);



}

void loop() {
  static bool started = false;
  static int startLevel = -1;

  if (startLevel < 0) startLevel = digitalRead(PIN_START_BTN);

  if (!started) {
    int v = digitalRead(PIN_START_BTN);
    if (v != startLevel) {
      delay(20);
      if (digitalRead(PIN_START_BTN) == v) started = true;
    }
    return;
  }

  if (ENABLE_MOTOR_SANITY) {
    ledBlink(1);
    ileri();
    delay(400);
    dur();
    delay(200);
    geri();
    delay(400);
    dur();
    while (true) {
      ledBlink(2);
      delay(500);
    }
  }

  if (ENABLE_SELFTEST) selfTestOnce();
  search();
}

void ileri(){ //motorlar ileri fonksiyonu
digitalWrite(L_IN1, HIGH);
digitalWrite(L_IN2, LOW);
digitalWrite(R_IN1, HIGH);
digitalWrite(R_IN2, LOW);
analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);

}

void geri(){ //motorlar geri fonksiyonu
digitalWrite(L_IN1, LOW);
digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN1, LOW);
digitalWrite(R_IN2, HIGH);
analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void sag(){ //sağa dönüş fonksiyonu(sol teker sabit sağ teker hareketli)
digitalWrite(L_IN1, HIGH);
digitalWrite(L_IN2, LOW);
digitalWrite(R_IN1, LOW);
digitalWrite(R_IN2, LOW);
analogWrite(L_PWM, 220);
analogWrite(R_PWM, 0);
    
}

void sol(){ // sola dönüş fonksiyonu(sağ teker sabit sol teker hareketli)
digitalWrite(L_IN1, LOW);
digitalWrite(L_IN2, LOW);
digitalWrite(R_IN1, HIGH);
digitalWrite(R_IN2, LOW);
analogWrite(L_PWM, 0);
analogWrite(R_PWM, 220);
    
}

void sag360(){ //sağa 360 derece dönüş fonksiyonu(sol teker geri sağ teker ileri)
digitalWrite(L_IN1, HIGH);
digitalWrite(L_IN2, LOW);
digitalWrite(R_IN1, LOW);
digitalWrite(R_IN2, HIGH);
analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}   
void sol360(){ //sola 360 derece dönüş fonksiyonu(sağ teker geri sol teker ileri)
digitalWrite(L_IN1, LOW);
digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN1, HIGH);
digitalWrite(R_IN2, LOW);
analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void dur(){ //durma fonksiyonu(tekerler serbest)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);

digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);

analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);
    
}

void anidur(){ //ani durma fonksiyonu(tekerler kitli)
digitalWrite(L_IN1, HIGH);
digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN1, HIGH);
digitalWrite(R_IN2, HIGH);
analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
delay(25);
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);

digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);

analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);
    
}

void tileri(){ //ileriyi tarama fonskiyonu
  gethcsrf();
  g_wall_front = (g_front_cm > 0 && g_front_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

void tsag(){ //sağı tarama fonksiyonu
  gethcsrr();
  g_wall_right = (g_right_cm > 0 && g_right_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

void tsol(){ //solu tarama fonksiyonu
  gethcsrl();
  g_wall_left = (g_left_cm > 0 && g_left_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

// ----- Function-based navigation helpers -----
bool insideMaze(int x, int y) {
  return (x >= 0 && x < (int)MAZE_W && y >= 0 && y < (int)MAZE_H);
}

float wrapYaw(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool qtrIsWhite() {
  if (BYPASS_QTR1A) return false;
  uint16_t v = analogRead(PIN_QTR1A);
  return v > QTR_WHITE_THRESHOLD;
}

void setMotorRaw(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
  }

  if (right >= 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
  }

  analogWrite(L_PWM, abs(left));
  analogWrite(R_PWM, abs(right));
}

void stopMotorsRaw() {
  setMotorRaw(0, 0);
}

void setWallKnownAtCurrent(uint8_t d, bool wall) {
  Cell &c = g_maze[g_y][g_x];
  c.known |= DIR_BIT[d];
  if (wall) c.walls |= DIR_BIT[d];
  else c.walls &= (uint8_t)~DIR_BIT[d];

  int nx = (int)g_x + DX[d];
  int ny = (int)g_y + DY[d];
  if (!insideMaze(nx, ny)) return;
  Cell &n = g_maze[ny][nx];
  uint8_t od = (uint8_t)((d + 2) & 3);
  n.known |= DIR_BIT[od];
  if (wall) n.walls |= DIR_BIT[od];
  else n.walls &= (uint8_t)~DIR_BIT[od];
}

void updateCellFromSensors() {
  tileri();
  tsol();
  tsag();

  Cell &c = g_maze[g_y][g_x];
  c.visited = 1;

  uint8_t dF = g_dir;
  uint8_t dL = (uint8_t)((g_dir + 3) & 3);
  uint8_t dR = (uint8_t)((g_dir + 1) & 3);
  uint8_t dB = (uint8_t)((g_dir + 2) & 3);

  setWallKnownAtCurrent(dF, g_wall_front != 0);
  setWallKnownAtCurrent(dL, g_wall_left != 0);
  setWallKnownAtCurrent(dR, g_wall_right != 0);

  if (g_have_prev && !(g_prev_x == g_x && g_prev_y == g_y)) {
    setWallKnownAtCurrent(dB, false);
  }
}

bool edgeOpenKnown(uint8_t x, uint8_t y, uint8_t d) {
  const Cell &c = g_maze[y][x];
  if ((c.known & DIR_BIT[d]) == 0) return false;
  return (c.walls & DIR_BIT[d]) == 0;
}

bool isFrontierCell(uint8_t x, uint8_t y) {
  const Cell &c = g_maze[y][x];
  if (!c.visited) return false;
  for (uint8_t d = 0; d < 4; d++) {
    if (!edgeOpenKnown(x, y, d)) continue;
    int nx = (int)x + DX[d];
    int ny = (int)y + DY[d];
    if (!insideMaze(nx, ny)) continue;
    if (!g_maze[ny][nx].visited) return true;
  }
  return false;
}

bool bfsToNearestFrontier(uint8_t sx, uint8_t sy, uint8_t &outTx, uint8_t &outTy,
                          uint8_t parentDir[MAZE_H][MAZE_W]) {
  for (uint8_t yy = 0; yy < MAZE_H; yy++) {
    for (uint8_t xx = 0; xx < MAZE_W; xx++) parentDir[yy][xx] = 0xFF;
  }

  uint8_t qx[MAZE_W * MAZE_H];
  uint8_t qy[MAZE_W * MAZE_H];
  uint8_t qh = 0, qt = 0;

  parentDir[sy][sx] = 4;
  qx[qt] = sx;
  qy[qt] = sy;
  qt++;

  while (qh != qt) {
    uint8_t x = qx[qh];
    uint8_t y = qy[qh];
    qh++;

    if (isFrontierCell(x, y)) {
      outTx = x;
      outTy = y;
      return true;
    }

    for (uint8_t d = 0; d < 4; d++) {
      if (!edgeOpenKnown(x, y, d)) continue;
      int nx = (int)x + DX[d];
      int ny = (int)y + DY[d];
      if (!insideMaze(nx, ny)) continue;
      if (!g_maze[ny][nx].visited) continue;
      if (parentDir[ny][nx] != 0xFF) continue;
      parentDir[ny][nx] = d;
      qx[qt] = (uint8_t)nx;
      qy[qt] = (uint8_t)ny;
      qt++;
    }
  }
  return false;
}

bool buildPath(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty,
               uint8_t parentDir[MAZE_H][MAZE_W], uint8_t path[MAZE_W * MAZE_H],
               uint8_t &pathLen) {
  if (parentDir[ty][tx] == 0xFF) return false;
  pathLen = 0;
  uint8_t cx = tx, cy = ty;
  while (!(cx == sx && cy == sy)) {
    uint8_t d = parentDir[cy][cx];
    if (d > 3) return false;
    path[pathLen++] = d;
    cx = (uint8_t)((int)cx - DX[d]);
    cy = (uint8_t)((int)cy - DY[d]);
    if (!insideMaze(cx, cy)) return false;
  }
  for (uint8_t i = 0; i < pathLen / 2; i++) {
    uint8_t t = path[i];
    path[i] = path[pathLen - 1 - i];
    path[pathLen - 1 - i] = t;
  }
  return true;
}

void turnToDir(uint8_t targetDir) {
  int8_t diff = (int8_t)targetDir - (int8_t)g_dir;
  while (diff > 2) diff -= 4;
  while (diff < -2) diff += 4;
  if (diff == 0) return;

  float startYaw = g_yaw_deg;
  float goal = wrapYaw(startYaw + (float)diff * 90.0f);

  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    getmpu();
    float err = wrapYaw(goal - g_yaw_deg);
    if ((err < 0 ? -err : err) < 4.0f) break;
    int s = (err > 0) ? 1 : -1;
    setMotorRaw(-120 * s, 120 * s);
    delay(2);
  }
  stopMotorsRaw();
  delay(10);
  g_dir = targetDir;
}

void readEncDelta(uint8_t &lastLA, uint8_t &lastRA, int &dL, int &dR) {
  uint8_t la = (uint8_t)digitalRead(PIN_ENC_L_A);
  uint8_t ra = (uint8_t)digitalRead(PIN_ENC_R_A);
  dL = 0;
  dR = 0;
  if (la != lastLA) {
    lastLA = la;
    if (la) dL = 1;
  }
  if (ra != lastRA) {
    lastRA = ra;
    if (ra) dR = 1;
  }
}

bool driveOneCell() {
  static constexpr int BASE = 180;
  static constexpr uint16_t TICKS_PER_CELL = 420;
  static constexpr uint32_t TIMEOUT_MS = 1400;

  uint16_t ticks = 0;
  uint8_t lastLA = (uint8_t)digitalRead(PIN_ENC_L_A);
  uint8_t lastRA = (uint8_t)digitalRead(PIN_ENC_R_A);

  uint32_t t0 = millis();
  uint32_t lastSense = 0;

  ledOn();
  setMotorRaw(BASE, BASE);
  while (ticks < TICKS_PER_CELL && (millis() - t0) < TIMEOUT_MS) {
    int dL = 0, dR = 0;
    readEncDelta(lastLA, lastRA, dL, dR);
    ticks += (uint16_t)(dL + dR);

    if (millis() - lastSense > 35) {
      lastSense = millis();
      gethcsrl();
      gethcsrr();
      bool haveL = (g_left_cm > 0 && g_left_cm < 30);
      bool haveR = (g_right_cm > 0 && g_right_cm < 30);
      int corr = 0;
      if (haveL && haveR) corr = (int)g_right_cm - (int)g_left_cm;
      else if (haveL) corr = (int)12 - (int)g_left_cm;
      else if (haveR) corr = (int)g_right_cm - (int)12;
      corr = constrain(corr, -10, 10);
      setMotorRaw(BASE + corr * 3, BASE - corr * 3);
    }
    delay(1);
  }
  stopMotorsRaw();
  ledOff();
  delay(10);
  return ticks >= (TICKS_PER_CELL / 4);
}

bool stepTo(uint8_t dirStep) {
  turnToDir(dirStep);
  tileri();
  if (g_wall_front) {
    ledBlink(4);
    return false;
  }

  g_prev_x = g_x;
  g_prev_y = g_y;
  g_have_prev = true;

  if (!driveOneCell()) return false;

  int nx = (int)g_x + DX[g_dir];
  int ny = (int)g_y + DY[g_dir];
  if (!insideMaze(nx, ny)) return false;
  g_x = (uint8_t)nx;
  g_y = (uint8_t)ny;

  Cell &a = g_maze[g_prev_y][g_prev_x];
  Cell &b = g_maze[g_y][g_x];
  a.known |= DIR_BIT[g_dir];
  a.walls &= (uint8_t)~DIR_BIT[g_dir];
  b.known |= DIR_BIT[(g_dir + 2) & 3];
  b.walls &= (uint8_t)~DIR_BIT[(g_dir + 2) & 3];

  return true;
}

void initSearchRuntime() {
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  for (uint8_t y = 0; y < MAZE_H; y++) {
    for (uint8_t x = 0; x < MAZE_W; x++) {
      g_maze[y][x].walls = 0;
      g_maze[y][x].known = 0;
      g_maze[y][x].visited = 0;
    }
  }

  for (uint8_t x = 0; x < MAZE_W; x++) {
    g_maze[0][x].known |= DIR_BIT[DIR_N];
    g_maze[0][x].walls |= DIR_BIT[DIR_N];
    g_maze[MAZE_H - 1][x].known |= DIR_BIT[DIR_S];
    g_maze[MAZE_H - 1][x].walls |= DIR_BIT[DIR_S];
  }
  for (uint8_t y = 0; y < MAZE_H; y++) {
    g_maze[y][0].known |= DIR_BIT[DIR_W];
    g_maze[y][0].walls |= DIR_BIT[DIR_W];
    g_maze[y][MAZE_W - 1].known |= DIR_BIT[DIR_E];
    g_maze[y][MAZE_W - 1].walls |= DIR_BIT[DIR_E];
  }

  g_have_prev = false;
  g_last_sensor_ms = 0;
  getmpu();
  stopMotorsRaw();
  delay(50);
  g_search_inited = true;
}

void search(){ // labirent çözme algoritması fonksiyonu

//“Frontier-based exploration + BFS (flood-fill)”: keşif hedefi olarak “frontier” (bilinmeyen kenarı olan en yakın hücre) seçiliyor, o hedefe en kısa yol BFS ile bulunup adım adım ilerleniyor.
  if (!g_search_inited) {
    initSearchRuntime();
  }

  if (qtrIsWhite()) {
    stopMotorsRaw();
    delay(5000);
    while (true) {
      stopMotorsRaw();
      delay(1000);
    }
  }

  if (millis() - g_last_sensor_ms > 35) {
    g_last_sensor_ms = millis();
    getmpu();
    updateCellFromSensors();
  }

  uint8_t targetX = g_x, targetY = g_y;
  uint8_t parentDir[MAZE_H][MAZE_W];
  bool haveTarget = bfsToNearestFrontier(g_x, g_y, targetX, targetY, parentDir);

  if (!haveTarget) {
    stopMotorsRaw();
    ledBlink(3);
    delay(100);
    return;
  }

  uint8_t path[MAZE_W * MAZE_H];
  uint8_t pathLen = 0;
  if (!buildPath(g_x, g_y, targetX, targetY, parentDir, path, pathLen)) {
    stopMotorsRaw();
    ledBlink(3);
    delay(50);
    return;
  }

  for (uint8_t i = 0; i < pathLen; i++) {
    if (qtrIsWhite()) break;
    if (!stepTo(path[i])) break;
    getmpu();
    updateCellFromSensors();
  }

  if (isFrontierCell(g_x, g_y)) {
    uint8_t options[4];
    uint8_t nopt = 0;
    for (uint8_t d = 0; d < 4; d++) {
      if (!edgeOpenKnown(g_x, g_y, d)) continue;
      int nx = (int)g_x + DX[d];
      int ny = (int)g_y + DY[d];
      if (!insideMaze(nx, ny)) continue;
      if (!g_maze[ny][nx].visited) options[nopt++] = d;
    }
    if (nopt) {
      uint8_t pick = options[0];
      for (uint8_t k = 0; k < nopt; k++) {
        if (options[k] == g_dir) {
          pick = options[k];
          break;
        }
      }
      stepTo(pick);
      getmpu();
      updateCellFromSensors();
    }
  }

}

void getmpu(){ //mpu6050 sensöründen veri okuma fonksiyonu
  static bool inited = false;
  static uint32_t lastUs = 0;
  static float biasGz = 0.0f;
  static constexpr uint8_t MPU_ADDR = 0x68;

  if (!inited) {
    Wire.begin();
    Wire.setClock(400000);
    delay(10);

    auto writeReg = [&](uint8_t reg, uint8_t val) -> bool {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(reg);
      Wire.write(val);
      return Wire.endTransmission() == 0;
    };

    auto readReg = [&](uint8_t reg, uint8_t *buf, uint8_t n) -> bool {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return false;
      uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)n);
      if (got != n) return false;
      for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
      return true;
    };

    uint8_t who = 0;
    if (!readReg(0x75, &who, 1) || (who != 0x68 && who != 0x69)) {
      g_mpu_ok = false;
#if ENABLE_DIAGNOSTICS
      Serial.println("MPU6050 FAIL");
#endif
      inited = true;
      lastUs = micros();
      return;
    }

    bool ok = true;
    ok &= writeReg(0x6B, 0x00);
    ok &= writeReg(0x1B, 0x00);
    ok &= writeReg(0x1A, 0x03);
    if (!ok) {
      g_mpu_ok = false;
#if ENABLE_DIAGNOSTICS
      Serial.println("MPU6050 FAIL");
#endif
      inited = true;
      lastUs = micros();
      return;
    }

    g_mpu_ok = true;

#if ENABLE_DIAGNOSTICS
    Serial.println("MPU6050 OK");
#endif

    auto readGz = [&]() -> int16_t {
      uint8_t b[6];
      if (!readReg(0x43, b, 6)) return 0;
      int16_t gz = (int16_t)((b[4] << 8) | b[5]);
      return gz;
    };

    delay(50);
    long sum = 0;
    const int N = 500;
    for (int i = 0; i < N; i++) {
      sum += (long)readGz();
      delay(2);
    }
    biasGz = (float)sum / (float)N;
    lastUs = micros();
    g_yaw_deg = 0.0f;
    inited = true;
    return;
  }

  if (!g_mpu_ok) return;

  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;
  lastUs = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x47);
  if (Wire.endTransmission(false) != 0) return;
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, 2);
  if (got != 2) return;
  uint8_t hi = (uint8_t)Wire.read();
  uint8_t lo = (uint8_t)Wire.read();
  int16_t gz = (int16_t)((hi << 8) | lo);
  float rateDps = ((float)gz - biasGz) / 131.0f;
  g_yaw_deg = wrapYaw(g_yaw_deg + rateDps * dt);
  return;
}

uint16_t pingUltrasonicCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  uint32_t us = pulseIn(echo, HIGH, 25000);
  if (!us) return 0;
  uint16_t cm = (uint16_t)(us / 58);
  if (cm > 400) cm = 400;
  return cm;
}

void gethcsrf(){ //hcsr04 ön sensöründen veri okuma fonksiyonu
  static uint32_t lastMs = 0;
  static uint16_t lastCm = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 60) {
    g_front_cm = lastCm;
    return;
  }
  lastMs = nowMs;

  uint16_t a = pingUltrasonicCM(HFT, H_F_E);
  delay(2);
  uint16_t b = pingUltrasonicCM(HFT, H_F_E);
  uint16_t m = (a == 0) ? b : ((b == 0) ? a : (uint16_t)((a + b) / 2));
  lastCm = m;
  g_front_cm = m;
}

void gethcsrl(){ //hcsr04 sol sensöründen veri okuma fonksiyonu
  static uint32_t lastMs = 0;
  static uint16_t lastCm = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 60) {
    g_left_cm = lastCm;
    return;
  }
  lastMs = nowMs;

  uint16_t a = pingUltrasonicCM(HLT, H_L_E);
  delay(2);
  uint16_t b = pingUltrasonicCM(HLT, H_L_E);
  uint16_t m = (a == 0) ? b : ((b == 0) ? a : (uint16_t)((a + b) / 2));
  lastCm = m;
  g_left_cm = m;
}

void gethcsrr(){ //hcsr04 sağ sensöründen veri okuma fonksiyonu
  static uint32_t lastMs = 0;
  static uint16_t lastCm = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 60) {
    g_right_cm = lastCm;
    return;
  }
  lastMs = nowMs;

  uint16_t a = pingUltrasonicCM(HRT, H_R_E);
  delay(2);
  uint16_t b = pingUltrasonicCM(HRT, H_R_E);
  uint16_t m = (a == 0) ? b : ((b == 0) ? a : (uint16_t)((a + b) / 2));
  lastCm = m;
  g_right_cm = m;
}
