// --- Solvex-3.ino pinout (PA/PB ile bağlanan sistem) ---

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include "I2Cdev.h"



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

void setup() {
Serial.begin(115600);
Serial.print("merheaba");    
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

  search();
}

void ileri(){ //motorlar ileri fonksiyonu
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);

}

void geri(){ //motorlar geri fonksiyonu
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void sag(){ //sağa dönüş fonksiyonu(sol teker sabit sağ teker hareketli)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 0);
    
}

void sol(){ // sola dönüş fonksiyonu(sağ teker sabit sol teker hareketli)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 0);
analogWrite(R_PWM, 255);
    
}

void sag360(){ //sağa 360 derece dönüş fonksiyonu(sol teker geri sağ teker ileri)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);   

digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}   
void sol360(){ //sola 360 derece dönüş fonksiyonu(sağ teker geri sol teker ileri)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);

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
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


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
  g_wall_front = (g_front_cm > 0 && g_front_cm < 14) ? 1 : 0;
}

void tsag(){ //sağı tarama fonksiyonu
  gethcsrr();
  g_wall_right = (g_right_cm > 0 && g_right_cm < 14) ? 1 : 0;
}

void tsol(){ //solu tarama fonksiyonu
  gethcsrl();
  g_wall_left = (g_left_cm > 0 && g_left_cm < 14) ? 1 : 0;
}

void search(){ // labirent çözme algoritması fonksiyonu

//“Frontier-based exploration + BFS (flood-fill)”: keşif hedefi olarak “frontier” (bilinmeyen kenarı olan en yakın hücre) seçiliyor, o hedefe en kısa yol BFS ile bulunup adım adım ilerleniyor.

  static bool inited = false;
  static uint32_t lastSensorMs = 0;
  static uint8_t prevX = 0;
  static uint8_t prevY = 0;
  static bool havePrev = false;

  auto inside = [](int x, int y) -> bool {
    return (x >= 0 && x < (int)MAZE_W && y >= 0 && y < (int)MAZE_H);
  };

  auto wrapYaw = [](float a) -> float {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  };

  static constexpr bool BYPASS_QTR1A = true;
  auto qtrIsWhite = []() -> bool {
    if (BYPASS_QTR1A) return false;
    uint16_t v = analogRead(PIN_QTR1A);
    return v > 2500;
  };

  auto setMotor = [&](int left, int right) {
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
  };

  auto stopMotors = [&]() { setMotor(0, 0); };

  auto updateCellFromSensors = [&]() {
    tileri();
    tsol();
    tsag();

    Cell &c = g_maze[g_y][g_x];
    c.visited = 1;

    uint8_t dF = g_dir;
    uint8_t dL = (uint8_t)((g_dir + 3) & 3);
    uint8_t dR = (uint8_t)((g_dir + 1) & 3);
    uint8_t dB = (uint8_t)((g_dir + 2) & 3);

    auto setWallKnown = [&](uint8_t d, bool wall) {
      c.known |= DIR_BIT[d];
      if (wall) c.walls |= DIR_BIT[d];
      else c.walls &= (uint8_t)~DIR_BIT[d];

      int nx = (int)g_x + DX[d];
      int ny = (int)g_y + DY[d];
      if (!inside(nx, ny)) return;
      Cell &n = g_maze[ny][nx];
      uint8_t od = (uint8_t)((d + 2) & 3);
      n.known |= DIR_BIT[od];
      if (wall) n.walls |= DIR_BIT[od];
      else n.walls &= (uint8_t)~DIR_BIT[od];
    };

    setWallKnown(dF, g_wall_front);
    setWallKnown(dL, g_wall_left);
    setWallKnown(dR, g_wall_right);

    if (havePrev) {
      if (prevX == g_x && prevY == g_y) {
      } else {
        setWallKnown(dB, false);
      }
    }
  };

  auto edgeOpenKnown = [&](uint8_t x, uint8_t y, uint8_t d) -> bool {
    const Cell &c = g_maze[y][x];
    if ((c.known & DIR_BIT[d]) == 0) return false;
    return (c.walls & DIR_BIT[d]) == 0;
  };

  auto isFrontierCell = [&](uint8_t x, uint8_t y) -> bool {
    const Cell &c = g_maze[y][x];
    if (!c.visited) return false;
    for (uint8_t d = 0; d < 4; d++) {
      if (!edgeOpenKnown(x, y, d)) continue;
      int nx = (int)x + DX[d];
      int ny = (int)y + DY[d];
      if (!inside(nx, ny)) continue;
      if (!g_maze[ny][nx].visited) return true;
    }
    return false;
  };

  auto bfsToNearestFrontier = [&](uint8_t sx, uint8_t sy, uint8_t &outTx, uint8_t &outTy,
                                 uint8_t parentDir[MAZE_H][MAZE_W]) -> bool {
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
        if (!inside(nx, ny)) continue;
        if (!g_maze[ny][nx].visited) continue;
        if (parentDir[ny][nx] != 0xFF) continue;
        parentDir[ny][nx] = d;
        qx[qt] = (uint8_t)nx;
        qy[qt] = (uint8_t)ny;
        qt++;
      }
    }
    return false;
  };

  auto buildPath = [&](uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty,
                       uint8_t parentDir[MAZE_H][MAZE_W], uint8_t path[MAZE_W * MAZE_H],
                       uint8_t &pathLen) -> bool {
    if (parentDir[ty][tx] == 0xFF) return false;
    pathLen = 0;
    uint8_t cx = tx, cy = ty;
    while (!(cx == sx && cy == sy)) {
      uint8_t d = parentDir[cy][cx];
      if (d > 3) return false;
      path[pathLen++] = d;
      cx = (uint8_t)((int)cx - DX[d]);
      cy = (uint8_t)((int)cy - DY[d]);
      if (!inside(cx, cy)) return false;
    }
    for (uint8_t i = 0; i < pathLen / 2; i++) {
      uint8_t t = path[i];
      path[i] = path[pathLen - 1 - i];
      path[pathLen - 1 - i] = t;
    }
    return true;
  };

  auto turnToDir = [&](uint8_t targetDir) {
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
      if (fabsf(err) < 4.0f) break;
      int s = (err > 0) ? 1 : -1;
      setMotor(-120 * s, 120 * s);
      delay(2);
    }
    stopMotors();
    delay(10);
    g_dir = targetDir;
  };

  auto readEncDelta = [&](uint8_t &lastLA, uint8_t &lastRA, int &dL, int &dR) {
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
  };

  auto driveOneCell = [&]() -> bool {
    static constexpr int BASE = 180;
    static constexpr uint16_t TICKS_PER_CELL = 420;
    static constexpr uint32_t TIMEOUT_MS = 1400;

    uint16_t ticks = 0;
    uint8_t lastLA = (uint8_t)digitalRead(PIN_ENC_L_A);
    uint8_t lastRA = (uint8_t)digitalRead(PIN_ENC_R_A);

    uint32_t t0 = millis();
    uint32_t lastSense = 0;

    setMotor(BASE, BASE);
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
        setMotor(BASE + corr * 3, BASE - corr * 3);
      }
      delay(1);
    }
    stopMotors();
    delay(10);
    return ticks >= (TICKS_PER_CELL / 4);
  };

  auto stepTo = [&](uint8_t dirStep) -> bool {
    turnToDir(dirStep);
    tileri();
    if (g_wall_front) return false;

    prevX = g_x;
    prevY = g_y;
    havePrev = true;

    if (!driveOneCell()) return false;

    int nx = (int)g_x + DX[g_dir];
    int ny = (int)g_y + DY[g_dir];
    if (!inside(nx, ny)) return false;
    g_x = (uint8_t)nx;
    g_y = (uint8_t)ny;

    Cell &a = g_maze[prevY][prevX];
    Cell &b = g_maze[g_y][g_x];
    a.known |= DIR_BIT[g_dir];
    a.walls &= (uint8_t)~DIR_BIT[g_dir];
    b.known |= DIR_BIT[(g_dir + 2) & 3];
    b.walls &= (uint8_t)~DIR_BIT[(g_dir + 2) & 3];

    return true;
  };

  if (!inited) {
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
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

    getmpu();
    inited = true;
    lastSensorMs = 0;
    stopMotors();
    delay(50);
  }

  if (qtrIsWhite()) {
    stopMotors();
    delay(5000);
    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  if (millis() - lastSensorMs > 35) {
    lastSensorMs = millis();
    getmpu();
    updateCellFromSensors();
  }

  uint8_t targetX = g_x, targetY = g_y;
  uint8_t parentDir[MAZE_H][MAZE_W];
  bool haveTarget = bfsToNearestFrontier(g_x, g_y, targetX, targetY, parentDir);

  if (!haveTarget) {
    stopMotors();
    delay(100);
    return;
  }

  uint8_t path[MAZE_W * MAZE_H];
  uint8_t pathLen = 0;
  if (!buildPath(g_x, g_y, targetX, targetY, parentDir, path, pathLen)) {
    stopMotors();
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
      if (!inside(nx, ny)) continue;
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
  static MPU6050 mpu;
  static uint32_t lastUs = 0;
  static float biasGz = 0.0f;

  if (!inited) {
    Wire.begin();
    mpu.initialize();
    delay(50);
    long sum = 0;
    const int N = 500;
    for (int i = 0; i < N; i++) {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
      sum += (long)gz;
      delay(2);
    }
    biasGz = (float)sum / (float)N;
    lastUs = micros();
    g_yaw_deg = 0.0f;
    inited = true;
    return;
  }

  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;
  lastUs = now;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float rateDps = ((float)gz - biasGz) / 131.0f;
  g_yaw_deg += rateDps * dt;
  while (g_yaw_deg > 180.0f) g_yaw_deg -= 360.0f;
  while (g_yaw_deg < -180.0f) g_yaw_deg += 360.0f;
}

void gethcsrf(){ //hcsr04 ön sensöründen veri okuma fonksiyonu
  auto ping = [&](int trig, int echo) -> uint16_t {
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
  };

  uint16_t a = ping(HFT, H_F_E);
  uint16_t b = ping(HFT, H_F_E);
  uint16_t c = ping(HFT, H_F_E);
  uint16_t m = a;
  if ((a <= b && b <= c) || (c <= b && b <= a)) m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else m = c;
  g_front_cm = m;
}

void gethcsrl(){ //hcsr04 sol sensöründen veri okuma fonksiyonu
  auto ping = [&](int trig, int echo) -> uint16_t {
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
  };

  uint16_t a = ping(HLT, H_L_E);
  uint16_t b = ping(HLT, H_L_E);
  uint16_t c = ping(HLT, H_L_E);
  uint16_t m = a;
  if ((a <= b && b <= c) || (c <= b && b <= a)) m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else m = c;
  g_left_cm = m;
}

void gethcsrr(){ //hcsr04 sağ sensöründen veri okuma fonksiyonu
  auto ping = [&](int trig, int echo) -> uint16_t {
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
  };

  uint16_t a = ping(HRT, H_R_E);
  uint16_t b = ping(HRT, H_R_E);
  uint16_t c = ping(HRT, H_R_E);
  uint16_t m = a;
  if ((a <= b && b <= c) || (c <= b && b <= a)) m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else m = c;
  g_right_cm = m;
}
