#include <Arduino.h>
#include <Wire.h>

const int PIN_START_BTN = PB5;
const int PIN_QTR1A = PA0;

const int HFT = PA1;
const int HLT = PA2;
const int HRT = PA3;

const int H_F_E = PA4;
const int H_L_E = PA5;
const int H_R_E = PA6;

const int R_PWM = PB0;
const int L_PWM = PB1;
const int R_IN1 = PB12;
const int R_IN2 = PB13;
const int L_IN1 = PB14;
const int L_IN2 = PB15;

const int PIN_ENC_L_A = PB10;
const int PIN_ENC_R_A = PB11;
const int PIN_ENC_L_B = PB8;
const int PIN_ENC_R_B = PB9;

const int PIN_LED = PC13;

static constexpr bool USE_ENCODERS = true;
static constexpr bool ENC_INVERT_L = false;
static constexpr bool ENC_INVERT_R = false;

static constexpr bool INVERT_L = false;
static constexpr bool INVERT_R = false;

static constexpr uint8_t PWM_RUN = 170;
static constexpr uint8_t PWM_TURN = 160;

static constexpr uint16_t WALL_CM_FRONT = 13;
static constexpr uint16_t WALL_CM_SIDE = 8;

static constexpr uint16_t PING_TIMEOUT_US = 20000;
static constexpr uint16_t PING_INTER_DELAY_MS = 35;

static constexpr uint16_t STEP_MS = 260;
static constexpr uint16_t TURN_90_MS = 290;
static constexpr uint16_t TURN_180_MS = 580;

static constexpr bool ENABLE_QTR_FINISH = true;
static constexpr uint16_t QTR_MARGIN = 250;
static constexpr uint8_t QTR_WHITE_STREAK = 3;

static constexpr bool ENABLE_MOTOR_TEST = false;
static constexpr uint16_t START_FORWARD_MS = 1200;
static constexpr long START_40CM_TICKS = 0;

enum RunState : uint8_t { ST_WAIT = 0, ST_START = 1, ST_MOTOR_TEST = 2, ST_RUN = 3, ST_STOP = 4 };
static RunState g_state = ST_WAIT;

static uint16_t g_cm_f = 0;
static uint16_t g_cm_l = 0;
static uint16_t g_cm_r = 0;

static int g_startIdleLevel = HIGH;
static int g_qtrBaseline = 0;
static uint8_t g_qtrStreak = 0;

static volatile long g_encL = 0;
static volatile long g_encR = 0;
static bool g_encOk = false;

static long g_stepTicksTarget = 0;
static long g_turn90TicksTarget = 0;
static long g_start40Ticks = 0;

static inline void ledOn() { digitalWrite(PIN_LED, LOW); }
static inline void ledOff() { digitalWrite(PIN_LED, HIGH); }

static void ledBlink(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    ledOn();
    delay(120);
    ledOff();
    delay(120);
  }
  delay(250);
}

static void ledPulse(uint8_t n, uint16_t onMs, uint16_t offMs, uint16_t gapMs) {
  for (uint8_t i = 0; i < n; i++) {
    ledOn();
    delay(onMs);
    ledOff();
    delay(offMs);
  }
  delay(gapMs);
}

static inline void ledCode(uint8_t code) {
  ledPulse(code, 70, 80, 220);
}

static void motorSet(int16_t left, int16_t right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (INVERT_L) left = -left;
  if (INVERT_R) right = -right;

  if (left == 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, 0);
  } else if (left > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, (uint8_t)left);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, (uint8_t)(-left));
  }

  if (right == 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_PWM, 0);
  } else if (right > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_PWM, (uint8_t)right);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_PWM, (uint8_t)(-right));
  }
}

static inline void dur() { motorSet(0, 0); }
static inline void ileri() { motorSet(PWM_RUN, PWM_RUN); }
static inline void geri() { motorSet(-PWM_RUN, -PWM_RUN); }
static inline void sag360() { motorSet(PWM_TURN, -PWM_TURN); }
static inline void sol360() { motorSet(-PWM_TURN, PWM_TURN); }

static void isrEncL() {
  int dir = (digitalRead(PIN_ENC_L_B) == HIGH) ? 1 : -1;
  if (ENC_INVERT_L) dir = -dir;
  g_encL += dir;
}

static void isrEncR() {
  int dir = (digitalRead(PIN_ENC_R_B) == HIGH) ? 1 : -1;
  if (ENC_INVERT_R) dir = -dir;
  g_encR += dir;
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

static long encAvgAbs() {
  long l = labs(encL());
  long r = labs(encR());
  return (l + r) / 2;
}

static uint16_t pingCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  uint32_t us = pulseIn(echo, HIGH, PING_TIMEOUT_US);
  if (!us) return 0;
  uint16_t cm = (uint16_t)(us / 58u);
  if (cm > 400) cm = 400;
  return cm;
}

static void readUltrasonics() {
  g_cm_f = pingCM(HFT, H_F_E);
  delay(PING_INTER_DELAY_MS);
  g_cm_l = pingCM(HLT, H_L_E);
  delay(PING_INTER_DELAY_MS);
  g_cm_r = pingCM(HRT, H_R_E);
}

static inline bool wallFront() { return (g_cm_f > 0 && g_cm_f < WALL_CM_FRONT); }
static inline bool wallLeft() { return (g_cm_l > 0 && g_cm_l < WALL_CM_SIDE); }
static inline bool wallRight() { return (g_cm_r > 0 && g_cm_r < WALL_CM_SIDE); }

static void stepForward() {
  if (!USE_ENCODERS || !g_encOk) {
    ileri();
    delay(STEP_MS);
    dur();
    delay(60);
    return;
  }

  if (g_stepTicksTarget <= 0) {
    resetEncoders();
    ileri();
    delay(STEP_MS);
    dur();
    delay(60);
    long m = encAvgAbs();
    if (m > 0) g_stepTicksTarget = m;
    return;
  }

  resetEncoders();
  ileri();
  unsigned long t0 = millis();
  while (encAvgAbs() < g_stepTicksTarget) {
    if ((millis() - t0) > 2500) break;
    delay(2);
  }
  dur();
  delay(60);
  long m = encAvgAbs();
  if (m > 0) g_stepTicksTarget = (g_stepTicksTarget * 3 + m) / 4;
}

static void turnLeft() {
  if (!USE_ENCODERS || !g_encOk) {
    sol360();
    delay(TURN_90_MS);
    dur();
    delay(80);
    return;
  }

  if (g_turn90TicksTarget <= 0) {
    resetEncoders();
    sol360();
    delay(TURN_90_MS);
    dur();
    delay(80);
    long m = encAvgAbs();
    if (m > 0) g_turn90TicksTarget = m;
    return;
  }

  resetEncoders();
  sol360();
  unsigned long t0 = millis();
  while (encAvgAbs() < g_turn90TicksTarget) {
    if ((millis() - t0) > 1800) break;
    delay(2);
  }
  dur();
  delay(80);
  long m = encAvgAbs();
  if (m > 0) g_turn90TicksTarget = (g_turn90TicksTarget * 3 + m) / 4;
}

static void turnRight() {
  if (!USE_ENCODERS || !g_encOk) {
    sag360();
    delay(TURN_90_MS);
    dur();
    delay(80);
    return;
  }

  if (g_turn90TicksTarget <= 0) {
    resetEncoders();
    sag360();
    delay(TURN_90_MS);
    dur();
    delay(80);
    long m = encAvgAbs();
    if (m > 0) g_turn90TicksTarget = m;
    return;
  }

  resetEncoders();
  sag360();
  unsigned long t0 = millis();
  while (encAvgAbs() < g_turn90TicksTarget) {
    if ((millis() - t0) > 1800) break;
    delay(2);
  }
  dur();
  delay(80);
  long m = encAvgAbs();
  if (m > 0) g_turn90TicksTarget = (g_turn90TicksTarget * 3 + m) / 4;
}

static void turnAround() {
  if (!USE_ENCODERS || !g_encOk) {
    sag360();
    delay(TURN_180_MS);
    dur();
    delay(100);
    return;
  }

  long target = (g_turn90TicksTarget > 0) ? (2 * g_turn90TicksTarget) : 0;
  if (target <= 0) {
    resetEncoders();
    sag360();
    delay(TURN_180_MS);
    dur();
    delay(100);
    return;
  }

  resetEncoders();
  sag360();
  unsigned long t0 = millis();
  while (encAvgAbs() < target) {
    if ((millis() - t0) > 2600) break;
    delay(2);
  }
  dur();
  delay(100);
}

static void startMove40cm() {
  ledCode(2);
  if (!USE_ENCODERS || !g_encOk) {
    ileri();
    delay(START_FORWARD_MS);
    dur();
    delay(120);
    ledCode(3);
    return;
  }

  long target = (START_40CM_TICKS > 0) ? START_40CM_TICKS : g_start40Ticks;
  if (target <= 0) {
    resetEncoders();
    ileri();
    delay(START_FORWARD_MS);
    dur();
    delay(120);
    long m = encAvgAbs();
    if (m > 0) g_start40Ticks = m;
    ledCode(3);
    return;
  }

  resetEncoders();
  ileri();
  unsigned long t0 = millis();
  while (encAvgAbs() < target) {
    if ((millis() - t0) > 4000) break;
    delay(2);
  }
  dur();
  delay(120);
  long m = encAvgAbs();
  if (m > 0) g_start40Ticks = m;
  ledCode(3);
}

static bool qtrIsWhite() {
  if (!ENABLE_QTR_FINISH) return false;
  int v = analogRead(PIN_QTR1A);
  bool white = v > (g_qtrBaseline + (int)QTR_MARGIN);
  if (white) {
    if (g_qtrStreak < 255) g_qtrStreak++;
  } else {
    g_qtrStreak = 0;
  }
  return g_qtrStreak >= QTR_WHITE_STREAK;
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

static void measureStartIdleLevel() {
  int ones = 0;
  const int n = 40;
  for (int i = 0; i < n; i++) {
    ones += (digitalRead(PIN_START_BTN) == HIGH) ? 1 : 0;
    delay(2);
  }
  g_startIdleLevel = (ones > (n / 2)) ? HIGH : LOW;
}

static bool startTriggered() {
  static uint32_t changedAt = 0;
  int v = digitalRead(PIN_START_BTN);
  if (v == g_startIdleLevel) {
    changedAt = 0;
    return false;
  }
  if (changedAt == 0) changedAt = millis();
  return (millis() - changedAt) >= 30;
}

static void motorTestSequence() {
  ledBlink(1);
  ileri();
  delay(600);
  dur();
  delay(250);

  geri();
  delay(600);
  dur();
  delay(250);

  sag360();
  delay(450);
  dur();
  delay(250);

  sol360();
  delay(450);
  dur();
  delay(250);
}

static void runLeftHandRuleStep() {
  readUltrasonics();

  bool wF = wallFront();
  bool wL = wallLeft();
  bool wR = wallRight();

  if (!wL) {
    turnLeft();
    stepForward();
    return;
  }
  if (!wF) {
    stepForward();
    return;
  }
  if (!wR) {
    turnRight();
    stepForward();
    return;
  }
  turnAround();
  stepForward();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  ledOff();

  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_QTR1A, INPUT);

  pinMode(HFT, OUTPUT);
  pinMode(HLT, OUTPUT);
  pinMode(HRT, OUTPUT);
  digitalWrite(HFT, LOW);
  digitalWrite(HLT, LOW);
  digitalWrite(HRT, LOW);

  pinMode(H_F_E, INPUT);
  pinMode(H_L_E, INPUT);
  pinMode(H_R_E, INPUT);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  dur();

  Wire.begin();

  if (USE_ENCODERS) {
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);
    int il = digitalPinToInterrupt(PIN_ENC_L_A);
    int ir = digitalPinToInterrupt(PIN_ENC_R_A);
    if (il >= 0 && ir >= 0) {
      attachInterrupt(il, isrEncL, RISING);
      attachInterrupt(ir, isrEncR, RISING);
      g_encOk = true;
      resetEncoders();
      ledCode(1);
    } else {
      ledCode(6);
    }
  }

  measureStartIdleLevel();
  calibrateQtrBaseline();

  ledBlink(2);
  g_state = ST_WAIT;
}

void loop() {
  if (g_state == ST_WAIT) {
    dur();
    if (startTriggered()) {
      ledCode(4);
      g_state = ST_START;
    }
    delay(5);
    return;
  }

  if (g_state == ST_START) {
    startMove40cm();
    if (ENABLE_MOTOR_TEST) g_state = ST_MOTOR_TEST;
    else g_state = ST_RUN;
    ledCode(5);
    return;
  }

  if (g_state == ST_MOTOR_TEST) {
    motorTestSequence();
    g_state = ST_RUN;
    return;
  }

  if (g_state == ST_RUN) {
    if (qtrIsWhite()) {
      g_state = ST_STOP;
      return;
    }
    runLeftHandRuleStep();
    return;
  }

  if (g_state == ST_STOP) {
    ledCode(7);
    dur();
    unsigned long t0 = millis();
    while ((millis() - t0) < 5000) {
      dur();
      delay(10);
    }
    while (true) {
      dur();
      delay(50);
    }
  }
}
