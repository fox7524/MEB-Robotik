#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 115200;

static const int PIN_START_BTN = PB5;

static const int PIN_ENC_L_A = PB10;
static const int PIN_ENC_L_B = PB8;
static const int PIN_ENC_R_A = PB11;
static const int PIN_ENC_R_B = PB9;

static volatile long g_l = 0;
static volatile long g_r = 0;
static volatile long g_lDir = 0;
static volatile long g_rDir = 0;

static void isrEncL() {
  if (PIN_ENC_L_B >= 0) {
    int b = digitalRead(PIN_ENC_L_B);
    if (b == HIGH) { g_l++; g_lDir++; } else { g_l++; g_lDir--; }
  } else {
    g_l++;
  }
}

static void isrEncR() {
  if (PIN_ENC_R_B >= 0) {
    int b = digitalRead(PIN_ENC_R_B);
    if (b == HIGH) { g_r++; g_rDir++; } else { g_r++; g_rDir--; }
  } else {
    g_r++;
  }
}

static void waitStart() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  while (digitalRead(PIN_START_BTN) == HIGH) {
    delay(5);
  }
  delay(250);
}

static long readAtomic(volatile long& v) {
  noInterrupts();
  long x = v;
  interrupts();
  return x;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.println("ENCODER_TEST");
  Serial.println("Set PIN_ENC_* at top, then upload");
  Serial.println("Press START to begin");

  if (PIN_ENC_L_A >= 0) pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  if (PIN_ENC_L_B >= 0) pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  if (PIN_ENC_R_A >= 0) pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  if (PIN_ENC_R_B >= 0) pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  if (PIN_ENC_L_A >= 0) attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
  if (PIN_ENC_R_A >= 0) attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);

  waitStart();
  Serial.println("RUN");
}

void loop() {
  static unsigned long lastMs = 0;
  static long lastL = 0;
  static long lastR = 0;
  unsigned long now = millis();
  if ((now - lastMs) < 250) return;
  lastMs = now;

  long l = readAtomic(g_l);
  long r = readAtomic(g_r);
  long dl = l - lastL;
  long dr = r - lastR;
  lastL = l;
  lastR = r;

  Serial.print("L=");
  Serial.print(l);
  Serial.print(" dL/250ms=");
  Serial.print(dl);
  if (PIN_ENC_L_B >= 0) {
    Serial.print(" Ldir=");
    Serial.print(readAtomic(g_lDir));
  }
  Serial.print("  R=");
  Serial.print(r);
  Serial.print(" dR/250ms=");
  Serial.print(dr);
  if (PIN_ENC_R_B >= 0) {
    Serial.print(" Rdir=");
    Serial.print(readAtomic(g_rDir));
  }
  Serial.println();
}
