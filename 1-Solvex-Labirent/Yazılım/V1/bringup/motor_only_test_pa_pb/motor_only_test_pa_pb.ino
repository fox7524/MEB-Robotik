#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 115200;

static const int PIN_START_BTN = PB5;
static const bool REQUIRE_START_BUTTON = false;

static const int R_IN1 = PB12;
static const int R_IN2 = PB13;
static const int L_IN1 = PB14;
static const int L_IN2 = PB15;

static void motorStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

static void ileri() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
}

static void geri() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
}

static void sag() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

static void sol() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
}

static bool isStartActive() {
  int v = digitalRead(PIN_START_BTN);
  return (v == LOW);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  motorStop();

  pinMode(PIN_START_BTN, INPUT_PULLUP);
  if (LED_BUILTIN >= 0) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  }

  Serial.println("MOTOR_ONLY_TEST_PA_PB");
  Serial.println("START=PB5 (INPUT_PULLUP, active LOW)");
  Serial.println("If motors never move, check ENA/ENB jumpers + motor power + GND common.");
  Serial.print("REQUIRE_START_BUTTON=");
  Serial.println(REQUIRE_START_BUTTON ? 1 : 0);
  Serial.println("RUN");
}

void loop() {
  if (REQUIRE_START_BUTTON && !isStartActive()) {
    motorStop();
    if (LED_BUILTIN >= 0) digitalWrite(LED_BUILTIN, (millis() / 250) & 1);
    delay(10);
    return;
  }

  if (LED_BUILTIN >= 0) digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("FWD");
  ileri();
  delay(1200);
  motorStop();

  Serial.println("REV");
  geri();
  delay(1200);
  motorStop();
  delay(400);

  Serial.println("LEFT");
  sol();
  delay(450);
  motorStop();
  delay(400);

  Serial.println("RIGHT");
  sag();
  delay(450);
  motorStop();
  delay(800);

  if (LED_BUILTIN >= 0) digitalWrite(LED_BUILTIN, LOW);
}
