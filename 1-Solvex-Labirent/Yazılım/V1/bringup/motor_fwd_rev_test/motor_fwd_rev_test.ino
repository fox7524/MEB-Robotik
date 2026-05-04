#include <Arduino.h>

static const int PIN_START_BTN = PB5;

static const int R_IN1 = PB12;
static const int R_IN2 = PB13;
static const int L_IN1 = PB14;
static const int L_IN2 = PB15;

static const int START_ACTIVE_LEVEL = HIGH;

static void motorStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

static void motorForward() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
}

static void motorReverse() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
}

void setup() {
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  motorStop();

  pinMode(PIN_START_BTN, INPUT_PULLDOWN);
}

void loop() {
  if (digitalRead(PIN_START_BTN) != START_ACTIVE_LEVEL) {
    motorStop();
    delay(10);
    return;
  }

  motorForward();
  delay(1200);
  motorStop();
  delay(400);

  motorReverse();
  delay(1200);
  motorStop();
  delay(700);
}
