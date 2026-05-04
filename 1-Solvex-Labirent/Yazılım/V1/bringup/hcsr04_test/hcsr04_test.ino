#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 115200;

static const int PIN_START_BTN = PB5;
static const bool REQUIRE_START_BUTTON = false;

static const int PIN_TRIG_FRONT = PA1;
static const int PIN_TRIG_LEFT  = PA2;
static const int PIN_TRIG_RIGHT = PA3;

static const int PIN_ECHO_FRONT = PA4;
static const int PIN_ECHO_LEFT  = PA5;
static const int PIN_ECHO_RIGHT = PA6;

static const unsigned long ECHO_TIMEOUT_US = 25000;
static const unsigned long INTER_PING_MS = 35;

static unsigned long pingEchoUs(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
}

static int pingCmOnce(int trigPin, int echoPin) {
  unsigned long us = pingEchoUs(trigPin, echoPin);
  if (us == 0) return -1;
  return (int)(us / 58);
}

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

static int pingCmMed3(int trigPin, int echoPin) {
  int a = pingCmOnce(trigPin, echoPin);
  delay(INTER_PING_MS);
  int b = pingCmOnce(trigPin, echoPin);
  delay(INTER_PING_MS);
  int c = pingCmOnce(trigPin, echoPin);
  if (a < 0) a = 500;
  if (b < 0) b = 500;
  if (c < 0) c = 500;
  return median3(a, b, c);
}

static bool isStartActive() { return digitalRead(PIN_START_BTN) == LOW; }

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_START_BTN, INPUT_PULLUP);

  pinMode(PIN_TRIG_FRONT, OUTPUT);
  digitalWrite(PIN_TRIG_FRONT, LOW);
  if (PIN_TRIG_LEFT != PIN_TRIG_FRONT) {
    pinMode(PIN_TRIG_LEFT, OUTPUT);
    digitalWrite(PIN_TRIG_LEFT, LOW);
  }
  if (PIN_TRIG_RIGHT != PIN_TRIG_FRONT && PIN_TRIG_RIGHT != PIN_TRIG_LEFT) {
    pinMode(PIN_TRIG_RIGHT, OUTPUT);
    digitalWrite(PIN_TRIG_RIGHT, LOW);
  }

  pinMode(PIN_ECHO_FRONT, INPUT);
  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);

  Serial.println("HCSR04_TEST");
  Serial.println("RUN");
}

void loop() {
  if (REQUIRE_START_BUTTON && !isStartActive()) {
    delay(10);
    return;
  }

  int f = pingCmMed3(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
  delay(INTER_PING_MS);
  int l = pingCmMed3(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
  delay(INTER_PING_MS);
  int r = pingCmMed3(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);

  Serial.print("F=");
  Serial.print(f);
  Serial.print("cm  L=");
  Serial.print(l);
  Serial.print("cm  R=");
  Serial.print(r);
  Serial.println("cm");

  delay(120);
}
