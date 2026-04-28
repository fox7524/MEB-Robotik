#include <Arduino.h>

static const uint32_t BAUD = 115200;

static const uint8_t TRIG_PIN = PA1;
static const uint8_t ECHO_PIN = PA0;

static const unsigned long ECHO_TIMEOUT_US = 25000;
static const unsigned long PING_INTERVAL_MS = 60;

static unsigned long readEchoPulseUs() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
}

static int readDistanceCm() {
  unsigned long us = readEchoPulseUs();
  if (us == 0) return -1;
  return (int)(us / 58);
}

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

static int readDistanceCmFiltered() {
  int a = readDistanceCm(); delay(5);
  int b = readDistanceCm(); delay(5);
  int c = readDistanceCm();

  if (a < 0) return (b >= 0 && c >= 0) ? ((b + c) / 2) : ((b >= 0) ? b : c);
  if (b < 0) return (a >= 0 && c >= 0) ? ((a + c) / 2) : ((a >= 0) ? a : c);
  if (c < 0) return (a >= 0 && b >= 0) ? ((a + b) / 2) : ((a >= 0) ? a : b);
  return median3(a, b, c);
}

void setup() {
  Serial.begin(BAUD);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  int cm = readDistanceCmFiltered();
  if (cm < 0) {
    Serial.println("cm=NA");
  } else {
    Serial.print("cm=");
    Serial.println(cm);
  }
  delay(PING_INTERVAL_MS);
}

