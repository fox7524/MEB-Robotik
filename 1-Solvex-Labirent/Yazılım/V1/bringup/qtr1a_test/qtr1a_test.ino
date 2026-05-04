#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 115200;
static const int PIN_START_BTN = PB5;

static const int PIN_QTR1A = PA0;
static const int QTR_MARGIN = 250;

static int g_baseline = 0;

static void waitStart() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  while (digitalRead(PIN_START_BTN) == HIGH) {
    delay(5);
  }
  delay(250);
}

static void calibrateBaseline() {
  long sum = 0;
  const int n = 300;
  for (int i = 0; i < n; i++) {
    sum += analogRead(PIN_QTR1A);
    delay(5);
  }
  g_baseline = (int)(sum / n);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_QTR1A, INPUT);

  Serial.println("QTR1A_TEST");
  Serial.println("Place on BLACK ground, then press START");
  waitStart();
  calibrateBaseline();
  Serial.print("BASELINE=");
  Serial.println(g_baseline);
}

void loop() {
  int v = analogRead(PIN_QTR1A);
  int delta = v - g_baseline;
  bool white = v > (g_baseline + QTR_MARGIN);

  Serial.print("raw=");
  Serial.print(v);
  Serial.print(" delta=");
  Serial.print(delta);
  Serial.print(" white=");
  Serial.println(white ? 1 : 0);

  delay(60);
}
