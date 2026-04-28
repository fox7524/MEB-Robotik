static const int TRIG_PIN = 6;

static const int ECHO_PINS[3] = {2, 3, 4};
static const char* NAMES[3] = {"FRONT", "LEFT", "RIGHT"};

static const unsigned long ECHO_TIMEOUT_US = 25000;
static const unsigned long BETWEEN_PINGS_MS = 60;

static unsigned long pulseEchoUs(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
}

static int readCmOnce(int echoPin) {
  unsigned long us = pulseEchoUs(echoPin);
  if (us == 0) return -1;
  return (int)(us / 58);
}

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

static int readCmFiltered(int echoPin) {
  int a = readCmOnce(echoPin); delay(5);
  int b = readCmOnce(echoPin); delay(5);
  int c = readCmOnce(echoPin);
  if (a < 0) return (b >= 0 && c >= 0) ? ((b + c) / 2) : ((b >= 0) ? b : c);
  if (b < 0) return (a >= 0 && c >= 0) ? ((a + c) / 2) : ((a >= 0) ? a : c);
  if (c < 0) return (a >= 0 && b >= 0) ? ((a + b) / 2) : ((a >= 0) ? a : b);
  return median3(a, b, c);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i < 3; i++) pinMode(ECHO_PINS[i], INPUT);
}

void loop() {
  for (int i = 0; i < 3; i++) {
    int cm = readCmFiltered(ECHO_PINS[i]);

    Serial.print(NAMES[i]);
    Serial.print('=');
    if (cm < 0) Serial.print("NA");
    else Serial.print(cm);
    Serial.print("cm");

    if (i < 2) Serial.print("  ");
    delay(BETWEEN_PINGS_MS);
  }
  Serial.println();
}