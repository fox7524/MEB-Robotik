#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 115200;

static const int PIN_START_BTN = PB5;

static const int R_IN1 = PB12;
static const int R_IN2 = PB13;
static const int R_PWM = -1;

static const int L_IN1 = PB14;
static const int L_IN2 = PB15;
static const int L_PWM = -1;

static const int PWM_MAX = 255;
static const int DRIVE_PWM = 200;
static const int TURN_PWM = 200;

static void motorStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  if (L_PWM >= 0) analogWrite(L_PWM, 0);
  if (R_PWM >= 0) analogWrite(R_PWM, 0);
}

static void motorSet(int left, int right) {
  left = constrain(left, -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

  if (left == 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    if (L_PWM >= 0) analogWrite(L_PWM, 0);
  } else if (left > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    if (L_PWM >= 0) analogWrite(L_PWM, left);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    if (L_PWM >= 0) analogWrite(L_PWM, -left);
  }

  if (right == 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    if (R_PWM >= 0) analogWrite(R_PWM, 0);
  } else if (right > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    if (R_PWM >= 0) analogWrite(R_PWM, right);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    if (R_PWM >= 0) analogWrite(R_PWM, -right);
  }
}

static void waitStart() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  while (digitalRead(PIN_START_BTN) == HIGH) {
    motorStop();
    delay(5);
  }
  delay(250);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  if (R_PWM >= 0) pinMode(R_PWM, OUTPUT);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  if (L_PWM >= 0) pinMode(L_PWM, OUTPUT);

  motorStop();

  Serial.println("MOTOR_TEST");
  Serial.println("Press START to begin");
  waitStart();
  Serial.println("RUN");
}

void loop() {
  Serial.println("FWD");
  motorSet(DRIVE_PWM, DRIVE_PWM);
  delay(1200);
  motorStop();
  delay(400);

  Serial.println("REV");
  motorSet(-DRIVE_PWM, -DRIVE_PWM);
  delay(1200);
  motorStop();
  delay(400);

  Serial.println("PIVOT_R");
  motorSet(TURN_PWM, -TURN_PWM);
  delay(450);
  motorStop();
  delay(400);

  Serial.println("PIVOT_L");
  motorSet(-TURN_PWM, TURN_PWM);
  delay(450);
  motorStop();
  delay(800);
}
