// ============================================
//   Mini Sumo Robot - Arduino Nano
//   2x HC-SR04 Ultrasonic + L298N Motor Driver
// ============================================

// --- HC-SR04 Front Ultrasonic Sensor ---
#define ECHO_FRONT  2
#define TRIG_FRONT  3

// --- HC-SR04 Right Ultrasonic Sensor ---
#define ECHO_RIGHT  4
#define TRIG_RIGHT  8

// --- Motor B (Left Motor) ---
#define B_1A  5
#define B_1B  6

// --- Motor A (Right Motor) ---
#define A_1A  9
#define A_1B  11

// --- Start Button ---
#define START_PIN 7

// --- Settings ---
#define ATTACK_SPEED      255
#define SEARCH_SPEED      180
#define DETECT_DISTANCE   40
#define STARTUP_DELAY     3000

void setup() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(A_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(START_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("Butona Bas...");
  stopMotors();

  while (digitalRead(START_PIN) == LOW) { delay(10); }

  Serial.println("Baslatiliyor...");
  delay(STARTUP_DELAY);
  Serial.println("GO!");
}

void loop() {
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (distFront > 0 && distFront <= DETECT_DISTANCE) {
    moveForward(ATTACK_SPEED);
  } else if (distRight > 0 && distRight <= DETECT_DISTANCE) {
    spinRight(SEARCH_SPEED);
    delay(200);
    moveForward(ATTACK_SPEED);
  } else {
    spinRight(SEARCH_SPEED);
  }
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void moveForward(int speed) {
  analogWrite(B_1A, speed); analogWrite(B_1B, 0);
  analogWrite(A_1A, speed); analogWrite(A_1B, 0);
}

void moveBackward(int speed) {
  analogWrite(B_1A, 0); analogWrite(B_1B, speed);
  analogWrite(A_1A, 0); analogWrite(A_1B, speed);
}

void spinRight(int speed) {
  analogWrite(B_1A, speed); analogWrite(B_1B, 0);
  analogWrite(A_1A, 0); analogWrite(A_1B, speed);
}

void spinLeft(int speed) {
  analogWrite(B_1A, 0); analogWrite(B_1B, speed);
  analogWrite(A_1A, speed); analogWrite(A_1B, 0);
}

void stopMotors() {
  analogWrite(B_1A, 0); analogWrite(B_1B, 0);
  analogWrite(A_1A, 0); analogWrite(A_1B, 0);
}