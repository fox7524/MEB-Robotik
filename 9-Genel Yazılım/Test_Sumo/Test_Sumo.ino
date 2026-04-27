// ==========================================
// 1. PIN DEFINITIONS
// ==========================================

const int MOTOR_LEFT_A  = 2; 
const int MOTOR_LEFT_B  = 3; 
const int MOTOR_RIGHT_A = 4; 
const int MOTOR_RIGHT_B = 5; 

const int TRIG_FRONT = 6;
const int ECHO_FRONT = 7;
const int TRIG_LEFT  = 8;
const int ECHO_LEFT  = 9;

const int START_MODULE = 10; // Start module pin

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void setMotors(bool lA, bool lB, bool rA, bool rB) {
  digitalWrite(MOTOR_LEFT_A, lA);
  digitalWrite(MOTOR_LEFT_B, lB);
  digitalWrite(MOTOR_RIGHT_A, rA);
  digitalWrite(MOTOR_RIGHT_B, rB);
}

void moveForward()  { setMotors(HIGH, LOW, HIGH, LOW); }
void moveBackward() { setMotors(LOW, HIGH, LOW, HIGH); }
void stopMotors()   { setMotors(LOW, LOW, LOW, LOW); }

// ==========================================
// 3. SETUP
// ==========================================
void setup() {
  Serial.begin(9600); 

  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(START_MODULE, INPUT); 
  
  stopMotors();
  Serial.println("--- Test Code Started ---");
}

// ==========================================
// 4. TEST LOOP
// ==========================================
void loop() {
  // 1. Read Sensors & Start Module
  int startState = digitalRead(START_MODULE);
  long frontDist = getDistance(TRIG_FRONT, ECHO_FRONT);
  long leftDist  = getDistance(TRIG_LEFT, ECHO_LEFT);

  // Print Start Module State
  Serial.print("Start Pin: ");
  Serial.print(startState == HIGH ? "HIGH" : "LOW ");
  Serial.print(" \t|\t ");

  // Print Ultrasonic Distances
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm \t|\t Left: ");
  Serial.print(leftDist);
  Serial.print(" cm \t|\t ");

  // 2. Motor Test Sequence (Changes state every 2 seconds)
  unsigned long currentTime = millis();
  int cyclePhase = (currentTime / 2000) % 4; 

  if (cyclePhase == 0) {
    moveForward();
    Serial.println("Motors: FORWARD");
  } 
  else if (cyclePhase == 1) {
    stopMotors();
    Serial.println("Motors: STOP");
  } 
  else if (cyclePhase == 2) {
    moveBackward();
    Serial.println("Motors: BACKWARD");
  } 
  else if (cyclePhase == 3) {
    stopMotors();
    Serial.println("Motors: STOP");
  }

  delay(250); // Read everything 4 times a second
}