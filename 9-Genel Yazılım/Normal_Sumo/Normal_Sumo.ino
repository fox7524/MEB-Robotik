// L9110 Motor Driver Pins
const int MOTOR_LEFT_A  = 2; // Left Motor Forward
const int MOTOR_LEFT_B  = 3; // Left Motor Reverse
const int MOTOR_RIGHT_A = 4; // Right Motor Forward
const int MOTOR_RIGHT_B = 5; // Right Motor Reverse

// HC-SR04 Ultrasonic Sensor Pins
const int TRIG_FRONT = 6;
const int ECHO_FRONT = 7;
const int TRIG_LEFT  = 8;
const int ECHO_LEFT  = 9;

// Start Module Pin
const int START_MODULE = 10; 

// Battle Constants
const int DETECTION_DIST = 40; // Max distance to register an enemy (in cm)

// ==========================================
// 2. BOTTOM-UP HELPER FUNCTIONS
// ==========================================

// -- Sensor Function --
// Returns distance in cm. Uses a timeout to prevent lag if no echo is received.
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  if (duration == 0) return 999; // Return high value if nothing detected
  
  return duration * 0.034 / 2;
}

// -- Motor Functions --
void setMotors(bool lA, bool lB, bool rA, bool rB) {
  digitalWrite(MOTOR_LEFT_A, lA);
  digitalWrite(MOTOR_LEFT_B, lB);
  digitalWrite(MOTOR_RIGHT_A, rA);
  digitalWrite(MOTOR_RIGHT_B, rB);
}

void moveForward() { setMotors(HIGH, LOW, HIGH, LOW); }
void turnLeft()    { setMotors(LOW, HIGH, HIGH, LOW); } // Left backward, Right forward
void turnRight()   { setMotors(HIGH, LOW, LOW, HIGH); } // Left forward, Right backward
void stopMotors()  { setMotors(LOW, LOW, LOW, LOW); }

// ==========================================
// 3. SETUP
// ==========================================
void setup() {
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // Assuming the start module pulls HIGH when the match begins
  pinMode(START_MODULE, INPUT); 
  
  stopMotors();
}

// ==========================================
// 4. MAIN BATTLE LOOP
// ==========================================
void loop() {
  // Check start module (Wait/Stop if signal is LOW)
  if (digitalRead(START_MODULE) == LOW) {
    stopMotors();
    return; // Skip the rest of the loop until started
  }

  // Read sensors
  long frontDist = getDistance(TRIG_FRONT, ECHO_FRONT);
  long leftDist  = getDistance(TRIG_LEFT, ECHO_LEFT);

  // Strategy Logic
  if (frontDist < DETECTION_DIST) {
    // Enemy is in front -> Charge!
    moveForward();
  } 
  else if (leftDist < DETECTION_DIST) {
    // Enemy is on the left -> Turn left to bring them to the front!
    turnLeft();
  } 
  else {
    // No enemy detected -> Search by spinning right
    // (Spinning right efficiently covers the blind spot while the left sensor scans)
    turnRight(); 
  }

  delay(10); // Short delay for loop stability
}