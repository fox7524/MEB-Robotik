// ==========================================
// 1. PIN DEFINITIONS

//B-1A 8
//B-1B 9
//A-1A 7
//A-1B 6
// ==========================================

const int MOTOR_LEFT_A  = 8; 
const int MOTOR_LEFT_B  = 9; 
const int MOTOR_RIGHT_A = 7; 
const int MOTOR_RIGHT_B = 6; 

const int TRIG_FRONT = 4;
const int ECHO_FRONT = 5;
const int TRIG_LEFT  = 2;
const int ECHO_LEFT  = 3;

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
  Serial.println();

  // 2. Motor Test Sequence (Changes state every 2 seconds)
  
  moveForward();
  delay(1000);
  moveBackward();
  delay(1000);
  

}