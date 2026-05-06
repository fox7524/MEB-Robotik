#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// --- SENSOR THRESHOLDS ---
const int THRESHOLD_FRONT = 15; 
const int THRESHOLD_SIDE  = 8;  // Increased for better centering room
const int TARGET_DIST     = 5;  // Ideal distance from a side wall (cm)

// Pin Definitions (STM32)
const int HFT = PA1; const int HLT = PA2; const int HRT = PA3;
const int HFE = PA4; const int HLE = PA5; const int HRE = PA6;
const int R_PWM = PB0;  const int L_PWM = PB1;
const int R_IN1 = PB12; const int R_IN2 = PB13;
const int L_IN1 = PB14; const int L_IN2 = PB15;
const int PIN_ENC_L_A = PB10; const int PIN_ENC_R_A = PB11;

MPU6050 mpu;
volatile long g_encL = 0;
volatile long g_encR = 0;
const long TICKS_PER_CELL = 3057; 

void isrEncL() { g_encL++; }
void isrEncR() { g_encR++; }

// --- SENSOR HELPERS ---
uint16_t getDist(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 20000UL);
  return (us == 0) ? 999 : (uint16_t)(us / 58UL);
}

// --- MOTOR PRIMITIVES ---
void setMotors(int left, int right) {
  digitalWrite(L_IN1, left > 0);  digitalWrite(L_IN2, left < 0);
  digitalWrite(R_IN1, right > 0); digitalWrite(R_IN2, right < 0);
  analogWrite(L_PWM, abs(left));
  analogWrite(R_PWM, abs(right));
}

void stopRobot(bool hard = false) {
  if(hard) {
    setMotors(255, 255); // Short reverse/brake
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, HIGH);
    delay(40);
  }
  setMotors(0, 0);
}

// --- THE FIX: SMART MOVEMENT ---
void stepForwardOneCell() {
  g_encL = 0; g_encR = 0;
  int baseSpeed = 180; // Lowered for better control

  while (((g_encL + g_encR) / 2) < TICKS_PER_CELL) {
    uint16_t dL = getDist(HLT, HLE);
    uint16_t dR = getDist(HRT, HRE);
    
    int correction = 0;
    // If we see a wall on the left, try to stay 5cm away
    if (dL < 12) correction = (dL - TARGET_DIST) * 15; 
    // If no left wall but a right wall, stay 5cm away from right
    else if (dR < 12) correction = (TARGET_DIST - dR) * 15;

    setMotors(baseSpeed + correction, baseSpeed - correction);
    
    // Safety: If something suddenly appears in front, stop!
    if (getDist(HFT, HFE) < 5) break; 
    delay(10); 
  }
  stopRobot(true);
  delay(150);
}

// --- SETUP & LOOP ---
void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();
  
  pinMode(HFT, OUTPUT); pinMode(HLT, OUTPUT); pinMode(HRT, OUTPUT);
  pinMode(HFE, INPUT);  pinMode(HLE, INPUT);  pinMode(HRE, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);
  
  delay(1000); // Wait for sensors to settle
}

void loop() {
  uint16_t distL = getDist(HLT, HLE); delay(30);
  uint16_t distF = getDist(HFT, HFE); delay(30);
  uint16_t distR = getDist(HRT, HRE);

  // Left-Hand Rule Logic
  if (distL > 10) { // Left is open
    setMotors(-200, 200); delay(280); stopRobot(true); delay(100);
    stepForwardOneCell();
  } 
  else if (distF > THRESHOLD_FRONT) { // Front is open
    stepForwardOneCell();
  } 
  else if (distR > 10) { // Right is open
    setMotors(200, -200); delay(280); stopRobot(true); delay(100);
    stepForwardOneCell();
  } 
  else { // Dead end
    setMotors(200, -200); delay(560); stopRobot(true); delay(100);
    stepForwardOneCell();
  }
}