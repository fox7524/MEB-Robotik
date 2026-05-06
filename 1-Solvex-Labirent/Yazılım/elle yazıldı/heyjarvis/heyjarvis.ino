#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// --- SENSOR THRESHOLDS ---
const int THRESHOLD_FRONT = 15; 
const int THRESHOLD_SIDE  = 4; // Distance in cm to trigger a nudge

const int HFT = PA1; const int HLT = PA2; const int HRT = PA3;
const int HFE = PA4; const int HLE = PA5; const int HRE = PA6;

const int R_PWM = PB0;  const int L_PWM = PB1;
const int R_IN1 = PB12; const int R_IN2 = PB13;
const int L_IN1 = PB14; const int L_IN2 = PB15;

const int PIN_ENC_L_A = PB10; const int PIN_ENC_R_A = PB11;

volatile long g_encL = 0;
volatile long g_encR = 0;
const long TICKS_PER_CELL = 3057; 

void isrEncL() { g_encL++; }
void isrEncR() { g_encR++; }

// --- SENSOR READING ---
uint16_t readDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 25000UL);
  if (us == 0) return 999;
  return (uint16_t)(us / 58UL);
}

// --- MOTOR CONTROLS ---
void motorDrive(int leftSpeed, int rightSpeed) {
  // Left Motor
  digitalWrite(L_IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(L_IN2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(L_PWM, abs(leftSpeed));
  
  // Right Motor
  digitalWrite(R_IN1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(R_IN2, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(R_PWM, abs(rightSpeed));
}

void anidur() {
  motorDrive(255, 255); // Brake
  delay(30);
  motorDrive(0, 0);
}

// --- THE NUDGE MOVEMENT ---
void stepForwardOneCell() {
  g_encL = 0; g_encR = 0;
  int baseSpeed = 175; // Safer speed for sensing

  while (((g_encL + g_encR) / 2) < TICKS_PER_CELL) {
    uint16_t dL = readDistance(HLT, HLE);
    delay(10); // Small pause for sensor
    uint16_t dR = readDistance(HRT, HRE);

    if (dL < THRESHOLD_SIDE) {
      // Too close to left -> veer right
      motorDrive(baseSpeed + 45, baseSpeed - 45);
    } 
    else if (dR < THRESHOLD_SIDE) {
      // Too close to right -> veer left
      motorDrive(baseSpeed - 45, baseSpeed + 45);
    } 
    else {
      // Center is clear
      motorDrive(baseSpeed, baseSpeed);
    }

    // Safety: If wall appears in front suddenly
    if (readDistance(HFT, HFE) < 6) break;
  }
  anidur();
  delay(100);
}

void setup() {
  Serial.begin(115200);
  pinMode(HFT, OUTPUT); pinMode(HLT, OUTPUT); pinMode(HRT, OUTPUT);
  pinMode(HFE, INPUT);  pinMode(HLE, INPUT);  pinMode(HRE, INPUT);
  
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);
}

void loop() {
  // 1. Scan
  uint16_t wallL = readDistance(HLT, HLE); delay(35);
  uint16_t wallF = readDistance(HFT, HFE); delay(35);
  uint16_t wallR = readDistance(HRT, HRE); delay(35);

  // 2. Left-Hand Rule Logic
  if (wallL > 12) { // Left path open
    motorDrive(-200, 200); // Turn Left
    delay(285);
    anidur();
    stepForwardOneCell();
  } 
  else if (wallF > 15) { // Front path open
    stepForwardOneCell();
  } 
  else if (wallR > 12) { // Right path open
    motorDrive(200, -200); // Turn Right
    delay(285);
    anidur();
    stepForwardOneCell();
  } 
  else { // Dead end
    motorDrive(200, -200); // U-Turn
    delay(570);
    anidur();
    stepForwardOneCell();
  }
}