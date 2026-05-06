// --- Solvex-3.ino pinout (PA/PB ile bağlanan sistem) ---

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include "I2Cdev.h"

// --- SENSOR THRESHOLDS ---
const int THRESHOLD_FRONT = 15; // 15cm for the front wall
const int THRESHOLD_SIDE  = 3; // 3cm for the side walls

const int BUILTIN_LED = PC13;
const int PIN_START_BTN = PB5;

const int PIN_QTR1A = PA0;

const int HFT = PA1;
const int HLT  = PA2;
const int HRT = PA3;

const int HFE = PA4;
const int HLE  = PA5;
const int HRE = PA6;


const int R_PWM = PB0;
const int L_PWM = PB1;
const int R_IN1 = PB12;
const int R_IN2 = PB13;
const int L_IN1 = PB14;
const int L_IN2 = PB15;

// Encoder (C1/C2 = A/B)
const int PIN_ENC_L_A = PB10; // C1
const int PIN_ENC_R_A = PB11; // C1
const int PIN_ENC_L_B = PB8;  // C2
const int PIN_ENC_R_B = PB9;  // C2

// ==========================================
// ENCODER SETUP  AND... ADD SOME MAGIC
// ==========================================
volatile long g_encL = 0;
volatile long g_encR = 0;
const long TICKS_PER_CELL = 3057; // 20cm of travel for your exact wheels

// These tiny functions run automatically in the background
void isrEncL() { g_encL++; }
void isrEncR() { g_encR++; }

// We will call this inside setup()
void setupEncoders() {
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  
  // Attach interrupts so the STM32 counts ticks automatically
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);
}


static uint16_t readCmF() {
  digitalWrite(HFT, LOW); 
  delayMicroseconds(2);
  digitalWrite(HFT, HIGH); 
  delayMicroseconds(10);
  digitalWrite(HFT, LOW);

  unsigned long us = pulseIn(HFE, HIGH, 30000UL); // Using specific echo pin
  if (!us) return 999; // If no echo, the path is wide open!

  uint16_t cm = (uint16_t)(us / 58UL);
  return cm;
}

static uint16_t readCmL() {
  digitalWrite(HLT, LOW); 
  delayMicroseconds(2);
  digitalWrite(HLT, HIGH); 
  delayMicroseconds(10);
  digitalWrite(HLT, LOW);
  unsigned long us = pulseIn(HLE, HIGH, 30000UL);
  if (!us) return 999;
  return (uint16_t)(us / 58UL);
}

static uint16_t readCmR() {
  digitalWrite(HRT, LOW); 
  delayMicroseconds(2);
  digitalWrite(HRT, HIGH); 
  delayMicroseconds(10);
  digitalWrite(HRT, LOW);
  unsigned long us = pulseIn(HRE, HIGH, 30000UL);
  if (!us) return 999;
  return (uint16_t)(us / 58UL);
}

// --- BOOLEAN CONVERTERS ---
bool isWallFront() {
  uint16_t dist = readCmF();
  return (dist < THRESHOLD_FRONT);
}

bool isWallLeft() {
  uint16_t dist = readCmL();
  return (dist < THRESHOLD_SIDE);
}

bool isWallRight() {
  uint16_t dist = readCmR();
  return (dist < THRESHOLD_SIDE);
}

void setup() {
setupEncoders();
Serial.begin(115200);
Serial.print("merheaba");    
pinMode(HFE, INPUT);
pinMode(HLE, INPUT);
pinMode(HRE, INPUT);
pinMode(PIN_START_BTN, INPUT);
pinMode(PIN_QTR1A, INPUT);
pinMode(PIN_ENC_L_A, INPUT);
pinMode(PIN_ENC_R_A, INPUT);
pinMode(PIN_ENC_L_B, INPUT);
pinMode(PIN_ENC_R_B, INPUT);
pinMode(HFT, OUTPUT);
pinMode(HLT, OUTPUT);
pinMode(HRT, OUTPUT);
pinMode(R_PWM, OUTPUT);
pinMode(R_IN1, OUTPUT);
pinMode(R_IN2, OUTPUT);
pinMode(L_PWM, OUTPUT);
pinMode(L_IN1, OUTPUT);
pinMode(L_IN2, OUTPUT);



}

// ==========================================
// SMART MOVEMENT
// ==========================================
void stepForwardOneCell() {
  // 1. Reset counters to zero
  g_encL = 0;
  g_encR = 0;
  
  // 2. Turn motors on
  ileri(); 
  
  // 3. Wait until the average of both wheels hits our Magic Number
  while ( ((g_encL + g_encR) / 2) < TICKS_PER_CELL ) {
    // The STM32 just waits here. 
    // The background interrupts are counting the ticks!
    delay(1); 
  }
  
  // 4. Hit the brakes hard!
  anidur(); 
  delay(150); // Let the robot settle before checking sensors again

}
void loop() {
// 1. Take a snapshot of the walls
  bool wallL = isWallLeft();
  delay(35); // MUST have this delay to prevent sensor cross-talk
  
  bool wallF = isWallFront();
  delay(35); 
  
  bool wallR = isWallRight();
  delay(35);

  // 2. Apply the Left-Hand Rule Priorities
  if (wallL == false) {
    // Priority 1: Left is open. Turn Left, then step forward.
    sol360(); 
    delay(290);
    dur(); 
    delay(100);
    stepForwardOneCell();
  } 
  else if (wallF == false) {
    // Priority 2: Left blocked, Front open. Step forward.
    stepForwardOneCell();
  } 
  else if (wallR == false) {
    // Priority 3: Left & Front blocked, Right open. Turn Right, step forward.
    sag360(); 
    delay(290);
    dur();
    delay(100);
    stepForwardOneCell();
  } 
  else {
    // Priority 4: Dead end. Turn Around, step forward.
    sag360(); 
    delay(580);
    dur(); 
    delay(100);
    stepForwardOneCell();
  }

}

void ileri(){ //motorlar ileri fonksiyonu
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);

}

void geri(){ //motorlar geri fonksiyonu
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN2, HIGH);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void sag(){ //sağa dönüş fonksiyonu(sol teker sabit sağ teker hareketli)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 0);
    
}

void sol(){ // sola dönüş fonksiyonu(sağ teker sabit sol teker hareketli)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);


analogWrite(L_PWM, 0);
analogWrite(R_PWM, 255);
    
}

void sag360(){ //sağa 360 derece dönüş fonksiyonu(sol teker geri sağ teker ileri)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);   

digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, HIGH);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}   
void sol360(){ //sola 360 derece dönüş fonksiyonu(sağ teker geri sol teker ileri)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN2, LOW);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void dur(){ //durma fonksiyonu(tekerler serbest)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);

digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);

analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);
    
}

void anidur(){ //ani durma fonksiyonu(tekerler kitli)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN2, HIGH);
digitalWrite(R_IN2, HIGH);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
delay(25);
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);

digitalWrite(L_IN2, LOW);
digitalWrite(R_IN2, LOW);

analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);
    
}

void tileri(){ //ileriyi tarama fonskiyonu
  gethcsrf();

}

void tsag(){ //sağı tarama fonksiyonu
  gethcsrr();

}

void tsol(){ //solu tarama fonksiyonu
  gethcsrl();

}

void search(){ // labirent çözme algoritması fonksiyonu

//“Frontier-based exploration + BFS (flood-fill)”: keşif hedefi olarak “frontier” (bilinmeyen kenarı olan en yakın hücre) seçiliyor, o hedefe en kısa yol BFS ile bulunup adım adım ilerleniyor.

}

void getmpu(){ //mpu6050 sensöründen veri okuma fonksiyonu
 
}

bool qtrIsWhite() { //qtr sensöründen veri okuma fonksiyonu
  if (BYPASS_QTR1A) return false;
  uint16_t v = analogRead(PIN_QTR1A);
  return v > QTR_WHITE_THRESHOLD; 
}