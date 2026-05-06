// --- Solvex-3.ino pinout (PA/PB ile bağlanan sistem) ---

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include "I2Cdev.h"


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


static uint16_t readCmF() {
  digitalWrite(HFT, LOW);
  delayMicroseconds(2);
  digitalWrite(HFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(HFT, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
  if (!us) return 0;

  uint16_t cmF = (uint16_t)(us / 58UL);
  if (cmF > 400) cmF = 400;
  return cmF;
}

static uint16_t readCmL() {
  digitalWrite(HLT, LOW);
  delayMicroseconds(2);
  digitalWrite(HLT, HIGH);
  delayMicroseconds(10);
  digitalWrite(HLT, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
  if (!us) return 0;

  uint16_t cmL = (uint16_t)(us / 58UL);
  if (cmL > 400) cmL = 400;
  return cmL;
}

static uint16_t readCmR() {
  digitalWrite(HRT, LOW);
  delayMicroseconds(2);
  digitalWrite(HRT, HIGH);
  delayMicroseconds(10);
  digitalWrite(HRT, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
  if (!us) return 0;

  uint16_t cmR = (uint16_t)(us / 58UL);
  if (cmR > 400) cmR = 400;
  return cmR;
}

void setup() {
Serial.begin(115600);
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
pinMode(R_IN1, OUTPUT);
pinMode(R_IN2, OUTPUT);
pinMode(L_IN1, OUTPUT);
pinMode(L_IN2, OUTPUT);



}

void loop() {
uint16_t cmF = readCmF();
delayMicroseconds(30);
uint16_t cmL = readCmL();
delayMicroseconds(30);
uint16_t cmR = readCmR();
delayMicroseconds(30);






}

void ileri(){ //motorlar ileri fonksiyonu
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);

}

void geri(){ //motorlar geri fonksiyonu
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void sag(){ //sağa dönüş fonksiyonu(sol teker sabit sağ teker hareketli)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 0);
    
}

void sol(){ // sola dönüş fonksiyonu(sağ teker sabit sol teker hareketli)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 0);
analogWrite(R_PWM, 255);
    
}

void sag360(){ //sağa 360 derece dönüş fonksiyonu(sol teker geri sağ teker ileri)
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);   

digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}   
void sol360(){ //sola 360 derece dönüş fonksiyonu(sağ teker geri sol teker ileri)
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);

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


digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


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
