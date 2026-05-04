// --- Solvex-3.ino pinout (PA/PB ile bağlanan sistem) ---

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include "I2Cdev.h"



const int PIN_START_BTN = PB5;

const int PIN_QTR1A = PA0;

const int HFT = PA1;
const int HLT  = PA2;
const int HRT = PA3;

const int H_F_E = PA4;
const int H_L_E  = PA5;
const int H_R_E = PA6;


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


void setup() {
Serial.begin(115600);
Serial.print("merheaba");    
pinMode(H_F_E ,INPUT);
pinMode(H_L_E ,INPUT);
pinMode(H_R_E ,INPUT);
pinMode(PIN_START_BTN ,INPUT);
pinMode(PIN_QTR1A ,INPUT);
pinMode(PIN_ENC_L_A ,INPUT);
pinMode(PIN_ENC_R_A ,INPUT);
pinMode(PIN_ENC_L_B ,INPUT);
pinMode(PIN_ENC_R_B ,INPUT);
pinMode(HFT ,OUTPUT);
pinMode(HLT ,OUTPUT);
pinMode(HRT ,OUTPUT);
pinMode(R_IN1 ,OUTPUT);
pinMode(R_IN2 ,OUTPUT);
pinMode(L_IN1 ,OUTPUT);
pinMode(L_IN2 ,OUTPUT);



}

void loop() {


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

void sag(){. //sağa dönüş fonksiyonu(sol teker sabit sağ teker hareketli)
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


    
}

void tsag(){ //sağı tarama fonksiyonu

    
}

void tsol(){ //solu tarama fonksiyonu

    
}

void search(){ // labirent çözme algoritması fonksiyonu

//“Frontier-based exploration + BFS (flood-fill)”: keşif hedefi olarak “frontier” (bilinmeyen kenarı olan en yakın hücre) seçiliyor, o hedefe en kısa yol BFS ile bulunup adım adım ilerleniyor.

}

void getmpu(){ //mpu6050 sensöründen veri okuma fonksiyonu


}

void gethcsrf(){ //hcsr04 ön sensöründen veri okuma fonksiyonu

    
}

void gethcsrl(){ //hcsr04 sol sensöründen veri okuma fonksiyonu


}

void gethcsrr(){ //hcsr04 sağ sensöründen veri okuma fonksiyonu

    
}
