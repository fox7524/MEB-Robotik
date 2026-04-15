#include <Arduino.h>

const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;


void setup() {
pinMode(FL_PWM, OUTPUT);
pinMode(FL_IN1, OUTPUT);
pinMode(FL_IN2, OUTPUT);

pinMode(FR_PWM, OUTPUT);
pinMode(FR_IN1, OUTPUT);
pinMode(FR_IN2, OUTPUT);
Serial.begin(115200);
Serial.print("Sistem Başladı");

}

void loop() {



}


void ileri(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void geri(int pwm){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sagslide(int pwm){

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void solslide(int pwm){

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sol360(int pwm){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void sag360(int pwm){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, pwm);
analogWrite(FR_PWM, pwm);
analogWrite(RL_PWM, pwm);
analogWrite(RR_PWM, pwm);

}

void dur(){

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void anidur(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, HIGH);

analogWrite(FL_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(RR_PWM, 255);

delay(30);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);

}

void sagon(){
digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, LOW);

analogWrite(FR_PWM, 255);

}

void solon(){
digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, LOW);

analogWrite(FL_PWM, 255);

}

void sagarka(){
digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, LOW);

analogWrite(RR_PWM, 255);

}

void solarka(){
digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, LOW);

analogWrite(RL_PWM, 255);

}

void eup(int pwma){

digitalWrite(E_IN1, LOW);
digitalWrite(E_IN2, HIGH);

analogWrite(E_PWM, pwma);

}

void edown(int pwma){

digitalWrite(E_IN1, HIGH);
digitalWrite(E_IN2, LOW);

analogWrite(E_PWM, pwma);

}


void adur(){

digitalWrite(E_IN1, HIGH);
digitalWrite(E_IN2, HIGH);

analogWrite(E_PWM, 255);

delay(10);

  digitalWrite(E_IN1, LOW);
digitalWrite(E_IN2, LOW);

  analogWrite(E_PWM, 0);
}

void fanidur(){

digitalWrite(FL_IN1, HIGH);
digitalWrite(FL_IN2, HIGH);

digitalWrite(FR_IN1, HIGH);
digitalWrite(FR_IN2, HIGH);

digitalWrite(RL_IN1, HIGH);
digitalWrite(RL_IN2, HIGH);

digitalWrite(RR_IN1, HIGH);
digitalWrite(RR_IN2, HIGH);

digitalWrite(E_IN1, HIGH);
digitalWrite(E_IN2, HIGH);

analogWrite(FL_PWM, 255);
analogWrite(FR_PWM, 255);
analogWrite(RL_PWM, 255);
analogWrite(RR_PWM, 255);
analogWrite(E_PWM, 255);

delay(10);

digitalWrite(FL_IN1, LOW);
digitalWrite(FL_IN2, LOW);

digitalWrite(FR_IN1, LOW);
digitalWrite(FR_IN2, LOW);

digitalWrite(RL_IN1, LOW);
digitalWrite(RL_IN2, LOW);

digitalWrite(RR_IN1, LOW);
digitalWrite(RR_IN2, LOW);

digitalWrite(E_IN1, LOW);
digitalWrite(E_IN2, LOW);

analogWrite(FL_PWM, 0);
analogWrite(FR_PWM, 0);
analogWrite(RL_PWM, 0);
analogWrite(RR_PWM, 0);
analogWrite(E_PWM, 0);


}


//veeee
//son
