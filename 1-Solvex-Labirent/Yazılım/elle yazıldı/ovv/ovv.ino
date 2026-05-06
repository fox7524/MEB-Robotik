

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
const int PIN_ENC_L_A = PB10; 
const int PIN_ENC_R_A = PB11; 
const int PIN_ENC_L_B = PB8;  
const int PIN_ENC_R_B = PB9;  

// --- TUNING CONSTANTS ---
static constexpr uint16_t WALL_THRESHOLD_CM = 9; 
static constexpr uint16_t CELL_TICKS = 450;     // Distance to move one cell
static constexpr uint16_t TURN_TICKS = 130;     // Ticks for 90 degree turn
static constexpr uint16_t QTR_THRESHOLD = 2500;

static constexpr uint8_t MAZE_W = 8;
static constexpr uint8_t MAZE_H = 16;

enum Dir : uint8_t { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };
static constexpr uint8_t DIR_BIT[4] = { 1u << 0, 1u << 1, 1u << 2, 1u << 3 };
static constexpr int8_t DX[4] = { 0, 1, 0, -1 };
static constexpr int8_t DY[4] = { -1, 0, 1, 0 };

struct Cell {
  uint8_t walls;
  uint8_t known;
  uint8_t visited;
};

static Cell g_maze[MAZE_H][MAZE_W];
static uint8_t g_x = 0;
static uint8_t g_y = 0;
static uint8_t g_dir = DIR_E;

static uint16_t g_front_cm = 0;
static uint16_t g_left_cm = 0;
static uint16_t g_right_cm = 0;
static uint8_t g_wall_front = 0;
static uint8_t g_wall_left = 0;
static uint8_t g_wall_right = 0;

static float g_yaw_deg = 0.0f;
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.print("merheaba");    
  
  pinMode(H_F_E, OUTPUT); // Trig must be output
  pinMode(H_L_E, OUTPUT);
  pinMode(H_R_E, OUTPUT);
  pinMode(PIN_START_BTN, INPUT);
  pinMode(PIN_QTR1A, INPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  pinMode(HFT, OUTPUT); // Echo is input (handled in ping)
  pinMode(HLT, OUTPUT);
  pinMode(HRT, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN1, OUTPUT); 
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  Wire.begin();
  mpu.initialize();
  if(!mpu.testConnection()) {
    // If MPU fails, we continue but rely on encoders for turns
  }

  // Wait for button press to start
  while(digitalRead(PIN_START_BTN) == LOW);
  delay(500);
}

void loop() {
  // 1. Finish Line Detection (QTR)
  if (analogRead(PIN_QTR1A) > QTR_THRESHOLD) {
    dur();
    while(true) { delay(1000); } // Stop and wait forever
  }

  // 2. Run the Solver
  search();
}

void ileri(){ 
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, 255); analogWrite(R_PWM, 255);
}

void geri(){ 
  digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, 255); analogWrite(R_PWM, 255); // Fixed logic
}

void sag(){ 
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, 200);   analogWrite(R_PWM, 0);
}

void sol(){ 
  digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, 0);     analogWrite(R_PWM, 200);
}

void sag360(){ 
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, 255);   analogWrite(R_PWM, 255);
}   

void sol360(){ 
  digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, 255);   analogWrite(R_PWM, 255);
}

void dur(){ 
  digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, 0);     analogWrite(R_PWM, 0);
}

void anidur(){ 
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, 255);   analogWrite(R_PWM, 255);
  delay(25);
  dur();
}

void tileri(){ 
  gethcsrf();
  g_wall_front = (g_front_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

void tsag(){ 
  gethcsrr();
  g_wall_right = (g_right_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

void tsol(){ 
  gethcsrl();
  g_wall_left = (g_left_cm < WALL_THRESHOLD_CM) ? 1 : 0;
}

// --- NAVIGATION ENGINE: RIGHT HAND RULE ---
void search(){ 
  // 1. Scan all directions sequentially to prevent interference
  tileri(); delay(30);
  tsag();   delay(30);
  tsol();   delay(30);

  // 2. Decision Logic (Right-Hand Rule)
  if (g_wall_front == 1) {
    // Path blocked! Try turning Right.
    sag();
    // Use encoders to ensure a clean 90 degree turn
    uint16_t ticks = 0;
    uint8_t lastLA = digitalRead(PIN_ENC_L_A);
    uint8_t lastRA = digitalRead(PIN_ENC_R_A);
    unsigned long startT = millis();
    while (ticks < TURN_TICKS && (millis() - startT < 1500)) {
      uint8_t la = digitalRead(PIN_ENC_L_A);
      uint8_t ra = digitalRead(PIN_ENC_R_A);
      if(la != lastLA) ticks++; lastLA = la;
      if(ra != lastRA) ticks++; lastRA = ra;
      delay(1);
    }
    dur(); delay(200);

    // Check if Right is ALSO blocked. If so, we MUST turn Left.
    tsag(); 
    if (g_wall_right == 1) {
       sol(); // Turn Left
       // Re-run encoder loop for left turn...
       uint16_t t2 = 0; unsigned long s2 = millis();
       while(t2 < TURN_TICKS && (millis()-s2 < 1500)){
          uint8_t la = digitalRead(PIN_ENC_L_A); uint8_t ra = digitalRead(PIN_ENC_R_A);
          if(la != lastLA) t2++; lastLA = la; if(ra != lastRA) t2++; lastRA = ra;
          delay(1);
       }
       dur(); delay(200);
    }
  }

  // 3. Execution: If Front is clear, drive one cell forward
  if (g_wall_front == 0) {
    ileri();
    uint16_t ticks = 0;
    uint8_t lastLA = digitalRead(PIN_ENC_L_A);
    uint8_t lastRA = digitalRead(PIN_ENC_R_A);
    unsigned long startT = millis();
    while (ticks < CELL_TICKS && (millis() - startT < 2000)) {
      // Center the robot using side sensors while driving
      int err = (int)g_right_cm - (int)g_left_cm;
      err = constrain(err, -10, 10);
      analogWrite(L_PWM, 255 + err); 
      analogWrite(R_PWM, 255 - err);

      uint8_t la = digitalRead(PIN_ENC_L_A);
      uint8_t ra = digitalRead(PIN_ENC_R_A);
      if(la != lastLA) ticks++; lastLA = la;
      if(ra != lastRA) ticks++; lastRA = ra;
      delay(1);
    }
    dur(); delay(200);
  }
}

void getmpu(){ 
  // Minimal MPU read to save flash and prevent I2C hanging
  if (Wire.endTransmission(false) == 0) {
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); // Start at Accel X
     Wire.endTransmission(false);
     Wire.requestFrom(0x68, 2);
     if(Wire.available() >= 2) {
        int16_t ax = (Wire.read() << 8) | Wire.read();
        // Using Accel as a fallback for orientation if Gyro is too heavy
        g_yaw_deg = (float)ax / 1000.0f; 
     }
  }
}

void gethcsrf(){ 
  digitalWrite(HFT, HIGH); delayMicroseconds(10); digitalWrite(HFT, LOW);
  uint32_t us = pulseIn(H_F_E, HIGH, 25000);
  g_front_cm = (us == 0) ? 400 : (us / 58);
}

void gethcsrl(){ 
  digitalWrite(HLT, HIGH); delayMicroseconds(10); digitalWrite(HLT, LOW);
  uint32_t us = pulseIn(H_L_E, HIGH, 25000);
  g_left_cm = (us == 0) ? 400 : (us / 58);
}

void gethcsrr(){ 
  digitalWrite(HRT, HIGH); delayMicroseconds(10); digitalWrite(HRT, LOW);
  uint32_t us = pulseIn(H_R_E, HIGH, 25000);
  g_right_cm = (us == 0) ? 400 : (us / 58);
}