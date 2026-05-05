 #include <Arduino.h>
 #include <Wire.h>
 
 const int PIN_START_BTN = PB5;
 const int PIN_QTR1A = PA0;
 
 const int HFT = PA1;
 const int HLT = PA2;
 const int HRT = PA3;
 
 const int H_F_E = PA4;
 const int H_L_E = PA5;
 const int H_R_E = PA6;
 
 const int R_PWM = PB0;
 const int L_PWM = PB1;
 const int R_IN1 = PB12;
 const int R_IN2 = PB13;
 const int L_IN1 = PB14;
 const int L_IN2 = PB15;
 
 const int PIN_LED = PC13;
 
 static constexpr bool BYPASS_QTR1A = true;
 static constexpr uint16_t QTR_WHITE_THRESHOLD = 2500;
 
 static constexpr uint16_t WALL_CM = 10;
 static constexpr uint16_t SIDE_CM = 18;

 enum AlgoMode : uint8_t {
   ALG_RIGHT_HAND = 0,
   ALG_LEFT_HAND = 1,
   ALG_TREMAUX = 2,
 };
 
 static constexpr AlgoMode ALGO_MODE = ALG_TREMAUX;
 
 static constexpr bool INVERT_L = false;
 static constexpr bool INVERT_R = false;
 
 static constexpr uint8_t PWM_RUN = 200;
 static constexpr uint8_t PWM_TURN = 170;
 
 static constexpr uint16_t PING_TIMEOUT_US = 25000;
 static constexpr uint32_t PING_MIN_INTERVAL_MS = 70;
 
 static uint32_t g_last_ping_ms_f = 0;
 static uint32_t g_last_ping_ms_l = 0;
 static uint32_t g_last_ping_ms_r = 0;
 
 static uint16_t g_cm_f = 0;
 static uint16_t g_cm_l = 0;
 static uint16_t g_cm_r = 0;
 
 enum RunState : uint8_t {
   ST_WAIT = 0,
   ST_MOTOR_TEST = 1,
   ST_RUN = 2,
   ST_STOP = 3,
 };
 
 static RunState g_state = ST_WAIT;
 
 static char g_path[256];
 static uint8_t g_path_len = 0;
 
 void ledOn() { digitalWrite(PIN_LED, LOW); }
 void ledOff() { digitalWrite(PIN_LED, HIGH); }
 
 void ledBlink(uint8_t n) {
   for (uint8_t i = 0; i < n; i++) {
     ledOn();
     delay(120);
     ledOff();
     delay(120);
   }
   delay(250);
 }
 
 bool qtrIsWhite() {
   if (BYPASS_QTR1A) return false;
   return analogRead(PIN_QTR1A) > QTR_WHITE_THRESHOLD;
 }
 
 void motorSet(int16_t left, int16_t right) {
   left = constrain(left, -255, 255);
   right = constrain(right, -255, 255);
 
   if (INVERT_L) left = -left;
   if (INVERT_R) right = -right;
 
   if (left >= 0) {
     digitalWrite(L_IN1, HIGH);
     digitalWrite(L_IN2, LOW);
   } else {
     digitalWrite(L_IN1, LOW);
     digitalWrite(L_IN2, HIGH);
   }
 
   if (right >= 0) {
     digitalWrite(R_IN1, HIGH);
     digitalWrite(R_IN2, LOW);
   } else {
     digitalWrite(R_IN1, LOW);
     digitalWrite(R_IN2, HIGH);
   }
 
   analogWrite(L_PWM, (uint8_t)abs(left));
   analogWrite(R_PWM, (uint8_t)abs(right));
 }
 
 void dur() {
   digitalWrite(L_IN1, LOW);
   digitalWrite(L_IN2, LOW);
   digitalWrite(R_IN1, LOW);
   digitalWrite(R_IN2, LOW);
   analogWrite(L_PWM, 0);
   analogWrite(R_PWM, 0);
 }
 
 void ileri() { motorSet(PWM_RUN, PWM_RUN); }
 void geri() { motorSet(-PWM_RUN, -PWM_RUN); }
 void sag360() { motorSet(PWM_TURN, -PWM_TURN); }
 void sol360() { motorSet(-PWM_TURN, PWM_TURN); }
 void sag() { motorSet(PWM_TURN, 0); }
 void sol() { motorSet(0, PWM_TURN); }
 
 uint16_t pingCM(int trig, int echo) {
   digitalWrite(trig, LOW);
   delayMicroseconds(2);
   digitalWrite(trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(trig, LOW);
 
   uint32_t us = pulseIn(echo, HIGH, PING_TIMEOUT_US);
   if (!us) return 0;
   uint16_t cm = (uint16_t)(us / 58u);
   if (cm > 400) cm = 400;
   return cm;
 }
 
 void readUltrasonics() {
   uint32_t now = millis();
 
   if (now - g_last_ping_ms_f >= PING_MIN_INTERVAL_MS) {
     g_last_ping_ms_f = now;
     g_cm_f = pingCM(HFT, H_F_E);
   }
 
   if (now - g_last_ping_ms_l >= PING_MIN_INTERVAL_MS) {
     g_last_ping_ms_l = now;
     g_cm_l = pingCM(HLT, H_L_E);
   }
 
   if (now - g_last_ping_ms_r >= PING_MIN_INTERVAL_MS) {
     g_last_ping_ms_r = now;
     g_cm_r = pingCM(HRT, H_R_E);
   }
 }
 
 bool startTriggered() {
   static bool inited = false;
   static int base = 0;
   static uint32_t stableSince = 0;
 
   int v = digitalRead(PIN_START_BTN);
 
   if (!inited) {
     inited = true;
     base = v;
     stableSince = millis();
     return false;
   }
 
   if (v == base) {
     stableSince = millis();
     return false;
   }
 
   if (millis() - stableSince >= 30) return true;
   return false;
 }
 
 void motorTestSequence() {
   ledBlink(1);
 
   ileri();
   delay(600);
   dur();
   delay(250);
 
   geri();
   delay(600);
   dur();
   delay(250);
 
   sag360();
   delay(500);
   dur();
   delay(250);
 
   sol360();
   delay(500);
   dur();
   delay(250);
 }
 
 void stepForwardTimed() {
   ileri();
   delay(260);
   dur();
   delay(60);
 }
 
 void turnRightTimed() {
   sag360();
   delay(290);
   dur();
   delay(80);
 }
 
 void turnLeftTimed() {
   sol360();
   delay(290);
   dur();
   delay(80);
 }
 
 void turnAroundTimed() {
   sag360();
   delay(580);
   dur();
   delay(100);
 }
 
 int8_t moveAngle(char m) {
   if (m == 'L') return -90;
   if (m == 'R') return 90;
   if (m == 'B') return 180;
   return 0;
 }
 
 char angleMove(int a) {
   a %= 360;
   if (a < 0) a += 360;
   if (a == 0) return 'S';
   if (a == 90) return 'R';
   if (a == 180) return 'B';
   return 'L';
 }
 
 void simplifyPath() {
   while (g_path_len >= 3 && g_path[g_path_len - 2] == 'B') {
     char a = g_path[g_path_len - 3];
     char c = g_path[g_path_len - 1];
     int total = moveAngle(a) + 180 + moveAngle(c);
     char rep = angleMove(total);
     g_path_len -= 3;
     g_path[g_path_len++] = rep;
   }
 }
 
 void recordMove(char m) {
   if (g_path_len < sizeof(g_path)) {
     g_path[g_path_len++] = m;
     simplifyPath();
   }
 }
 
 void executeMove(char m) {
   if (m == 'R') {
     turnRightTimed();
     stepForwardTimed();
     return;
   }
   if (m == 'L') {
     turnLeftTimed();
     stepForwardTimed();
     return;
   }
   if (m == 'B') {
     turnAroundTimed();
     stepForwardTimed();
     return;
   }
   stepForwardTimed();
 }
 
 char chooseMoveRightFirst(bool wallF, bool wallL, bool wallR) {
   if (!wallR) return 'R';
   if (!wallF) return 'S';
   if (!wallL) return 'L';
   return 'B';
 }
 
 char chooseMoveLeftFirst(bool wallF, bool wallL, bool wallR) {
   if (!wallL) return 'L';
   if (!wallF) return 'S';
   if (!wallR) return 'R';
   return 'B';
 }
 
 void runRightHandRule() {
   readUltrasonics();
 
   bool wallF = (g_cm_f > 0 && g_cm_f < WALL_CM);
   bool wallR = (g_cm_r > 0 && g_cm_r < SIDE_CM);
   bool wallL = (g_cm_l > 0 && g_cm_l < SIDE_CM);
 
   char m = chooseMoveRightFirst(wallF, wallL, wallR);
   executeMove(m);
 }
 
 void runLeftHandRule() {
   readUltrasonics();
 
   bool wallF = (g_cm_f > 0 && g_cm_f < WALL_CM);
   bool wallR = (g_cm_r > 0 && g_cm_r < SIDE_CM);
   bool wallL = (g_cm_l > 0 && g_cm_l < SIDE_CM);
 
   char m = chooseMoveLeftFirst(wallF, wallL, wallR);
   executeMove(m);
 }
 
 void runTremaux() {
   readUltrasonics();
 
   bool wallF = (g_cm_f > 0 && g_cm_f < WALL_CM);
   bool wallR = (g_cm_r > 0 && g_cm_r < SIDE_CM);
   bool wallL = (g_cm_l > 0 && g_cm_l < SIDE_CM);
 
   char m = chooseMoveRightFirst(wallF, wallL, wallR);
   recordMove(m);
   executeMove(m);
 }
 
 void runSelectedAlgorithm() {
   if (ALGO_MODE == ALG_LEFT_HAND) {
     runLeftHandRule();
     return;
   }
   if (ALGO_MODE == ALG_TREMAUX) {
     runTremaux();
     return;
   }
   runRightHandRule();
 }
 
 void setup() {
   pinMode(PIN_LED, OUTPUT);
   ledOff();
 
   pinMode(PIN_START_BTN, INPUT);
   pinMode(PIN_QTR1A, INPUT);
 
   pinMode(HFT, OUTPUT);
   pinMode(HLT, OUTPUT);
   pinMode(HRT, OUTPUT);
   digitalWrite(HFT, LOW);
   digitalWrite(HLT, LOW);
   digitalWrite(HRT, LOW);
 
   pinMode(H_F_E, INPUT);
   pinMode(H_L_E, INPUT);
   pinMode(H_R_E, INPUT);
 
   pinMode(L_IN1, OUTPUT);
   pinMode(L_IN2, OUTPUT);
   pinMode(R_IN1, OUTPUT);
   pinMode(R_IN2, OUTPUT);
   pinMode(L_PWM, OUTPUT);
   pinMode(R_PWM, OUTPUT);
 
   dur();
 
   Wire.begin();
 
   g_state = ST_WAIT;
   ledBlink(2);
   if (ALGO_MODE == ALG_RIGHT_HAND) ledBlink(1);
   else if (ALGO_MODE == ALG_LEFT_HAND) ledBlink(2);
   else ledBlink(3);
 }
 
 void loop() {
   if (g_state == ST_WAIT) {
     dur();
     if (startTriggered()) {
       g_state = ST_MOTOR_TEST;
     }
     delay(5);
     return;
   }
 
   if (g_state == ST_MOTOR_TEST) {
     motorTestSequence();
     g_state = ST_RUN;
     g_path_len = 0;
     return;
   }
 
   if (g_state == ST_RUN) {
     if (qtrIsWhite()) {
       g_state = ST_STOP;
       return;
     }
     runSelectedAlgorithm();
     return;
   }
 
   if (g_state == ST_STOP) {
     dur();
     ledBlink(4);
     delay(1000);
     return;
   }
 }
