 
 static const int PIN_START_BTN = 7;
 
 static const int PIN_L_IN1 = 18;
 static const int PIN_L_IN2 = 19;
 static const int PIN_L_PWM = 20;
 
 static const int PIN_R_IN1 = 15;
 static const int PIN_R_IN2 = 16;
 static const int PIN_R_PWM = 17;
 
 static const int PIN_ENC_L_A = -1;
 static const int PIN_ENC_R_A = -1;
 
 static const int PWM_MAX = 255;
 
 static volatile long g_encL = 0;
 static volatile long g_encR = 0;
 
 static void isrEncL() { g_encL++; }
 static void isrEncR() { g_encR++; }
 
 static void motorStop() {
   digitalWrite(PIN_L_IN1, LOW);
   digitalWrite(PIN_L_IN2, LOW);
   digitalWrite(PIN_R_IN1, LOW);
   digitalWrite(PIN_R_IN2, LOW);
   analogWrite(PIN_L_PWM, 0);
   analogWrite(PIN_R_PWM, 0);
 }
 
 static void motorSet(int left, int right) {
   left = constrain(left, -PWM_MAX, PWM_MAX);
   right = constrain(right, -PWM_MAX, PWM_MAX);
 
   if (left == 0) {
     digitalWrite(PIN_L_IN1, LOW);
     digitalWrite(PIN_L_IN2, LOW);
     analogWrite(PIN_L_PWM, 0);
   } else if (left > 0) {
     digitalWrite(PIN_L_IN1, HIGH);
     digitalWrite(PIN_L_IN2, LOW);
     analogWrite(PIN_L_PWM, left);
   } else {
     digitalWrite(PIN_L_IN1, LOW);
     digitalWrite(PIN_L_IN2, HIGH);
     analogWrite(PIN_L_PWM, -left);
   }
 
   if (right == 0) {
     digitalWrite(PIN_R_IN1, LOW);
     digitalWrite(PIN_R_IN2, LOW);
     analogWrite(PIN_R_PWM, 0);
   } else if (right > 0) {
     digitalWrite(PIN_R_IN1, HIGH);
     digitalWrite(PIN_R_IN2, LOW);
     analogWrite(PIN_R_PWM, right);
   } else {
     digitalWrite(PIN_R_IN1, LOW);
     digitalWrite(PIN_R_IN2, HIGH);
     analogWrite(PIN_R_PWM, -right);
   }
 }
 
 static bool encodersEnabled() { return (PIN_ENC_L_A >= 0) && (PIN_ENC_R_A >= 0); }
 
 static void resetEncoders() {
   noInterrupts();
   g_encL = 0;
   g_encR = 0;
   interrupts();
 }
 
 static long encL() {
   noInterrupts();
   long v = g_encL;
   interrupts();
   return v;
 }
 
 static long encR() {
   noInterrupts();
   long v = g_encR;
   interrupts();
   return v;
 }
 
 static void driveForMs(int leftPwm, int rightPwm, unsigned long ms) {
   motorSet(leftPwm, rightPwm);
   unsigned long t0 = millis();
   while ((millis() - t0) < ms) {
     delay(1);
   }
   motorStop();
 }
 
 static void driveTicks(long targetTicks, int basePwm) {
   if (!encodersEnabled()) {
     driveForMs(basePwm, basePwm, 700);
     return;
   }
 
   targetTicks = labs(targetTicks);
   int dir = (basePwm >= 0) ? 1 : -1;
   basePwm = abs(basePwm);
   basePwm = constrain(basePwm, 0, PWM_MAX);
 
   resetEncoders();
 
   while (true) {
     long l = encL();
     long r = encR();
     if (l >= targetTicks && r >= targetTicks) break;
 
     long err = l - r;
     int k = 2;
     int corr = constrain((int)(err * k), -60, 60);
 
     int left = basePwm - corr;
     int right = basePwm + corr;
 
     left = constrain(left, 0, PWM_MAX);
     right = constrain(right, 0, PWM_MAX);
 
     motorSet(dir * left, dir * right);
     delay(2);
   }
 
   motorStop();
 }
 
 static void pivotRightMs(unsigned long ms, int pwm) { driveForMs(pwm, -pwm, ms); }
 static void pivotLeftMs(unsigned long ms, int pwm) { driveForMs(-pwm, pwm, ms); }
 
 void setup() {
   Serial.begin(115200);
 
   pinMode(PIN_START_BTN, INPUT_PULLUP);
 
   pinMode(PIN_L_IN1, OUTPUT);
   pinMode(PIN_L_IN2, OUTPUT);
   pinMode(PIN_L_PWM, OUTPUT);
 
   pinMode(PIN_R_IN1, OUTPUT);
   pinMode(PIN_R_IN2, OUTPUT);
   pinMode(PIN_R_PWM, OUTPUT);
 
   motorStop();
 
   if (encodersEnabled()) {
     pinMode(PIN_ENC_L_A, INPUT_PULLUP);
     pinMode(PIN_ENC_R_A, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, RISING);
     attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, RISING);
   }
 
   while (digitalRead(PIN_START_BTN) == HIGH) {
     delay(5);
   }
   delay(300);
 }
 
 void loop() {
   driveTicks(500, 170);
   delay(400);
   driveTicks(500, -170);
   delay(400);
 
   pivotRightMs(350, 170);
   delay(400);
   pivotLeftMs(350, 170);
   delay(800);
 
   Serial.print("encL=");
   Serial.print(encL());
   Serial.print(" encR=");
   Serial.println(encR());
 
   while (true) {
     motorStop();
     delay(50);
   }
 }
