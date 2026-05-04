#include <Arduino.h>
#include <Wire.h>

static const uint32_t SERIAL_BAUD = 115200;
static const int PIN_START_BTN = PB5;
static const bool REQUIRE_START_BUTTON = false;

static const uint8_t MPU_ADDR = 0x68;

static bool isStartActive() { return digitalRead(PIN_START_BTN) == LOW; }

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t* out, int n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)addr, n);
  if (got != n) return false;
  for (int i = 0; i < n; i++) out[i] = (uint8_t)Wire.read();
  return true;
}

static bool mpuInit() {
  if (!i2cWrite8(MPU_ADDR, 0x6B, 0x00)) return false;
  if (!i2cWrite8(MPU_ADDR, 0x1B, 0x00)) return false;
  if (!i2cWrite8(MPU_ADDR, 0x1C, 0x00)) return false;
  return true;
}

static bool readWhoAmI(uint8_t& v) {
  uint8_t b = 0;
  if (!i2cReadN(MPU_ADDR, 0x75, &b, 1)) return false;
  v = b;
  return true;
}

static bool readAccelGyro(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t b[14];
  if (!i2cReadN(MPU_ADDR, 0x3B, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

static float gyroBiasZ = 0.0f;
static float yawDeg = 0.0f;
static unsigned long lastUs = 0;

static void calibrateGyroZ() {
  float sum = 0.0f;
  const int n = 800;
  for (int i = 0; i < n; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readAccelGyro(ax, ay, az, gx, gy, gz)) sum += (float)gz;
    delay(2);
  }
  gyroBiasZ = sum / (float)n;
}

static void updateYaw() {
  unsigned long now = micros();
  unsigned long dtUs = now - lastUs;
  lastUs = now;
  if (dtUs == 0) return;
  int16_t ax, ay, az, gx, gy, gz;
  if (!readAccelGyro(ax, ay, az, gx, gy, gz)) return;
  float gzDps = ((float)gz - gyroBiasZ) / 131.0f;
  yawDeg += gzDps * ((float)dtUs / 1000000.0f);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("MPU6050_TEST");
  pinMode(PIN_START_BTN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  if (!mpuInit()) {
    Serial.println("MPU_INIT_FAIL");
    while (true) delay(1000);
  }

  uint8_t who = 0;
  if (!readWhoAmI(who)) {
    Serial.println("WHOAMI_FAIL");
    while (true) delay(1000);
  }
  Serial.print("WHO_AM_I=0x");
  Serial.println(who, HEX);

  Serial.println("CAL_GYRO_Z");
  calibrateGyroZ();
  Serial.print("BIAS_GZ=");
  Serial.println(gyroBiasZ, 2);

  yawDeg = 0.0f;
  lastUs = micros();
  Serial.println("RUN");
}

void loop() {
  if (REQUIRE_START_BUTTON && !isStartActive()) {
    delay(10);
    return;
  }

  updateYaw();
  int16_t ax, ay, az, gx, gy, gz;
  if (!readAccelGyro(ax, ay, az, gx, gy, gz)) {
    Serial.println("READ_FAIL");
    delay(200);
    return;
  }

  Serial.print("ax=");
  Serial.print(ax);
  Serial.print(" ay=");
  Serial.print(ay);
  Serial.print(" az=");
  Serial.print(az);
  Serial.print(" gx=");
  Serial.print(gx);
  Serial.print(" gy=");
  Serial.print(gy);
  Serial.print(" gz=");
  Serial.print(gz);
  Serial.print(" yawDeg=");
  Serial.println(yawDeg, 2);

  delay(80);
}
