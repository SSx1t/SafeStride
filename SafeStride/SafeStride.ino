/*
  MYOSA Continuous Sensor Monitor

  Streams live accelerometer + altitude readings to Serial Monitor at ~10 Hz.
*/

#include <Wire.h>
#include <math.h>

#define MPU_ADDR_PRIMARY  0x68
#define MPU_ADDR_ALT      0x69
#define BMP_ADDR          0x77

uint8_t mpu_addr = 0;
bool bmp_ok = false;

// BMP180 calibration coefficients
int16_t  AC1, AC2, AC3, BMP_B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
const uint8_t OSS = 0;

// BMP180 helpers
uint8_t bmp_read8(uint8_t reg) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDR, 1);
  return Wire.read();
}

uint16_t bmp_read16(uint8_t reg) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDR, 2);
  uint16_t v = Wire.read() << 8;
  v |= Wire.read();
  return v;
}

void bmp_write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool bmp_init() {
  if (bmp_read8(0xD0) != 0x55) return false;

  AC1 = bmp_read16(0xAA);
  AC2 = bmp_read16(0xAC);
  AC3 = bmp_read16(0xAE);
  AC4 = bmp_read16(0xB0);
  AC5 = bmp_read16(0xB2);
  AC6 = bmp_read16(0xB4);
  BMP_B1 = bmp_read16(0xB6);
  B2  = bmp_read16(0xB8);
  MB  = bmp_read16(0xBA);
  MC  = bmp_read16(0xBC);
  MD  = bmp_read16(0xBE);

  return true;
}

long bmp_read_raw_temp() {
  bmp_write8(0xF4, 0x2E);
  delay(5);
  return (long)bmp_read16(0xF6);
}

long bmp_read_raw_pressure() {
  bmp_write8(0xF4, 0x34 + (OSS << 6));
  delay(5);
  return (long)bmp_read16(0xF6);
}

float bmp_altitude() {
  long UT = bmp_read_raw_temp();
  long UP = bmp_read_raw_pressure();

  long X1 = ((UT - (long)AC6) * (long)AC5) >> 15;
  long X2 = ((long)MC << 11) / (X1 + MD);
  long B5 = X1 + X2;

  long B6 = B5 - 4000;
  X1 = ((long)B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((long)AC2 * B6) >> 11;
  long X3 = X1 + X2;
  long B3 = ((((long)AC1 * 4 + X3) << OSS) + 2) >> 2;

  X1 = ((long)AC3 * B6) >> 13;
  X2 = ((long)BMP_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;

  unsigned long B4 = ((unsigned long)AC4 * (unsigned long)(X3 + 32768)) >> 15;
  unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> OSS);

  long p;
  if (B7 < 0x80000000) p = (B7 * 2) / B4;
  else                 p = (B7 / B4) * 2;

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);

  return 44330.0f * (1.0f - pow(p / 101325.0f, 1.0f / 5.255f));
}

// MPU6050 helpers
bool mpu_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(addr, (uint8_t)1);
  if (!Wire.available()) return false;

  uint8_t who = Wire.read();
  return (who == 0x68 || who == 0x70 || who == 0x71);
}

void mpu_wake(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool mpu_read_accel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom(mpu_addr, (uint8_t)6);
  if (n != 6) return false;

  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();

  ax = rx / 16384.0f * 1000.0f;
  ay = ry / 16384.0f * 1000.0f;
  az = rz / 16384.0f * 1000.0f;

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();
  Wire.setClock(100000);
  delay(500);

  Serial.println();
  Serial.println("=== MYOSA Continuous Sensor Monitor ===");
  Serial.println();

  if (mpu_probe(MPU_ADDR_PRIMARY)) {
    mpu_addr = MPU_ADDR_PRIMARY;
    Serial.println("MPU6050 found at 0x68");
  } else if (mpu_probe(MPU_ADDR_ALT)) {
    mpu_addr = MPU_ADDR_ALT;
    Serial.println("MPU6050 found at 0x69");
  } else {
    Serial.println("MPU6050 NOT FOUND on bus");
  }

  if (mpu_addr) {
    mpu_wake(mpu_addr);
    delay(100);
  }

  bmp_ok = bmp_init();
  Serial.println(bmp_ok ? "BMP180 found at 0x77" : "BMP180 NOT FOUND");

  Serial.println();
  Serial.println("Streaming live data:");
  Serial.println();
  Serial.println("t_ms\tax(mg)\tay(mg)\taz(mg)\t|a|(mg)\talt(m)\t\tstatus");
  Serial.println("-----\t------\t------\t------\t-------\t------\t\t------");
}

void loop() {
  static unsigned long lastSample = 0;
  static unsigned long lastAlt = 0;
  static float cachedAlt = 0;

  unsigned long now = millis();

  if (now - lastSample < 100) return;
  lastSample = now;

  float ax = 0, ay = 0, az = 0;
  bool accel_ok = false;

  if (mpu_addr) {
    accel_ok = mpu_read_accel(ax, ay, az);
  }

  if (bmp_ok && (now - lastAlt > 500)) {
    lastAlt = now;
    cachedAlt = bmp_altitude();
  }

  float mag = sqrt(ax * ax + ay * ay + az * az);

  Serial.print(now);       Serial.print("\t");
  Serial.print(ax, 1);     Serial.print("\t");
  Serial.print(ay, 1);     Serial.print("\t");
  Serial.print(az, 1);     Serial.print("\t");
  Serial.print(mag, 1);    Serial.print("\t");

  if (bmp_ok) Serial.print(cachedAlt, 2);
  else        Serial.print("---");

  Serial.print("\t\t");

  if (!mpu_addr)       Serial.print("[ACCEL DEAD] ");
  else if (!accel_ok)  Serial.print("[ACCEL READ FAIL] ");
  else if (mag < 100)  Serial.print("[ACCEL STUCK ZERO] ");
  else if (mag > 800 && mag < 1200) Serial.print("[OK] ");
  else                 Serial.print("[motion/odd] ");

  if (!bmp_ok) Serial.print("[ALT DEAD]");

  Serial.println();
}