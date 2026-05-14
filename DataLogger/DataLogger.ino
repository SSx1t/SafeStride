// #include <Wire.h>
// #include <NimBLEDevice.h>

// #include "oled.h"
// #include "BarometricPressure.h"

// #define SERVICE_UUID    "4f5b1a00-7b3a-4f9b-9e80-1a1e5b7c2d40"
// #define CMD_CHAR_UUID   "4f5b1a01-7b3a-4f9b-9e80-1a1e5b7c2d40"
// #define LOG_CHAR_UUID   "4f5b1a04-7b3a-4f9b-9e80-1a1e5b7c2d40"

// #define MPU_ADDR_PRIMARY 0x68
// #define MPU_ADDR_ALT     0x69

// uint8_t mpuAddr = 0;

// oLed oled(128, 64);
// BarometricPressure bmp(ULTRA_LOW_POWER);

// NimBLECharacteristic* logChar = nullptr;
// NimBLECharacteristic* cmdChar = nullptr;

// bool deviceConnected = false;
// bool recording = false;

// unsigned long recordStart = 0;

// // Bias values
// float axBias = 0, ayBias = 0, azBias = 0;
// float gxBias = 0, gyBias = 0, gzBias = 0;

// // ───────── OLED ─────────
// void showWaiting() {
//   oled.clearDisplay();
//   oled.setTextSize(2);
//   oled.setCursor(0, 16);
//   oled.print("WAITING");
//   oled.setTextSize(1);
//   oled.setCursor(0, 48);
//   oled.print("BLE connect");
//   oled.display();
// }

// void showReady() {
//   oled.clearDisplay();
//   oled.setTextSize(2);
//   oled.setCursor(0, 16);
//   oled.print("READY");
//   oled.setTextSize(1);
//   oled.setCursor(0, 48);
//   oled.print("Enter on PC");
//   oled.display();
// }

// void showRecording() {
//   oled.clearDisplay();
//   oled.setTextSize(2);
//   oled.setCursor(0, 16);
//   oled.print("REC");
//   oled.setTextSize(1);
//   oled.setCursor(0, 48);
//   oled.print("Enter stop");
//   oled.display();
// }

// // ───────── MPU ─────────
// bool probeMPU(uint8_t addr) {
//   Wire.beginTransmission(addr);
//   Wire.write(0x75);
//   if (Wire.endTransmission(false) != 0) return false;

//   Wire.requestFrom(addr, (uint8_t)1);
//   if (!Wire.available()) return false;

//   uint8_t who = Wire.read();
//   return who == 0x68 || who == 0x70 || who == 0x71;
// }

// void wakeMPU(uint8_t addr) {
//   Wire.beginTransmission(addr);
//   Wire.write(0x6B);
//   Wire.write(0x00);
//   Wire.endTransmission();
//   delay(100);

//   Wire.beginTransmission(addr);
//   Wire.write(0x1C);
//   Wire.write(0x00);
//   Wire.endTransmission();

//   Wire.beginTransmission(addr);
//   Wire.write(0x1B);
//   Wire.write(0x00);
//   Wire.endTransmission();

//   Wire.beginTransmission(addr);
//   Wire.write(0x1A);
//   Wire.write(0x03);
//   Wire.endTransmission();
// }

// bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
//   if (mpuAddr == 0) return false;

//   Wire.beginTransmission(mpuAddr);
//   Wire.write(0x3B);
//   if (Wire.endTransmission(false) != 0) return false;

//   if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;

//   int16_t rawAx = (Wire.read() << 8) | Wire.read();
//   int16_t rawAy = (Wire.read() << 8) | Wire.read();
//   int16_t rawAz = (Wire.read() << 8) | Wire.read();

//   Wire.read(); Wire.read();

//   int16_t rawGx = (Wire.read() << 8) | Wire.read();
//   int16_t rawGy = (Wire.read() << 8) | Wire.read();
//   int16_t rawGz = (Wire.read() << 8) | Wire.read();

//   ax = rawAx / 16384.0f * 1000.0f;
//   ay = rawAy / 16384.0f * 1000.0f;
//   az = rawAz / 16384.0f * 1000.0f;

//   gx = rawGx / 131.0f;
//   gy = rawGy / 131.0f;
//   gz = rawGz / 131.0f;

//   return true;
// }

// // ───────── CALIBRATION ─────────
// void calibrateMPU() {
//   Serial.println("# Calibrating... keep device still");

//   const int N = 300;
//   float sx = 0, sy = 0, sz = 0;
//   float sgx = 0, sgy = 0, sgz = 0;

//   for (int i = 0; i < N; i++) {
//     float ax, ay, az, gx, gy, gz;

//     if (readMPU(ax, ay, az, gx, gy, gz)) {
//       sx += ax;
//       sy += ay;
//       sz += az;
//       sgx += gx;
//       sgy += gy;
//       sgz += gz;
//     }
//     delay(10);
//   }

//   axBias = sx / N;
//   ayBias = sy / N;
//   azBias = sz / N;

//   gxBias = sgx / N;
//   gyBias = sgy / N;
//   gzBias = sgz / N;

//   // Keep gravity on Y axis
//   ayBias -= 1000.0f;

//   Serial.println("# Calibration done");
// }

// // ───────── BLE ─────────
// void startRecording() {
//   recordStart = millis();
//   recording = true;
//   Serial.println("# RECORDING STARTED");
//   showRecording();
// }

// void stopRecording() {
//   recording = false;
//   Serial.println("# RECORDING STOPPED");
//   showReady();
// }

// class ServerCB : public NimBLEServerCallbacks {
//   void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
//     deviceConnected = true;
//     showReady();
//   }

//   void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
//     deviceConnected = false;
//     recording = false;
//     showWaiting();
//     NimBLEDevice::startAdvertising();
//   }
// };

// class CmdCB : public NimBLECharacteristicCallbacks {
//   void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
//     std::string v = c->getValue();
//     if (v.length() == 0) return;

//     if (v[0] == 0x20) startRecording();
//     else if (v[0] == 0x21) stopRecording();
//   }
// };

// void setupBLE() {
//   NimBLEDevice::init("SafeStride-Logger");

//   NimBLEServer* s = NimBLEDevice::createServer();
//   s->setCallbacks(new ServerCB());

//   NimBLEService* svc = s->createService(SERVICE_UUID);

//   cmdChar = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
//   cmdChar->setCallbacks(new CmdCB());

//   logChar = svc->createCharacteristic(LOG_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

//   svc->start();

//   NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
//   adv->addServiceUUID(SERVICE_UUID);
//   adv->setName("SafeStride-Logger");
//   adv->start();
// }

// // ───────── SETUP ─────────
// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   Wire.begin();
//   delay(500);

//   oled.begin();
//   bmp.begin();

//   if (probeMPU(MPU_ADDR_PRIMARY)) mpuAddr = MPU_ADDR_PRIMARY;
//   else if (probeMPU(MPU_ADDR_ALT)) mpuAddr = MPU_ADDR_ALT;

//   if (mpuAddr) {
//     wakeMPU(mpuAddr);
//     calibrateMPU();
//   }

//   showWaiting();
//   setupBLE();
// }

// // ───────── LOOP ─────────
// void loop() {
//   static unsigned long lastSample = 0;
//   static float alt = 0;

//   unsigned long now = millis();

//   if (now - lastSample >= 10) {
//     lastSample = now;

//     float ax, ay, az, gx, gy, gz;
//     if (!readMPU(ax, ay, az, gx, gy, gz)) return;

//     // Apply bias correction
//     ax -= axBias;
//     ay -= ayBias;
//     az -= azBias;
//     gx -= gxBias;
//     gy -= gyBias;
//     gz -= gzBias;

//     alt = bmp.getAltitude(101325.0f, false);

//     if (recording) {
//       unsigned long t = millis() - recordStart;

//       String csv = String(t) + "," +
//                    String(ax,2) + "," +
//                    String(ay,2) + "," +
//                    String(az,2) + "," +
//                    String(gx,2) + "," +
//                    String(gy,2) + "," +
//                    String(gz,2) + "," +
//                    String(alt,2);

//       Serial.println(csv);

//       if (deviceConnected) {
//         logChar->setValue(csv);
//         logChar->notify();
//       }
//     }
//   }
// }

#include <Wire.h>
#include <NimBLEDevice.h>

#include "oled.h"
#include "BarometricPressure.h"

#define SERVICE_UUID    "4f5b1a00-7b3a-4f9b-9e80-1a1e5b7c2d40"
#define CMD_CHAR_UUID   "4f5b1a01-7b3a-4f9b-9e80-1a1e5b7c2d40"
#define LOG_CHAR_UUID   "4f5b1a04-7b3a-4f9b-9e80-1a1e5b7c2d40"

#define MPU_ADDR_PRIMARY 0x68
#define MPU_ADDR_ALT     0x69

uint8_t mpuAddr = 0;

oLed oled(128, 64);
BarometricPressure bmp(ULTRA_LOW_POWER);

NimBLECharacteristic* logChar = nullptr;
NimBLECharacteristic* cmdChar = nullptr;

bool deviceConnected = false;
bool recording = false;

unsigned long recordStart = 0;

// Bias values
float axBias = 0, ayBias = 0, azBias = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;

// Filtering / sampling state
float axFilt = 0, ayFilt = 0, azFilt = 0;
float gxFilt = 0, gyFilt = 0, gzFilt = 0;

// small ring buffers for median(3)
float axBuf[3] = {0,0,0}, ayBuf[3] = {0,0,0}, azBuf[3] = {0,0,0};
float gxBuf[3] = {0,0,0}, gyBuf[3] = {0,0,0}, gzBuf[3] = {0,0,0};
uint8_t bufIndex = 0;

// Timing (micros)
unsigned long lastMicros = 0;
const unsigned long sampleIntervalUs = 10000; // 100 Hz

// BLE batching
String bleBatch = "";
int bleCount = 0;
const int BLE_BATCH_SIZE = 10;

// Spike clamp thresholds
const float ACCEL_SPIKE_THRESHOLD = 0.5f; // g
const float GYRO_SPIKE_THRESHOLD = 200.0f; // deg/s

const float MY_PI = 3.14159265358979323846f;

// helpers
float median3(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

float applyIIR(float prev, float raw, float dt, float fc) {
  if (dt <= 0) return prev;
  float rc = 1.0f / (2.0f * MY_PI * fc);
  float alpha = dt / (rc + dt);
  return prev + alpha * (raw - prev);
}

float clampSpike(float prevFilt, float candidate, float thresh) {
  float delta = candidate - prevFilt;
  if (fabs(delta) > thresh) {
    return prevFilt + (delta > 0 ? thresh : -thresh);
  }
  return candidate;
}

// Baseline pressure for relative altitude
float baselinePressure = 101325.0f; 

// ───────── OLED ─────────
void showWaiting() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 16);
  oled.print("WAITING");
  oled.setTextSize(1);
  oled.setCursor(0, 48);
  oled.print("BLE connect");
  oled.display();
}

void showReady() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 16);
  oled.print("READY");
  oled.setTextSize(1);
  oled.setCursor(0, 48);
  oled.print("Enter on PC");
  oled.display();
}

void showRecording() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 16);
  oled.print("REC");
  oled.setTextSize(1);
  oled.setCursor(0, 48);
  oled.print("Enter stop");
  oled.display();
}

// ───────── MPU ─────────
bool probeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(addr, (uint8_t)1);
  if (!Wire.available()) return false;

  uint8_t who = Wire.read();
  return who == 0x68 || who == 0x70 || who == 0x71;
}

void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x10); // Gyroscope to ±1000 °/s
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1A);
  // Reduce sensor internal bandwidth to cut HF noise (DLPF_CFG=4 ≈ 21 Hz)
  Wire.write(0x04);
  Wire.endTransmission();
}

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (mpuAddr == 0) return false;

  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read();

  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // MD INTEGRATION FIX: App expects units in 'g', not 'mg'. Removed *1000.0f
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;

  gx = rawGx / 32.8f; 
  gy = rawGy / 32.8f;
  gz = rawGz / 32.8f;

  return true;
}

// ───────── CALIBRATION ─────────
void calibrateMPU() {
  Serial.println("# Calibrating... keep device still");

  const int N = 300;
  float sx = 0, sy = 0, sz = 0;
  float sgx = 0, sgy = 0, sgz = 0;

  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;

    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sx += ax;
      sy += ay;
      sz += az;
      sgx += gx;
      sgy += gy;
      sgz += gz;
    }
    delay(10);
  }

  axBias = sx / N;
  ayBias = sy / N;
  azBias = sz / N;

  gxBias = sgx / N;
  gyBias = sgy / N;
  gzBias = sgz / N;

  // MD INTEGRATION FIX: Gravity offset is now 1.0g since acceleration is in 'g'
  ayBias -= 1.0f;

  Serial.println("# Calibration done");
}

// ───────── BLE ─────────
void startRecording() {
  recordStart = millis();
  recording = true;
  Serial.println("# RECORDING STARTED");
  showRecording();
}

void stopRecording() {
  recording = false;
  Serial.println("# RECORDING STOPPED");
  showReady();
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    deviceConnected = true;
    showReady();
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    deviceConnected = false;
    recording = false;
    showWaiting();
    NimBLEDevice::startAdvertising();
  }
};

class CmdCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    if (v.length() == 0) return;

    if (v[0] == 0x20) startRecording();
    else if (v[0] == 0x21) stopRecording();
  }
};

void setupBLE() {
  NimBLEDevice::init("SafeStride-Logger");
  
  // MD INTEGRATION FIX: App requests MTU up to 247. Set this side accordingly.
  NimBLEDevice::setMTU(247);

  NimBLEServer* s = NimBLEDevice::createServer();
  s->setCallbacks(new ServerCB());

  NimBLEService* svc = s->createService(SERVICE_UUID);

  cmdChar = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  cmdChar->setCallbacks(new CmdCB());

  logChar = svc->createCharacteristic(LOG_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  
  // ---------------------------------------------------------
  // FIXED: Build explicit Advertisement and Scan Response Data
  // ---------------------------------------------------------
  
  // 1. Primary Advertisement (Just the name)
  NimBLEAdvertisementData advData;
  advData.setName("SafeStride-Logger");
  adv->setAdvertisementData(advData);

  // 2. Scan Response Data (Holds the 128-bit UUID)
  NimBLEAdvertisementData scanResp;
  scanResp.setCompleteServices(NimBLEUUID(SERVICE_UUID));
  adv->setScanResponseData(scanResp);

  adv->start();
}

// ───────── SETUP ─────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();
  delay(500);

  oled.begin();
  bmp.begin();

  baselinePressure = bmp.getPressure();

  if (probeMPU(MPU_ADDR_PRIMARY)) mpuAddr = MPU_ADDR_PRIMARY;
  else if (probeMPU(MPU_ADDR_ALT)) mpuAddr = MPU_ADDR_ALT;

  if (mpuAddr) {
    wakeMPU(mpuAddr);
    calibrateMPU();
  }

  showWaiting();
  setupBLE();
}

// ───────── LOOP ─────────
void loop() {
  unsigned long now = micros();
  static float alt = 0;

  if (lastMicros == 0) lastMicros = now;

  if (now - lastMicros >= sampleIntervalUs) {
    unsigned long elapsed = now - lastMicros;
    float dt = (float)elapsed / 1e6f;
    lastMicros = now;

    float ax, ay, az, gx, gy, gz;
    if (!readMPU(ax, ay, az, gx, gy, gz)) return;

    // Apply bias correction
    ax -= axBias; ay -= ayBias; az -= azBias;
    gx -= gxBias; gy -= gyBias; gz -= gzBias;

    // update median ring buffers
    axBuf[bufIndex] = ax; ayBuf[bufIndex] = ay; azBuf[bufIndex] = az;
    gxBuf[bufIndex] = gx; gyBuf[bufIndex] = gy; gzBuf[bufIndex] = gz;

    float medAx = median3(axBuf[0], axBuf[1], axBuf[2]);
    float medAy = median3(ayBuf[0], ayBuf[1], ayBuf[2]);
    float medAz = median3(azBuf[0], azBuf[1], azBuf[2]);

    float medGx = median3(gxBuf[0], gxBuf[1], gxBuf[2]);
    float medGy = median3(gyBuf[0], gyBuf[1], gyBuf[2]);
    float medGz = median3(gzBuf[0], gzBuf[1], gzBuf[2]);

    // advance buffer index
    bufIndex = (bufIndex + 1) % 3;

    // apply IIR filters
    axFilt = applyIIR(axFilt, medAx, dt, 15.0f);
    ayFilt = applyIIR(ayFilt, medAy, dt, 15.0f);
    azFilt = applyIIR(azFilt, medAz, dt, 15.0f);

    gxFilt = applyIIR(gxFilt, medGx, dt, 20.0f);
    gyFilt = applyIIR(gyFilt, medGy, dt, 20.0f);
    gzFilt = applyIIR(gzFilt, medGz, dt, 20.0f);

    // clamp large spikes
    axFilt = clampSpike(axFilt, axFilt, ACCEL_SPIKE_THRESHOLD);
    ayFilt = clampSpike(ayFilt, ayFilt, ACCEL_SPIKE_THRESHOLD);
    azFilt = clampSpike(azFilt, azFilt, ACCEL_SPIKE_THRESHOLD);

    gxFilt = clampSpike(gxFilt, gxFilt, GYRO_SPIKE_THRESHOLD);
    gyFilt = clampSpike(gyFilt, gyFilt, GYRO_SPIKE_THRESHOLD);
    gzFilt = clampSpike(gzFilt, gzFilt, GYRO_SPIKE_THRESHOLD);

    alt = bmp.getAltitude(baselinePressure, false);

    if (recording) {
      unsigned long t = millis() - recordStart;

      String csv = String(t) + "," +
                   String(axFilt,3) + "," +
                   String(ayFilt,3) + "," +
                   String(azFilt,3) + "," +
                   String(gxFilt,2) + "," +
                   String(gyFilt,2) + "," +
                   String(gzFilt,2) + "," +
                   String(alt,3);

      Serial.println(csv);

      // batch for BLE to reduce jitter and packet overhead
      bleBatch += csv + "\n";
      bleCount++;

      if (deviceConnected && bleCount >= BLE_BATCH_SIZE) {
        logChar->setValue(bleBatch.c_str());
        logChar->notify();
        bleBatch = "";
        bleCount = 0;
      }
    }
  }
}