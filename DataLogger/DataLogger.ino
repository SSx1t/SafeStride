/*
 * SafeStride DataLogger — ESP32 firmware (MYOSA platform)
 *
 * Streams 100 Hz IMU (MPU6050) + ~5 Hz barometric altitude (BMP180) over BLE
 * as ASCII CSV, one line per notification:
 *
 *   <tMs>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<altM>
 *
 *   accel — g     (2 decimals)
 *   gyro  — deg/s (2 decimals)
 *   alt   — m relative to baseline pressure captured at boot (3 decimals)
 *
 * Improvements over the original sketch:
 *  - BMP180 read moved off the IMU hot path. The BMP180 driver blocks for
 *    ~10 ms (two delay(5) calls inside it); running it every 200 ms drops
 *    1 sample / 20 instead of stuttering every sample.
 *  - MPU clock source switched from internal 8 MHz RC to gyro X PLL
 *    (CLKSEL = 1) — better stability and lower temperature drift.
 *  - SMPLRT_DIV = 9 → MPU chip output rate matched to our 100 Hz read rate
 *    so every read returns a freshly produced sample.
 *  - I²C bus bumped to 400 kHz fast-mode (all three MYOSA sensors support it).
 *  - Loop catch-up is bounded: a long blocking call (BMP, OLED, BLE stack)
 *    resets the cadence instead of bursting 100s of samples to "catch up".
 *  - Serial baud bumped to 921600 so 100 Hz CSV prints don't back-pressure
 *    the loop via a full TX FIFO.
 *  - Calibration auto-detects which axis is gravity-aligned, so the rest
 *    magnitude is ≈ 1.0 g regardless of how the belt is mounted.
 *
 * BLE wire format, UUIDs, device name, and start/stop bytes are unchanged
 * — the Android app speaks this firmware verbatim.
 */

#include <Wire.h>
#include <NimBLEDevice.h>
#include <math.h>

#include "oled.h"
#include "BarometricPressure.h"
#include "gait_model.h"

#define SERVICE_UUID    "4f5b1a00-7b3a-4f9b-9e80-1a1e5b7c2d40"
#define CMD_CHAR_UUID   "4f5b1a01-7b3a-4f9b-9e80-1a1e5b7c2d40"
#define LOG_CHAR_UUID   "4f5b1a04-7b3a-4f9b-9e80-1a1e5b7c2d40"
#define ML_CHAR_UUID    "4f5b1a05-7b3a-4f9b-9e80-1a1e5b7c2d40"

#define MPU_ADDR_PRIMARY 0x68
#define MPU_ADDR_ALT     0x69

static const uint32_t SAMPLE_PERIOD_MS     = 10;   // 100 Hz IMU
static const uint32_t BARO_PERIOD_MS       = 200;  // ~5 Hz altitude refresh
static const uint32_t CATCHUP_THRESHOLD_MS = 50;   // reset, don't burst, on long stalls
static const float    G_CONST              = 9.81f;
static const int      TARGET_HZ            = 100;
static const int      GAIT_WINDOW_SAMPLES   = 1000;  // 10 s at 100 Hz
static const int      GAIT_PREDICTION_STEP  = 50;    // 500 ms
static const bool     ML_DEBUG_FEATURES     = false;

// Master Serial-Monitor switch. False (default) = no Serial.begin, no UART
// writes, no snprintf done purely for printing — entire path is dead-code-
// eliminated by the compiler. Flip to true only when debugging over USB.
// In wireless/BLE-only use the phone is the sole consumer; running 100 Hz
// CSV through the UART steals cycles and adds jitter to BLE notifies.
static constexpr bool SERIAL_DEBUG          = false;

uint8_t mpuAddr = 0;

// Tell the OLED driver to leave the bus at 400 kHz after its writes
// (default clkAfter is 100 kHz, which would slow MPU/BMP reads too).
oLed oled(128, 64, &Wire, -1, 400000UL, 400000UL);
BarometricPressure bmp(ULTRA_LOW_POWER);

NimBLECharacteristic* logChar = nullptr;
NimBLECharacteristic* cmdChar = nullptr;
NimBLECharacteristic* mlChar  = nullptr;

bool deviceConnected = false;
bool recording = false;

unsigned long recordStart = 0;

float axBias = 0, ayBias = 0, azBias = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;

float baselinePressure = 101325.0f;
float cachedAlt = 0.0f;
// ---------- FILTERING / SMOOTHING ----------
static const bool STREAM_FILTERED_VALUES = false;
static const float ACC_ALPHA = 0.30f;
static const float GYR_ALPHA = 0.35f;
static const float ALT_ALPHA = 0.10f;

struct LowPass1 {
  bool ready = false;
  float y = 0.0f;
  float update(float x, float alpha) {
    if (!ready) { y = x; ready = true; return y; }
    y = y + alpha * (x - y);
    return y;
  }
};

LowPass1 axLP, ayLP, azLP;
LowPass1 gxLP, gyLP, gzLP;
LowPass1 altLP;

Eloquent::ML::Port::GaitClassifier gaitClassifier;

enum MlLabel : uint8_t {
  LABEL_NO_GAIT = 0,
  LABEL_HEALTHY_GAIT = 1,
  LABEL_STROKE_GAIT = 2
};

struct GaitSample {
  float vertDyn;
  float mlDyn;
  float yaw;
};

struct GaitFeatureWindow {
  float vert[GAIT_WINDOW_SAMPLES];
  float ml[GAIT_WINDOW_SAMPLES];
  float yaw[GAIT_WINDOW_SAMPLES];
};

static GaitSample gaitBuf[GAIT_WINDOW_SAMPLES];
static int gaitHead = 0;
static int gaitCount = 0;
static int samplesSincePrediction = 0;
static GaitFeatureWindow gaitWin;

class MlLabelSmoother {
public:
  static const int N = 7;

  MlLabelSmoother() { reset(); }

  void reset() {
    for (int i = 0; i < N; i++) hist[i] = LABEL_NO_GAIT;
    idx = 0;
    count = 0;
    stable = LABEL_NO_GAIT;
  }

  MlLabel update(MlLabel label) {
    hist[idx] = label;
    idx = (idx + 1) % N;
    if (count < N) count++;

    int noGait = 0, healthy = 0, stroke = 0;
    for (int i = 0; i < count; i++) {
      if (hist[i] == LABEL_NO_GAIT) noGait++;
      else if (hist[i] == LABEL_HEALTHY_GAIT) healthy++;
      else if (hist[i] == LABEL_STROKE_GAIT) stroke++;
    }

    MlLabel majority = LABEL_NO_GAIT;
    int best = noGait;
    if (healthy > best) { best = healthy; majority = LABEL_HEALTHY_GAIT; }
    if (stroke > best)  { best = stroke;  majority = LABEL_STROKE_GAIT; }

    stable = majority;
    return stable;
  }

  MlLabel get() const { return stable; }

private:
  MlLabel hist[N];
  int idx = 0;
  int count = 0;
  MlLabel stable = LABEL_NO_GAIT;
};

MlLabelSmoother mlSmoother;

// ───────── ML FEATURE / LABEL PIPELINE ─────────
void resetGaitPipeline() {
  gaitHead = 0;
  gaitCount = 0;
  samplesSincePrediction = 0;
  mlSmoother.reset();
}

void pushGaitSample(float vertDyn, float mlDyn, float yaw) {
  gaitBuf[gaitHead].vertDyn = vertDyn;
  gaitBuf[gaitHead].mlDyn = mlDyn;
  gaitBuf[gaitHead].yaw = yaw;
  gaitHead = (gaitHead + 1) % GAIT_WINDOW_SAMPLES;
  if (gaitCount < GAIT_WINDOW_SAMPLES) gaitCount++;
  samplesSincePrediction++;
}

void copyGaitWindow() {
  int start = (gaitCount < GAIT_WINDOW_SAMPLES) ? 0 : gaitHead;
  for (int i = 0; i < GAIT_WINDOW_SAMPLES; i++) {
    int idx = (start + i) % GAIT_WINDOW_SAMPLES;
    gaitWin.vert[i] = gaitBuf[idx].vertDyn;
    gaitWin.ml[i] = gaitBuf[idx].mlDyn;
    gaitWin.yaw[i] = gaitBuf[idx].yaw;
  }
}

float meanOf(const float* x, int n) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) sum += x[i];
  return (float)(sum / n);
}

float stdOf(const float* x, int n, float mean) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    double d = x[i] - mean;
    sum += d * d;
  }
  return (float)sqrt(sum / n);
}

float rangeOf(const float* x, int n) {
  float mn = x[0], mx = x[0];
  for (int i = 1; i < n; i++) {
    if (x[i] < mn) mn = x[i];
    if (x[i] > mx) mx = x[i];
  }
  return mx - mn;
}

float rmsOf(const float* x, int n) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) sum += (double)x[i] * (double)x[i];
  return (float)sqrt(sum / n);
}

float absMeanOf(const float* x, int n) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) sum += fabsf(x[i]);
  return (float)(sum / n);
}

float skewOf(const float* x, int n, float mean, float stdv) {
  if (stdv < 1e-6f) return 0.0f;
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    double z = (x[i] - mean) / stdv;
    sum += z * z * z;
  }
  return (float)(sum / n);
}

float kurtOf(const float* x, int n, float mean, float stdv) {
  if (stdv < 1e-6f) return 0.0f;
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    double z = (x[i] - mean) / stdv;
    sum += z * z * z * z;
  }
  return (float)((sum / n) - 3.0);
}

float autocorrAtLag(const float* x, int n, int lag) {
  if (lag >= n) return 0.0f;
  float mean = meanOf(x, n);
  double denom = 0.0, num = 0.0;
  for (int i = 0; i < n; i++) {
    double d = x[i] - mean;
    denom += d * d;
  }
  if (denom < 1e-6) return 0.0f;
  for (int i = 0; i < n - lag; i++) {
    num += (x[i] - mean) * (x[i + lag] - mean);
  }
  return (float)(num / denom);
}

int findStepPeaks(const float* x, int n, int* peakIdx, int maxPeaks) {
  const float threshold = 0.35f * G_CONST;
  const int minDistance = 25;
  int count = 0;
  int lastPeak = -minDistance;

  for (int i = 1; i < n - 1; i++) {
    bool localMax = (x[i] > x[i - 1]) && (x[i] >= x[i + 1]);
    bool aboveThr = x[i] > threshold;
    bool farEnough = (i - lastPeak) >= minDistance;
    if (localMax && aboveThr && farEnough) {
      if (count < maxPeaks) peakIdx[count] = i;
      count++;
      lastPeak = i;
    }
  }

  return (count > maxPeaks) ? maxPeaks : count;
}

bool extractGaitFeatures(const GaitFeatureWindow& win, float features[18]) {
  const float* vert = win.vert;
  const float* ml = win.ml;
  const float* yaw = win.yaw;

  float vertMean = meanOf(vert, GAIT_WINDOW_SAMPLES);
  float vertStd = stdOf(vert, GAIT_WINDOW_SAMPLES, vertMean);
  float mlMean = meanOf(ml, GAIT_WINDOW_SAMPLES);
  float mlStd = stdOf(ml, GAIT_WINDOW_SAMPLES, mlMean);
  float yawMean = meanOf(yaw, GAIT_WINDOW_SAMPLES);
  float yawStd = stdOf(yaw, GAIT_WINDOW_SAMPLES, yawMean);

  features[0]  = vertMean;
  features[1]  = vertStd;
  features[2]  = rangeOf(vert, GAIT_WINDOW_SAMPLES);
  features[3]  = skewOf(vert, GAIT_WINDOW_SAMPLES, vertMean, vertStd);
  features[4]  = kurtOf(vert, GAIT_WINDOW_SAMPLES, vertMean, vertStd);
  features[5]  = mlStd;
  features[6]  = rmsOf(ml, GAIT_WINDOW_SAMPLES);
  features[7]  = rangeOf(ml, GAIT_WINDOW_SAMPLES);
  features[8]  = yawStd;
  features[9]  = absMeanOf(yaw, GAIT_WINDOW_SAMPLES);

  int peakIdx[128];
  int peakCount = findStepPeaks(vert, GAIT_WINDOW_SAMPLES, peakIdx, 128);
  features[10] = (float)peakCount;

  if (peakCount > 1) {
    float sumIntervals = 0.0f;
    float sumIntervalsSq = 0.0f;
    for (int i = 1; i < peakCount; i++) {
      float interval = (float)(peakIdx[i] - peakIdx[i - 1]) / TARGET_HZ;
      sumIntervals += interval;
      sumIntervalsSq += interval * interval;
    }
    float meanInterval = sumIntervals / (peakCount - 1);
    features[11] = 60.0f / meanInterval;
    float intervalVar = (sumIntervalsSq / (peakCount - 1)) - (meanInterval * meanInterval);
    if (intervalVar < 0.0f) intervalVar = 0.0f;
    features[12] = (meanInterval > 1e-6f) ? sqrtf(intervalVar) / meanInterval : 0.0f;
  } else {
    features[11] = 0.0f;
    features[12] = 0.0f;
  }

  float bestAc = 0.0f;
  int bestLag = 0;
  for (int lag = 30; lag < 80; lag++) {
    float ac = autocorrAtLag(vert, GAIT_WINDOW_SAMPLES, lag);
    if (ac > bestAc) {
      bestAc = ac;
      bestLag = lag;
    }
  }
  features[13] = bestAc;

  int strideLag = bestLag * 2;
  float strideAc = (strideLag < GAIT_WINDOW_SAMPLES) ? autocorrAtLag(vert, GAIT_WINDOW_SAMPLES, strideLag) : 0.0f;
  features[14] = strideAc;
  features[15] = (strideAc > 0.1f) ? (bestAc / strideAc) : 0.0f;

  float jerk[GAIT_WINDOW_SAMPLES - 1];
  for (int i = 0; i < GAIT_WINDOW_SAMPLES - 1; i++) {
    jerk[i] = vert[i + 1] - vert[i];
  }
  features[16] = stdOf(jerk, GAIT_WINDOW_SAMPLES - 1, meanOf(jerk, GAIT_WINDOW_SAMPLES - 1));
  float jerkMax = 0.0f;
  for (int i = 0; i < GAIT_WINDOW_SAMPLES - 1; i++) {
    float a = fabsf(jerk[i]);
    if (a > jerkMax) jerkMax = a;
  }
  features[17] = jerkMax;

  return true;
}

bool isValidGaitWindow(const float features[18]) {
  bool enoughSteps = features[10] >= 4.0f;
  bool cadenceOk = features[11] >= 40.0f && features[11] <= 180.0f;
  bool motionOk = features[1] >= 0.15f;
  bool periodicEnough = features[13] >= 0.10f;
  return enoughSteps && cadenceOk && motionOk && periodicEnough;
}

const char* labelToText(MlLabel label) {
  switch (label) {
    case LABEL_HEALTHY_GAIT: return "healthy_gait";
    case LABEL_STROKE_GAIT:  return "stroke_gait";
    default:                 return "no_gait";
  }
}

void publishMlLabel(MlLabel rawLabel, MlLabel stableLabel, const float features[18]) {
  char msg[160];
  unsigned long t = millis() - recordStart;
  snprintf(msg, sizeof(msg), "%lu,%s,%s,%.0f,%.1f,%.3f",
           t,
           labelToText(rawLabel),
           labelToText(stableLabel),
           features[10],
           features[11],
           features[13]);
  if (mlChar != nullptr) {
    mlChar->setValue(std::string(msg));
    if (deviceConnected) {
      mlChar->notify();
    }
  }

  if (SERIAL_DEBUG && ML_DEBUG_FEATURES) {
    Serial.print("# ML ");
    for (int i = 0; i < 18; i++) {
      if (i) Serial.print(',');
      Serial.print(features[i], 4);
    }
    Serial.println();
  }
}

void runGaitInferenceIfReady() {
  if (gaitCount < GAIT_WINDOW_SAMPLES) return;
  if (samplesSincePrediction < GAIT_PREDICTION_STEP) return;
  samplesSincePrediction = 0;

  copyGaitWindow();

  float features[18];
  if (!extractGaitFeatures(gaitWin, features)) return;

  MlLabel rawLabel = LABEL_NO_GAIT;
  if (isValidGaitWindow(features)) {
    int pred = gaitClassifier.predict(features);
    rawLabel = (pred == 0) ? LABEL_HEALTHY_GAIT : LABEL_STROKE_GAIT;
  }

  MlLabel stableLabel = mlSmoother.update(rawLabel);
  publishMlLabel(rawLabel, stableLabel, features);
}

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
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(addr, (uint8_t)1);
  if (!Wire.available()) return false;

  uint8_t who = Wire.read();
  return who == 0x68 || who == 0x70 || who == 0x71;
}

void wakeMPU(uint8_t addr) {
  // PWR_MGMT_1: clear sleep, select gyro X PLL (CLKSEL = 1) for stability.
  // The internal 8 MHz RC oscillator drifts noticeably with temperature.
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(100);

  // SMPLRT_DIV = 9 → 1 kHz / (1 + 9) = 100 Hz output rate, matching our reads.
  Wire.beginTransmission(addr);
  Wire.write(0x19);
  Wire.write(0x09);
  Wire.endTransmission();

  // CONFIG: DLPF_CFG = 4 → ~21 Hz accel / ~20 Hz gyro low-pass (recommended for gait)
  Wire.beginTransmission(addr);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  // GYRO_CONFIG: ±1000 °/s (FS_SEL = 2, bits 4:3).
  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();

  // ACCEL_CONFIG: ±2 g (AFS_SEL = 0).
  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (mpuAddr == 0) return false;

  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);  // ACCEL_XOUT_H — burst 14 bytes through GYRO_ZOUT_L.
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 16384.0f;  // ±2 g full scale → 16384 LSB/g
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;

  gx = rawGx / 32.8f;     // ±1000 °/s full scale → 32.8 LSB/(°/s)
  gy = rawGy / 32.8f;
  gz = rawGz / 32.8f;

  return true;
}

// ───────── CALIBRATION ─────────
void calibrateMPU() {
  if (SERIAL_DEBUG) Serial.println("# Calibrating... keep device still");

  const int N = 300;
  float sx = 0, sy = 0, sz = 0;
  float sgx = 0, sgy = 0, sgz = 0;
  int ok = 0;

  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sx  += ax;  sy  += ay;  sz  += az;
      sgx += gx;  sgy += gy;  sgz += gz;
      ok++;
    }
    delay(10);
  }
  if (ok < 50) {
    if (SERIAL_DEBUG) Serial.println("# Calibration FAILED — too few good reads");
    return;
  }

  axBias = sx  / ok;  ayBias = sy  / ok;  azBias = sz  / ok;
  gxBias = sgx / ok;  gyBias = sgy / ok;  gzBias = sgz / ok;

  // Preserve 1 g of gravity on whichever axis is dominant at rest, so the
  // phone-side step detector (which expects rest |a| ≈ 1.0 g) keeps working
  // regardless of how the belt is mounted.
  float absX = fabsf(axBias), absY = fabsf(ayBias), absZ = fabsf(azBias);
  if (absX >= absY && absX >= absZ) {
    axBias -= (axBias >= 0 ? 1.0f : -1.0f);
  } else if (absY >= absZ) {
    ayBias -= (ayBias >= 0 ? 1.0f : -1.0f);
  } else {
    azBias -= (azBias >= 0 ? 1.0f : -1.0f);
  }

  if (SERIAL_DEBUG) {
    Serial.print("# Calibration done — good reads: ");
    Serial.println(ok);
  }
}

// ───────── BLE ─────────
void startRecording() {
  recordStart = millis();
  recording = true;
  resetGaitPipeline();
  if (SERIAL_DEBUG) Serial.println("# RECORDING STARTED");
  showRecording();
}

void stopRecording() {
  recording = false;
  if (SERIAL_DEBUG) Serial.println("# RECORDING STOPPED");
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
    if      (v[0] == 0x20) startRecording();
    else if (v[0] == 0x21) stopRecording();
  }
};

void setupBLE() {
  NimBLEDevice::init("SafeStride-Logger");
  NimBLEDevice::setMTU(247);

  NimBLEServer* s = NimBLEDevice::createServer();
  s->setCallbacks(new ServerCB());

  NimBLEService* svc = s->createService(SERVICE_UUID);

  cmdChar = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  cmdChar->setCallbacks(new CmdCB());

  logChar = svc->createCharacteristic(LOG_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  mlChar = svc->createCharacteristic(ML_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  // Name in the primary packet, full 128-bit UUID in the scan response —
  // they won't both fit in 31 bytes alongside flags.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName("SafeStride-Logger");
  adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setCompleteServices(NimBLEUUID(SERVICE_UUID));
  adv->setScanResponseData(scanResp);

  adv->start();
}

// ───────── SETUP ─────────
void setup() {
  // Skip Serial.begin entirely when SERIAL_DEBUG is off — saves the 1.5 s
  // settle wait at boot and frees the UART hardware.
  if (SERIAL_DEBUG) {
    Serial.begin(921600);
    delay(1500);
  }

  Wire.begin();
  Wire.setClock(400000);
  delay(200);

  oled.begin();
  bmp.begin();

  // Average a few BMP samples to lock in a stable baseline pressure.
  float pSum = 0;
  int   pN   = 0;
  for (int i = 0; i < 10; i++) {
    float pBar = bmp.getPressureBar(false);  // hPa
    if (pBar > 500.0f && pBar < 1200.0f) {
      pSum += pBar * 100.0f;  // Pa
      pN++;
    }
    delay(20);
  }
  if (pN > 0) baselinePressure = pSum / pN;

  if      (probeMPU(MPU_ADDR_PRIMARY)) mpuAddr = MPU_ADDR_PRIMARY;
  else if (probeMPU(MPU_ADDR_ALT))     mpuAddr = MPU_ADDR_ALT;

  if (mpuAddr) {
    wakeMPU(mpuAddr);
    delay(50);
    calibrateMPU();
  } else {
    if (SERIAL_DEBUG) Serial.println("# MPU6050 not found on bus");
  }

  showWaiting();
  setupBLE();
}

// ───────── LOOP ─────────
void loop() {
  static unsigned long lastSample = 0;
  static unsigned long lastBaro   = 0;

  unsigned long now = millis();

  // Refresh barometer-derived altitude at ~5 Hz, well off the IMU cadence.
  // The BMP180 driver blocks for ~10 ms per read (two internal delay(5)s),
  // so we run it once per ~20 IMU samples rather than every iteration.
  if (now - lastBaro >= BARO_PERIOD_MS) {
    lastBaro = now;
    float pBar = bmp.getPressureBar(false);
    if (pBar > 0.0f) {
      float pPa = pBar * 100.0f;
      float rawAlt = 44330.0f * (1.0f - powf(pPa / baselinePressure, 1.0f / 5.255f));
      cachedAlt = altLP.update(rawAlt, ALT_ALPHA);
    }
    now = millis();  // BMP read just consumed ~10 ms; refresh.
  }

  if (now - lastSample < SAMPLE_PERIOD_MS) return;

  // If something stalled the loop past CATCHUP_THRESHOLD_MS, drop the missed
  // samples instead of bursting them all back-to-back. Otherwise advance by
  // exactly SAMPLE_PERIOD_MS to keep the long-run cadence locked to 100 Hz.
  if (now - lastSample > CATCHUP_THRESHOLD_MS) lastSample = now;
  else                                         lastSample += SAMPLE_PERIOD_MS;

  float ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) return;

  // Bias-correct
  ax -= axBias;  ay -= ayBias;  az -= azBias;
  gx -= gxBias;  gy -= gyBias;  gz -= gzBias;

  // Feed ML pipeline with the same body-frame variables the trainer used,
  // remapped from this hardware's mounting (MYOSA at the lumbar, chip face
  // posterior; Y vertical, X medio-lateral, Z anteroposterior — confirmed
  // from the GY-521 silkscreen). The training script uses ax/ay/gx for
  // vert/ml/yaw because its dataset has X-vertical mounting; here we map
  // ay/ax/gy to give the model the same body-frame quantities it expects.
  pushGaitSample((ay - 1.0f) * G_CONST, ax * G_CONST, gy);
  if (recording) {
    runGaitInferenceIfReady();
  }

  // Update software low-pass filters (keeps smooth output, but low lag with chosen alpha)
  float axf = axLP.update(ax, ACC_ALPHA);
  float ayf = ayLP.update(ay, ACC_ALPHA);
  float azf = azLP.update(az, ACC_ALPHA);
  float gxf = gxLP.update(gx, GYR_ALPHA);
  float gyf = gyLP.update(gy, GYR_ALPHA);
  float gzf = gzLP.update(gz, GYR_ALPHA);

  // Prepare values to stream: either filtered or raw bias-corrected depending on flag
  float out_ax = STREAM_FILTERED_VALUES ? axf : ax;
  float out_ay = STREAM_FILTERED_VALUES ? ayf : ay;
  float out_az = STREAM_FILTERED_VALUES ? azf : az;
  float out_gx = STREAM_FILTERED_VALUES ? gxf : gx;
  float out_gy = STREAM_FILTERED_VALUES ? gyf : gy;
  float out_gz = STREAM_FILTERED_VALUES ? gzf : gz;

  if (!recording) return;

  unsigned long t = millis() - recordStart;

  // Skip the snprintf entirely if nothing's listening — at 100 Hz this
  // saves ~80 µs / sample and reduces BLE-notify jitter.
  if (!deviceConnected && !SERIAL_DEBUG) return;

  char buf[128];
  int n = snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f",
                   t, out_ax, out_ay, out_az, out_gx, out_gy, out_gz, cachedAlt);
  if (n > 0) {
    if (SERIAL_DEBUG) Serial.println(buf);
    if (deviceConnected) {
      logChar->setValue(std::string(buf));
      logChar->notify();
    }
  }
}
