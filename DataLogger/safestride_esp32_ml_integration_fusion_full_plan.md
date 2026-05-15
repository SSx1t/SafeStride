# SafeStride ESP32 DataLogger + ML Integration + Fusion Execution Plan

## Purpose

This is an execution plan for an implementation agent. The goal is to combine the existing SafeStride DataLogger firmware, the tuning/noise-reduction work, and the exported gait machine-learning model into one reliable ESP32-side inference path.

The plan has four goals:

1. Keep the current 100 Hz BLE CSV DataLogger working.
2. Add ESP32-side gait inference from `gait_model.h` without breaking the phone/app parser.
3. Fuse simple walking-validity logic, model prediction, and label smoothing into one stable output.
4. Validate that the ESP32-generated features match the Python training pipeline before trusting live predictions.

Important: the current model is a gait classifier for `healthy` vs `stroke`. It is not yet a clinically validated `normal` vs `about_to_fall` model. This plan includes the correct path for future fall-risk work, but the first ESP32 implementation must label the current model honestly.

---

## 1. Current facts from the uploaded files

### 1.1 DataLogger status

The uploaded `DataLogger.ino` is still primarily a BLE CSV streamer with this production schema:

```text
t_ms,ax,ay,az,gx,gy,gz,alt
```

It currently streams 100 Hz IMU data plus slower altitude updates. The project README also describes `DataLogger.ino` as the working firmware used to collect calibration CSVs and describes the same BLE behavior.

Agent action: keep this 8-column production stream unchanged.

### 1.2 Previous tuning plan status

The uploaded `DataLogger.ino` should be audited because the file still appears to contain old patterns such as:

```cpp
Wire.write(0x03);   // DLPF_CFG = 3
String csv = ...;   // repeated String concatenation at 100 Hz
```

Agent action: do not assume the tuning plan is already merged into the actual repository. Run the audit in Phase 0 and either confirm it is merged or apply it before ML integration.

### 1.3 Training script status

The uploaded `train_model.py` contains this active configuration:

```python
SOURCE_HZ = 104
TARGET_HZ = 100
WINDOW_SEC = 10.0
WINDOW_SIZE = int(WINDOW_SEC * TARGET_HZ)
STRIDE = WINDOW_SIZE // 2
```

This means the active model training uses 10-second windows at 100 Hz, i.e. 1000 samples per window. Some comments still say 5 seconds or 500 samples. Treat the code, not the comment, as the source of truth until this is corrected.

The script defines 18 features in this order:

```text
0  vert_mean
1  vert_std
2  vert_range
3  vert_skew
4  vert_kurt
5  ml_std
6  ml_rms
7  ml_range
8  yaw_std
9  yaw_abs_mean
10 step_count
11 cadence_spm
12 step_interval_cv
13 step_ac
14 stride_ac
15 gsi
16 jerk_std
17 jerk_max
```

The script currently maps the dataset axes as:

```python
DATASET_VERT = 'ax'
DATASET_ML   = 'ay'
DATASET_YAW  = 'gx'
```

Then it computes:

```python
vert_dyn = (vert_raw - 1.0) * 9.81
ml_dyn   = ml_raw * 9.81
yaw      = gx
```

Agent action: the firmware must match these units, axis choices, window size, and feature order exactly, or the Random Forest output will be unreliable.

### 1.4 Model header status

The uploaded `gait_model.h` defines:

```cpp
Eloquent::ML::Port::GaitClassifier
int predict(float *x)
```

The current uploaded header appears to contain 10 Random Forest trees, while the uploaded `train_model.py` trains the final model with 20 trees. Treat the header as stale until regenerated from the current training script.

Agent action: retrain or reload the intended final model, regenerate `gait_model.h`, and verify the number of trees in the header matches the final `n_estimators`.

### 1.5 Current model label meaning

The current model returns:

```text
0 = healthy
1 = stroke
```

It does not return:

```text
0 = normal
1 = about_to_fall
```

Agent action: publish labels as `healthy_gait` and `stroke_gait`, or use neutral internal names such as `gait_class_0` and `gait_class_1` until the product team approves the wording.

---

## 2. Non-negotiable integration rules

1. Do not feed raw `ax, ay, az, gx, gy, gz` directly into `GaitClassifier::predict()`.
2. Build the exact 18-feature vector expected by the model.
3. Do not call the classifier every 10 ms.
4. Run inference only after a full rolling window is available.
5. Keep the current 8-column BLE log characteristic unchanged.
6. Add a second BLE characteristic for ML labels/debug output.
7. Do not use altitude as a model input for the current Random Forest because the training data did not include altitude.
8. Do not claim fall-risk prediction until a fall-risk dataset, labels, model, and validation exist.
9. Use non-blocking scheduling with `millis()` or equivalent timing; do not add blocking delays in the 100 Hz loop.
10. Validate feature parity between Python and ESP32 before evaluating model accuracy.

---

## 3. Final architecture target

The target runtime pipeline is:

```text
MPU6050 + BMP180
    |
    v
100 Hz sensor read
    |
    v
bias correction and optional light filtering
    |
    +------------------------------+
    |                              |
    v                              v
existing BLE CSV stream            gait feature pipeline
8 columns only                     vertical dynamic accel
                                   medio-lateral dynamic accel
                                   yaw gyro
                                   rolling window
                                   18 features
                                   walking-validity gate
                                   Random Forest prediction
                                   label smoothing
                                   ML BLE characteristic
```

The fusion strategy is:

```text
1. Sensor-level fusion:
   - IMU is used for gait features.
   - Barometer altitude is streamed and logged, but not used by the current RF model.

2. Decision-level fusion:
   - A simple gait-validity gate decides whether the current window looks like walking.
   - If the window does not look like walking, publish `no_gait` or `unknown`, not `healthy/stroke`.
   - If the window looks like walking, run the Random Forest and map 0/1 to healthy_gait/stroke_gait.
   - Apply label smoothing and hold-time logic before publishing.

3. Interface fusion:
   - Keep `LOG_CHAR_UUID` as raw sensor CSV.
   - Add `ML_CHAR_UUID` as label/status output.
```

---

## 4. Phase 0 - Repository audit before implementation

Run these checks first.

```bash
grep -n "DLPF\|Wire.write(0x03)\|Wire.write(0x04)" DataLogger.ino
grep -n "LowPass\|ACC_ALPHA\|GYR_ALPHA\|ALT_ALPHA" DataLogger.ino
grep -n "String csv\|snprintf" DataLogger.ino
grep -n "WINDOW_SEC\|WINDOW_SIZE\|n_estimators\|FEATURE_COLS" train_model.py
grep -c "tree #" gait_model.h
```

Expected target after all work:

```text
DataLogger.ino:
  DLPF_CFG selected and documented, default 0x04 unless testing says otherwise
  LowPass1 filters present if the tuning plan is supposed to be merged
  snprintf() used for CSV, not String concatenation

train_model.py:
  WINDOW_SEC intentionally chosen and comments corrected
  final model hyperparameters match the exported header

gait_model.h:
  tree count matches train_model.py final n_estimators
```

If the previous noise-reduction changes are not present, apply them before the ML work:

```text
- DLPF_CFG from 0x03 to 0x04 initially
- EMA filters for accel, gyro, altitude
- higher CSV precision
- snprintf() instead of repeated String concatenation
- no change to the 8-column BLE CSV schema
```

---

## 5. Phase 1 - Resolve training/model alignment

### 5.1 Choose one window length

There are two valid options.

#### Option A: Use the uploaded training code as-is

Use:

```text
WINDOW_SEC = 10.0
WINDOW_SIZE = 1000 samples at 100 Hz
STRIDE = 500 samples
```

Firmware constants:

```cpp
static const int TARGET_HZ = 100;
static const int GAIT_WINDOW_SAMPLES = 1000;
static const int GAIT_PREDICTION_STEP = 50;  // 500 ms
```

This gives slower first feedback, because the first label cannot appear until the first 10-second window is full.

#### Option B: Return to the original 5-second design

Edit `train_model.py`:

```python
WINDOW_SEC = 5.0
WINDOW_SIZE = int(WINDOW_SEC * TARGET_HZ)  # 500 samples
STRIDE = WINDOW_SIZE // 2
```

Then retrain and regenerate the header.

Firmware constants:

```cpp
static const int TARGET_HZ = 100;
static const int GAIT_WINDOW_SAMPLES = 500;
static const int GAIT_PREDICTION_STEP = 50;  // 500 ms
```

Recommended default for immediate implementation:

```text
Use Option A first for model parity with the current script.
After the on-device feature pipeline is validated, retrain with Option B if 10-second feedback is too slow for rehabilitation use.
```

### 5.2 Make feature extraction firmware-friendly

The current Python feature extraction uses:

```python
pd.Series(vert).skew()
pd.Series(vert).kurt()
scipy.signal.find_peaks(...)
```

These can be ported, but they are easy to mismatch. The recommended implementation is to replace them with explicit firmware-friendly functions in Python, retrain, and then implement the same formulas in C++.

Add Python helpers like this in `train_model.py`:

```python
def fw_mean(x):
    return float(np.mean(x))

def fw_std(x):
    m = np.mean(x)
    return float(np.sqrt(np.mean((x - m) ** 2)))

def fw_skew(x):
    m = np.mean(x)
    s = fw_std(x)
    if s < 1e-6:
        return 0.0
    return float(np.mean(((x - m) / s) ** 3))

def fw_kurt(x):
    m = np.mean(x)
    s = fw_std(x)
    if s < 1e-6:
        return 0.0
    return float(np.mean(((x - m) / s) ** 4) - 3.0)

def fw_find_step_peaks(vert, threshold=0.35 * 9.81, min_distance=25):
    peaks = []
    last = -min_distance
    for i in range(1, len(vert) - 1):
        is_peak = (
            vert[i] > threshold and
            vert[i] > vert[i - 1] and
            vert[i] >= vert[i + 1] and
            (i - last) >= min_distance
        )
        if is_peak:
            peaks.append(i)
            last = i
    return np.array(peaks, dtype=np.int32)
```

Then change `extract_features()` to use these helpers.

If the team does not want to retrain, then the ESP32 must replicate the current pandas/scipy behavior closely and prove parity with a golden test window. Retraining with explicit formulas is safer and easier.

### 5.3 Align final model hyperparameters

The final model block in `train_model.py` should be the only source of truth. Use the same values in cross-validation and final training unless there is a deliberate reason not to.

Recommended initial final model:

```python
clf_final = RandomForestClassifier(
    n_estimators=20,
    max_depth=9,
    min_samples_leaf=10,
    class_weight='balanced',
    random_state=42,
    n_jobs=-1
)
```

Also update the cross-validation block to use the same values.

### 5.4 Retrain and regenerate

Run:

```bash
python train_model.py
python convert_to_cpp.py
```

Copy the regenerated header into the Arduino sketch folder that will compile the firmware:

```text
DataLogger/gait_model.h
```

or:

```text
SafeStride/gait_model.h
```

depending on which sketch is being compiled.

Verify tree count:

```bash
grep -c "tree #" gait_model.h
```

Expected:

```text
20
```

if `n_estimators=20` is the chosen final setting.

---

## 6. Phase 2 - Build a Python golden-window parity test

Before writing much firmware, generate one or more golden feature vectors from Python. These will be used to confirm the ESP32 feature extractor.

Create a script named:

```text
make_golden_window.py
```

Example behavior:

```python
import json
import numpy as np
import pandas as pd
from train_model import TARGET_HZ, WINDOW_SIZE, FEATURE_COLS, extract_features, G

csv_path = "calibration/healthy_1.csv"  # update path as needed

df = pd.read_csv(csv_path, comment="#", names=["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "alt"])
df = df[df["t_ms"] < 1e9].reset_index(drop=True)

# Match the firmware axis mapping selected in Phase 4.
vert_g = df["ax"].values[:WINDOW_SIZE]
ml_g = df["ay"].values[:WINDOW_SIZE]
yaw = df["gx"].values[:WINDOW_SIZE]

vert_dyn = (vert_g - 1.0) * G
ml_dyn = ml_g * G

feat_dict = extract_features(vert_dyn, ml_dyn, yaw)
features = [float(feat_dict[c]) for c in FEATURE_COLS]

with open("golden_window_features.json", "w") as f:
    json.dump({"feature_order": FEATURE_COLS, "features": features}, f, indent=2)

print(features)
```

After firmware implementation, the ESP32 should print the same 18 features for the same window. The comparison does not need to be bit-exact, but it should be close.

Initial acceptance target:

```text
Absolute error for normal-scale features: <= 1e-3 to 1e-2
Relative error for large features: <= 1 percent
Step count: exact match preferred
Cadence: <= 1 step/min difference
```

If parity fails, do not evaluate model predictions yet. Fix feature extraction first.

---

## 7. Phase 3 - Firmware integration plan

### 7.1 Add model include

Place `gait_model.h` next to the compiled Arduino sketch and add near the top of `DataLogger.ino`:

```cpp
#include "gait_model.h"

Eloquent::ML::Port::GaitClassifier gaitClassifier;
```

Do not edit `gait_model.h` by hand.

### 7.2 Add ML BLE characteristic

Keep the existing raw data characteristic unchanged:

```cpp
#define LOG_CHAR_UUID "4f5b1a04-7b3a-4f9b-9e80-1a1e5b7c2d40"
```

Add a new characteristic:

```cpp
#define ML_CHAR_UUID  "4f5b1a05-7b3a-4f9b-9e80-1a1e5b7c2d40"

NimBLECharacteristic* mlChar = nullptr;
```

Inside `setupBLE()` after `logChar` is created:

```cpp
mlChar = svc->createCharacteristic(
  ML_CHAR_UUID,
  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
);
```

Recommended ML label payload:

```text
t_ms,raw_label,stable_label,step_count,cadence_spm
```

Example:

```text
12345,healthy_gait,healthy_gait,12,104.2
```

Do not append this to the existing 8-column raw CSV stream.

### 7.3 Add constants

Use these values if current `train_model.py` remains at 10 seconds:

```cpp
static const float G_CONST = 9.81f;
static const int TARGET_HZ = 100;
static const int GAIT_WINDOW_SAMPLES = 1000;
static const int GAIT_PREDICTION_STEP = 50;  // every 500 ms at 100 Hz
```

If retrained to 5 seconds, use:

```cpp
static const int GAIT_WINDOW_SAMPLES = 500;
```

### 7.4 Add rolling buffer

Use fixed-size global/static arrays. Do not allocate memory in `loop()`.

```cpp
struct GaitSample {
  float vertDyn;  // m/s^2
  float mlDyn;    // m/s^2
  float yaw;      // deg/s
};

GaitSample gaitBuf[GAIT_WINDOW_SAMPLES];
int gaitHead = 0;
int gaitCount = 0;
int samplesSincePrediction = 0;

float vertWin[GAIT_WINDOW_SAMPLES];
float mlWin[GAIT_WINDOW_SAMPLES];
float yawWin[GAIT_WINDOW_SAMPLES];
```

Memory estimate:

```text
1000 samples * 3 floats * 4 bytes = 12000 bytes for gaitBuf
3 extra 1000-float windows = 12000 bytes
Total feature buffers about 24 KB
```

This is acceptable on ESP32, but keep arrays global/static.

### 7.5 Reset ML buffers on recording start

In `startRecording()` add:

```cpp
resetGaitPipeline();
```

Implement:

```cpp
void resetGaitPipeline() {
  gaitHead = 0;
  gaitCount = 0;
  samplesSincePrediction = 0;
  // Also reset label smoother here.
}
```

This prevents old data from a previous session from contaminating the first prediction.

### 7.6 Axis mapping

The Python training path expects:

```text
vertical acceleration in g, including about +1 g at rest
medio-lateral acceleration in g
yaw gyro in deg/s
```

Then it computes:

```text
vert_dyn = (vertical_g - 1.0) * 9.81
ml_dyn = ml_g * 9.81
yaw = yaw_dps
```

The firmware calibration should preserve the 1 g gravity component on the dominant vertical axis. Track that axis explicitly.

Add globals:

```cpp
enum AxisId : uint8_t {
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2
};

AxisId verticalAxis = AXIS_X;
float verticalSign = 1.0f;

// Start with these only if hardware mounting matches the dataset convention.
// Validate with recorded walking data.
AxisId mlAxis = AXIS_Y;
AxisId yawAxis = AXIS_X;
```

During calibration, after computing `axBias`, `ayBias`, and `azBias`, set the vertical axis:

```cpp
float absX = fabsf(axBias), absY = fabsf(ayBias), absZ = fabsf(azBias);

if (absX >= absY && absX >= absZ) {
  verticalAxis = AXIS_X;
  verticalSign = (axBias >= 0.0f) ? 1.0f : -1.0f;
  axBias -= verticalSign;
} else if (absY >= absZ) {
  verticalAxis = AXIS_Y;
  verticalSign = (ayBias >= 0.0f) ? 1.0f : -1.0f;
  ayBias -= verticalSign;
} else {
  verticalAxis = AXIS_Z;
  verticalSign = (azBias >= 0.0f) ? 1.0f : -1.0f;
  azBias -= verticalSign;
}
```

Add helpers:

```cpp
float readAccelAxis(AxisId axis, float ax, float ay, float az) {
  if (axis == AXIS_X) return ax;
  if (axis == AXIS_Y) return ay;
  return az;
}

float readGyroAxis(AxisId axis, float gx, float gy, float gz) {
  if (axis == AXIS_X) return gx;
  if (axis == AXIS_Y) return gy;
  return gz;
}
```

Build the model input sample:

```cpp
float vertical_g = verticalSign * readAccelAxis(verticalAxis, ax, ay, az);
float ml_g = readAccelAxis(mlAxis, ax, ay, az);
float yaw_dps = readGyroAxis(yawAxis, gx, gy, gz);

float vertDyn = (vertical_g - 1.0f) * G_CONST;
float mlDyn = ml_g * G_CONST;

pushGaitSample(vertDyn, mlDyn, yaw_dps);
```

Agent warning: if the belt orientation differs from the Felius lower-back dataset orientation, hard-coded `mlAxis=AXIS_Y` and `yawAxis=AXIS_X` may be wrong. Use recorded walking data and feature parity tests to confirm.

### 7.7 Push samples

```cpp
void pushGaitSample(float vertDyn, float mlDyn, float yaw) {
  gaitBuf[gaitHead].vertDyn = vertDyn;
  gaitBuf[gaitHead].mlDyn = mlDyn;
  gaitBuf[gaitHead].yaw = yaw;

  gaitHead = (gaitHead + 1) % GAIT_WINDOW_SAMPLES;

  if (gaitCount < GAIT_WINDOW_SAMPLES) {
    gaitCount++;
  }

  samplesSincePrediction++;
}
```

Call this once per valid 100 Hz IMU sample after bias correction and after deciding whether the model uses raw/bias-corrected or filtered values.

Recommended default:

```text
For first model parity: use the same signal path that was used in recordings/training.
If retraining with filtered data, use filtered data in both Python and firmware.
```

### 7.8 Copy circular window into chronological arrays

```cpp
void copyGaitWindow() {
  int start = gaitHead;  // oldest sample when buffer is full

  for (int i = 0; i < GAIT_WINDOW_SAMPLES; i++) {
    int idx = (start + i) % GAIT_WINDOW_SAMPLES;
    vertWin[i] = gaitBuf[idx].vertDyn;
    mlWin[i] = gaitBuf[idx].mlDyn;
    yawWin[i] = gaitBuf[idx].yaw;
  }
}
```

Call this only after:

```cpp
gaitCount >= GAIT_WINDOW_SAMPLES
```

---

## 8. Phase 4 - Port feature extraction to C++

### 8.1 Helper functions

Use population standard deviation to match NumPy `np.std()` default.

```cpp
float meanOf(const float* x, int n) {
  double s = 0.0;
  for (int i = 0; i < n; i++) s += x[i];
  return (float)(s / n);
}

float stdOf(const float* x, int n, float mean) {
  double s2 = 0.0;
  for (int i = 0; i < n; i++) {
    double d = (double)x[i] - mean;
    s2 += d * d;
  }
  return (float)sqrt(s2 / n);
}

float rangeOf(const float* x, int n) {
  float mn = x[0];
  float mx = x[0];
  for (int i = 1; i < n; i++) {
    if (x[i] < mn) mn = x[i];
    if (x[i] > mx) mx = x[i];
  }
  return mx - mn;
}

float rmsOf(const float* x, int n) {
  double s2 = 0.0;
  for (int i = 0; i < n; i++) s2 += (double)x[i] * x[i];
  return (float)sqrt(s2 / n);
}

float absMeanOf(const float* x, int n) {
  double s = 0.0;
  for (int i = 0; i < n; i++) s += fabsf(x[i]);
  return (float)(s / n);
}

float skewOf(const float* x, int n, float mean, float stdv) {
  if (stdv < 1e-6f) return 0.0f;
  double s3 = 0.0;
  for (int i = 0; i < n; i++) {
    double z = ((double)x[i] - mean) / stdv;
    s3 += z * z * z;
  }
  return (float)(s3 / n);
}

float kurtOf(const float* x, int n, float mean, float stdv) {
  if (stdv < 1e-6f) return 0.0f;
  double s4 = 0.0;
  for (int i = 0; i < n; i++) {
    double z = ((double)x[i] - mean) / stdv;
    s4 += z * z * z * z;
  }
  return (float)((s4 / n) - 3.0);
}

float autocorrAtLag(const float* x, int n, int lag) {
  if (lag >= n) return 0.0f;

  float m = meanOf(x, n);

  double denom = 0.0;
  for (int i = 0; i < n; i++) {
    double d = (double)x[i] - m;
    denom += d * d;
  }

  if (denom < 1e-6) return 0.0f;

  double num = 0.0;
  for (int i = 0; i < n - lag; i++) {
    num += ((double)x[i] - m) * ((double)x[i + lag] - m);
  }

  return (float)(num / denom);
}
```

### 8.2 Peak detection

Use the same simple detector in Python and C++ if retraining as recommended.

```cpp
int findStepPeaks(const float* x, int n, int* peakIdx, int maxPeaks) {
  const float threshold = 0.35f * G_CONST;
  const int minDistance = 25;

  int count = 0;
  int stored = 0;
  int lastPeak = -minDistance;

  for (int i = 1; i < n - 1; i++) {
    bool isPeak = x[i] > threshold &&
                  x[i] > x[i - 1] &&
                  x[i] >= x[i + 1] &&
                  (i - lastPeak) >= minDistance;

    if (isPeak) {
      if (stored < maxPeaks) {
        peakIdx[stored] = i;
        stored++;
      }
      count++;
      lastPeak = i;
    }
  }

  return count;
}
```

### 8.3 Extract features

```cpp
bool extractGaitFeatures(float features[18]) {
  if (gaitCount < GAIT_WINDOW_SAMPLES) return false;

  copyGaitWindow();

  const int n = GAIT_WINDOW_SAMPLES;

  float vertMean = meanOf(vertWin, n);
  float vertStd = stdOf(vertWin, n, vertMean);

  float mlMean = meanOf(mlWin, n);
  float yawMean = meanOf(yawWin, n);

  features[0] = vertMean;
  features[1] = vertStd;
  features[2] = rangeOf(vertWin, n);
  features[3] = skewOf(vertWin, n, vertMean, vertStd);
  features[4] = kurtOf(vertWin, n, vertMean, vertStd);

  features[5] = stdOf(mlWin, n, mlMean);
  features[6] = rmsOf(mlWin, n);
  features[7] = rangeOf(mlWin, n);

  features[8] = stdOf(yawWin, n, yawMean);
  features[9] = absMeanOf(yawWin, n);

  static int peaks[256];
  int stepCount = findStepPeaks(vertWin, n, peaks, 256);
  int storedPeaks = stepCount;
  if (storedPeaks > 256) storedPeaks = 256;

  features[10] = (float)stepCount;

  if (storedPeaks > 1) {
    double sumIntervals = 0.0;
    double intervals[255];
    int intervalCount = storedPeaks - 1;

    for (int i = 0; i < intervalCount; i++) {
      intervals[i] = (double)(peaks[i + 1] - peaks[i]) / TARGET_HZ;
      sumIntervals += intervals[i];
    }

    double meanInterval = sumIntervals / intervalCount;

    if (meanInterval > 1e-6) {
      double s2 = 0.0;
      for (int i = 0; i < intervalCount; i++) {
        double d = intervals[i] - meanInterval;
        s2 += d * d;
      }
      double stdInterval = sqrt(s2 / intervalCount);
      features[11] = (float)(60.0 / meanInterval);
      features[12] = (float)(stdInterval / meanInterval);
    } else {
      features[11] = 0.0f;
      features[12] = 0.0f;
    }
  } else {
    features[11] = 0.0f;
    features[12] = 0.0f;
  }

  float bestAc = 0.0f;
  int bestLag = 0;
  for (int lag = 30; lag < 80; lag++) {
    float ac = autocorrAtLag(vertWin, n, lag);
    if (ac > bestAc) {
      bestAc = ac;
      bestLag = lag;
    }
  }

  features[13] = bestAc;

  int strideLag = bestLag * 2;
  float strideAc = 0.0f;
  if (strideLag < n) {
    strideAc = autocorrAtLag(vertWin, n, strideLag);
  }

  features[14] = strideAc;
  features[15] = (strideAc > 0.1f) ? (bestAc / strideAc) : 0.0f;

  if (n > 1) {
    double jerkMean = 0.0;
    int jn = n - 1;

    for (int i = 0; i < jn; i++) {
      jerkMean += (double)(vertWin[i + 1] - vertWin[i]);
    }
    jerkMean /= jn;

    double s2 = 0.0;
    float jerkMax = 0.0f;

    for (int i = 0; i < jn; i++) {
      float j = vertWin[i + 1] - vertWin[i];
      double d = (double)j - jerkMean;
      s2 += d * d;
      float aj = fabsf(j);
      if (aj > jerkMax) jerkMax = aj;
    }

    features[16] = (float)sqrt(s2 / jn);
    features[17] = jerkMax;
  } else {
    features[16] = 0.0f;
    features[17] = 0.0f;
  }

  for (int i = 0; i < 18; i++) {
    if (!isfinite(features[i])) features[i] = 0.0f;
  }

  return true;
}
```

Important: if the Python training script is not changed to use these firmware-friendly formulas, this C++ extractor may not exactly match the old Python extractor. Feature parity testing decides whether the implementation is acceptable.

---

## 9. Phase 5 - Walking-validity gate and model fusion

The Random Forest was trained on walking gait windows. It should not be forced to classify stillness, sitting, or random handling as healthy/stroke.

Add a gate that decides whether the window contains valid walking/gait before using the model output.

```cpp
enum MlLabel : uint8_t {
  LABEL_NO_GAIT = 0,
  LABEL_HEALTHY_GAIT = 1,
  LABEL_STROKE_GAIT = 2
};

bool isValidGaitWindow(const float features[18]) {
  float vertStd = features[1];
  float stepCount = features[10];
  float cadence = features[11];
  float stepAc = features[13];

  bool enoughSteps = stepCount >= 4.0f;
  bool cadenceOk = cadence >= 40.0f && cadence <= 180.0f;
  bool motionOk = vertStd >= 0.15f;
  bool periodicEnough = stepAc >= 0.10f;

  return enoughSteps && cadenceOk && motionOk && periodicEnough;
}

MlLabel fuseGateAndModel(const float features[18]) {
  if (!isValidGaitWindow(features)) {
    return LABEL_NO_GAIT;
  }

  int pred = gaitClassifier.predict((float*)features);

  if (pred == 0) return LABEL_HEALTHY_GAIT;
  if (pred == 1) return LABEL_STROKE_GAIT;

  return LABEL_NO_GAIT;
}
```

Initial thresholds to tune:

```text
step_count >= 4
cadence between 40 and 180 steps/min
vert_std >= 0.15 m/s^2
step_ac >= 0.10
```

These are engineering thresholds. Tune them using real recordings.

---

## 10. Phase 6 - Label smoothing

Do not publish the raw label directly. Use a small label history and a minimum hold time.

```cpp
class MlLabelSmoother {
public:
  static const uint8_t N = 7;

  uint8_t buf[N] = {0};
  uint8_t head = 0;
  uint8_t filled = 0;

  MlLabel stable = LABEL_NO_GAIT;
  uint32_t lastChangeMs = 0;

  void reset() {
    for (uint8_t i = 0; i < N; i++) buf[i] = LABEL_NO_GAIT;
    head = 0;
    filled = 0;
    stable = LABEL_NO_GAIT;
    lastChangeMs = millis();
  }

  MlLabel update(MlLabel raw, uint32_t nowMs) {
    if (raw > LABEL_STROKE_GAIT) return stable;

    buf[head] = (uint8_t)raw;
    head = (head + 1) % N;
    if (filled < N) filled++;

    uint8_t votes[3] = {0, 0, 0};
    for (uint8_t i = 0; i < filled; i++) {
      if (buf[i] <= LABEL_STROKE_GAIT) votes[buf[i]]++;
    }

    MlLabel candidate = stable;
    uint8_t bestVotes = votes[(uint8_t)stable];

    for (uint8_t label = 0; label <= LABEL_STROKE_GAIT; label++) {
      if (votes[label] > bestVotes) {
        bestVotes = votes[label];
        candidate = (MlLabel)label;
      }
    }

    // Require strong agreement before changing labels.
    uint32_t minHoldMs = 800;
    uint8_t requiredVotes = 5;

    if (candidate != stable &&
        bestVotes >= requiredVotes &&
        nowMs - lastChangeMs >= minHoldMs) {
      stable = candidate;
      lastChangeMs = nowMs;
    }

    return stable;
  }
};

MlLabelSmoother mlSmoother;
```

For a future real fall-risk model, use asymmetric smoothing:

```text
Enter fall-risk quickly.
Exit fall-risk slowly.
```

But for the current healthy/stroke model, use symmetric majority-vote smoothing.

---

## 11. Phase 7 - Main loop inference scheduling

Add a function:

```cpp
void runGaitInferenceIfReady() {
  if (gaitCount < GAIT_WINDOW_SAMPLES) return;
  if (samplesSincePrediction < GAIT_PREDICTION_STEP) return;

  samplesSincePrediction = 0;

  float features[18];
  if (!extractGaitFeatures(features)) return;

  MlLabel rawLabel = fuseGateAndModel(features);
  MlLabel stableLabel = mlSmoother.update(rawLabel, millis());

  publishMlLabel(rawLabel, stableLabel, features);
}
```

Call it after pushing the current sample:

```cpp
pushGaitSample(vertDyn, mlDyn, yaw_dps);
runGaitInferenceIfReady();
```

Important: keep 100 Hz sampling first. If inference causes timing gaps, optimize feature extraction or run inference less often.

Recommended first prediction interval:

```text
50 samples = 500 ms
```

If stable and responsive enough, keep it. If UI feedback is too slow, try:

```text
25 samples = 250 ms
```

---

## 12. Phase 8 - Publish ML label without breaking raw CSV

Add:

```cpp
const char* labelToText(MlLabel label) {
  switch (label) {
    case LABEL_NO_GAIT: return "no_gait";
    case LABEL_HEALTHY_GAIT: return "healthy_gait";
    case LABEL_STROKE_GAIT: return "stroke_gait";
    default: return "unknown";
  }
}

void publishMlLabel(MlLabel rawLabel, MlLabel stableLabel, const float features[18]) {
  char msg[160];
  unsigned long t = recording ? (millis() - recordStart) : millis();

  int n = snprintf(
    msg,
    sizeof(msg),
    "%lu,%s,%s,%.0f,%.1f,%.3f",
    t,
    labelToText(rawLabel),
    labelToText(stableLabel),
    features[10],
    features[11],
    features[13]
  );

  if (n <= 0 || n >= (int)sizeof(msg)) return;

  Serial.print("# ML,");
  Serial.println(msg);

  if (deviceConnected && mlChar != nullptr) {
    mlChar->setValue((uint8_t*)msg, (size_t)n);
    mlChar->notify();
  }
}
```

The ML characteristic schema is:

```text
t_ms,raw_label,stable_label,step_count,cadence_spm,step_ac
```

Example:

```text
12345,healthy_gait,healthy_gait,11,103.7,0.522
```

Do not write this payload to `LOG_CHAR_UUID`.

---

## 13. Phase 9 - Debug feature output mode

Add a compile-time flag:

```cpp
static const bool ML_DEBUG_FEATURES = false;
```

When true, print the feature vector:

```cpp
void printFeatureDebug(const float features[18]) {
  Serial.print("# ML_FEATURES");
  for (int i = 0; i < 18; i++) {
    Serial.print(',');
    Serial.print(features[i], 6);
  }
  Serial.println();
}
```

Call after extraction:

```cpp
if (ML_DEBUG_FEATURES) {
  printFeatureDebug(features);
}
```

Use this to compare ESP32 features against `golden_window_features.json`.

Disable it for normal BLE recording.

---

## 14. Phase 10 - Noise/tuning plan to keep fused with ML work

The tuning changes remain part of the complete plan.

### 14.1 MPU6050 DLPF

Initial setting:

```cpp
Wire.write(0x1A);
Wire.write(0x04);  // DLPF_CFG = 4
```

Expected DLPF settings to test:

```text
3: about 44 Hz accel / 42 Hz gyro, lower delay, noisier
4: about 21 Hz accel / 20 Hz gyro, recommended first gait setting
5: about 10 Hz accel / 10 Hz gyro, smoother but more delay
```

Start at `0x04`. Only use `0x05` if the model remains too noisy and stumble/fast-motion response is still acceptable.

### 14.2 EMA filters

Recommended starting values:

```cpp
static const float ACC_ALPHA = 0.30f;
static const float GYR_ALPHA = 0.35f;
static const float ALT_ALPHA = 0.10f;
```

Keep filters warm even when not recording.

### 14.3 Raw vs filtered model input

There are two safe choices:

```text
Choice 1: train on raw/bias-corrected data and run model on raw/bias-corrected data.
Choice 2: train on filtered data and run model on filtered data.
```

Do not train one way and infer the other way without validating accuracy.

Recommended first approach:

```text
Use the same signal path that generated the training/calibration data.
Use filtered values for display/log smoothness only if the model is validated with that path.
```

### 14.4 CSV precision and String replacement

The raw stream should use fixed buffers:

```cpp
char csv[180];
int n = snprintf(csv, sizeof(csv),
                 "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f",
                 t, ax, ay, az, gx, gy, gz, cachedAlt);
```

Keep exactly 8 comma-separated fields.

---

## 15. Phase 11 - End-to-end validation checklist

### 15.1 Build checks

The firmware must compile with:

```text
DataLogger.ino
NimBLE Arduino library
BarometricPressure library
OLED dependency, if still used
gait_model.h in the sketch include path
```

No dynamic allocation in the 100 Hz loop.

### 15.2 BLE checks

Verify:

```text
Device advertises as SafeStride-Logger.
Command 0x20 starts recording.
Command 0x21 stops recording.
LOG_CHAR_UUID still emits 8-column CSV.
ML_CHAR_UUID emits ML label payload.
Raw CSV parser remains compatible.
```

### 15.3 Timing checks

Record 60 seconds and calculate:

```text
median sample interval
95th percentile sample interval
maximum normal sample gap
samples per second
number of ML labels per second
```

Acceptance targets:

```text
Median sample interval: about 10 ms
95th percentile sample interval: <= 15 ms preferred
Maximum ordinary sample gap: <= 50 ms preferred
ML output interval: about 500 ms if GAIT_PREDICTION_STEP=50
No inference-related stalls visible in raw CSV timing
```

### 15.4 Feature parity checks

Use the same recorded CSV window in Python and ESP32 debug mode.

Acceptance targets:

```text
All 18 features present and in correct order.
Feature signs and units match.
Step count matches or is explained.
Cadence close to Python output.
Autocorrelation features close enough for stable prediction.
```

Do not proceed to live prediction evaluation until this passes.

### 15.5 Label checks

Stationary table test:

```text
Expected stable ML label: no_gait
```

Standing still while worn:

```text
Expected stable ML label: no_gait
```

Normal walking:

```text
Expected stable ML label: healthy_gait or stroke_gait depending on subject/model
Expected behavior: no rapid flicker
```

Short handling/transitions:

```text
Expected behavior: label should not bounce every 500 ms
```

### 15.6 Regression checks for noise tuning

Repeat the earlier tuning recordings:

```text
table still, 30 seconds
worn standing, 30 seconds
normal walking, 60 seconds
turn walking, 60 seconds
safe stumble/recovery simulation only if approved
```

Compare:

```text
stationary accel std
stationary gyro std
altitude jitter
label transitions per minute
first-label latency
feature parity errors
```

---

## 16. Phase 12 - Future fall-risk model plan

The current model cannot honestly be used as `about_to_fall` detection. To build that model, create a new dataset and a new label schema.

Required classes might be:

```text
0 = no_gait / stationary
1 = normal_gait
2 = abnormal_gait / unstable_gait
3 = near_fall_or_loss_of_balance
```

Required data collection:

```text
normal walking
slow walking
standing
sitting
turning
limping
post-stroke gait
stumbling
recovery steps
harness-protected near-fall simulations, if clinically approved
```

Training changes:

```text
- Include real SafeStride belt recordings, not only Felius lower-back data.
- Decide whether altitude should be included.
- If altitude is included, add altitude-derived features and retrain.
- Export a new model header with the new class names.
- Update label smoother to enter fall-risk quickly and exit slowly.
```

Safety rule:

```text
False alarms are less dangerous than missed fall-risk warnings, but neither should be reported as clinically validated without a validation protocol.
```

---

## 17. Agent execution checklist

Execute in this order:

```text
1. Audit current repo against Phase 0.
2. Apply missing DataLogger tuning changes if they are not already merged.
3. Resolve window length: default to current 10-second script unless retraining to 5 seconds is explicitly chosen.
4. Make feature extraction firmware-friendly in Python or commit to exact pandas/scipy parity.
5. Align CV and final Random Forest hyperparameters.
6. Run train_model.py.
7. Run convert_to_cpp.py.
8. Copy regenerated gait_model.h into the firmware sketch folder.
9. Verify tree count in gait_model.h.
10. Add gait_model.h include and GaitClassifier object.
11. Add ML BLE characteristic.
12. Add axis mapping and rolling gait buffer.
13. Port feature extraction.
14. Add walking-validity gate.
15. Add label smoother.
16. Add publishMlLabel().
17. Add debug feature output mode.
18. Flash firmware.
19. Verify raw BLE CSV is unchanged.
20. Verify ML characteristic output.
21. Run Python-vs-ESP32 feature parity test.
22. Tune gate thresholds and label smoothing.
23. Save before/after recordings and metrics.
24. Document final constants used.
```

---

## 18. Definition of done

The implementation is done only when all of these are true:

```text
- DataLogger still streams exactly 8 raw CSV fields on LOG_CHAR_UUID.
- A second ML characteristic publishes label/status output.
- gait_model.h was regenerated from the current selected model.
- The number of trees in gait_model.h matches final n_estimators.
- Firmware window length matches train_model.py.
- Firmware feature order matches feature_order.txt / FEATURE_COLS.
- ESP32 feature vector matches Python golden-window features.
- The classifier is called every 250-500 ms, not every 10 ms.
- The walking-validity gate prevents stillness from being classified as healthy/stroke gait.
- Label smoothing prevents rapid flicker.
- Current labels are documented as healthy/stroke gait, not fall-risk.
- Timing remains close to 100 Hz with no inference-induced stalls.
- All constants and thresholds are written down.
```

---

## 19. References for the agent

Project files:

```text
README.md
DataLogger.ino
train_model.py
gait_model.h
safestride_datalogger_tuning_and_changes_plan.md
```

External technical references:

```text
MPU-6000/MPU-6050 Register Map and Descriptions, Register 26 DLPF table:
https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf

micromlgen repository, sklearn model export to C/C++ for microcontrollers:
https://github.com/eloquentarduino/micromlgen

Arduino millis documentation for non-blocking timing:
https://docs.arduino.cc/language-reference/en/functions/time/millis/

Sliding-window segmentation for inertial-sensor human activity recognition:
https://www.mdpi.com/1424-8220/19/22/5026
```
