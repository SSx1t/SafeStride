# SafeStride DataLogger Noise Reduction and Classification Smoothing Plan

## Purpose

This file is an execution plan for an implementation agent. The goal is to make the SafeStride smart-belt DataLogger output more stable and to stop movement labels from flickering rapidly between classes such as `walking` and `not_walking`.

The problem has two separate causes and must be fixed in two separate layers:

1. **Sensor/data noise**: IMU and altitude values jump around, so the streamed numbers look unstable.
2. **Decision noise**: the movement classifier changes labels too quickly, for example `walking -> not_walking -> walking` within one second.

Do not try to solve decision noise only by heavily smoothing the raw IMU. Excessive sensor filtering can hide real stumble or fall-warning motion. Use light sensor filtering plus explicit classifier-output smoothing.

---

## Current Project Context

From the project notes:

- `DataLogger/DataLogger.ino` is the working ESP32 firmware used for calibration recordings.
- It streams CSV over BLE at 100 Hz with this schema:

```text
t_ms,ax,ay,az,gx,gy,gz,alt
```

- Acceleration is in `g`, gyro is in `deg/s`, altitude is relative to boot pressure.
- The production app currently does classification from raw IMU samples on the phone side.
- The C++ `GaitClassifier` path is currently not used by the production DataLogger unless someone implements the rolling buffer, feature extractor, and model call on the ESP32.
- The trained gait model expects 5-second windows at 100 Hz and an 18-feature vector in the exact order from `feature_order.txt`.
- The existing exported model returns `0 = healthy` and `1 = stroke`; it is not automatically a clinically valid `normal` vs `about_to_fall` model.

Important implication:

> If labels are flickering today, the label-smoothing code probably belongs in the phone app or wherever classification is currently performed, not necessarily inside `DataLogger.ino`.

Still, this plan includes both firmware changes and classifier-output smoothing so the agent can execute the correct branch.

---

## Files to Modify

### Required

```text
DataLogger/DataLogger.ino
```

### Conditional

Modify one of these depending on where live classification actually runs:

```text
Android app classification module
```

or

```text
SafeStride/SafeStride.ino
SafeStride/gait.h
SafeStride/sensors.h
SafeStride/gait_model.h usage path
```

Only implement ESP-side classification if the project explicitly decides to run the model on the ESP32.

---

## High-Level Execution Order

1. Create a branch named something like:

```text
fix/datalogger-noise-and-label-smoothing
```

2. Make sensor-output changes in `DataLogger.ino`:
   - Change MPU6050 DLPF setting from `3` to `4` initially.
   - Add software exponential moving average filters.
   - Smooth altitude separately.
   - Increase CSV numeric precision.
   - Replace repeated Arduino `String` concatenation with a fixed `char` buffer and `snprintf()`.

3. Confirm whether classification is phone-side or ESP-side.

4. Add decision smoothing in the classification component:
   - Use rolling windows for classification, not single IMU samples.
   - Add majority voting and minimum hold time.
   - Add asymmetric fall-risk behavior: enter fall-risk quickly, exit slowly.
   - If classifier probabilities are available, add probability smoothing and hysteresis.

5. Run the tuning plan and collect before/after recordings.

6. Accept only when both numeric stability and label stability pass the acceptance criteria in this document.

---

# Part A: DataLogger Firmware Changes

## A1. Change the MPU6050 Hardware Low-Pass Filter

In `wakeMPU(uint8_t addr)`, find this block:

```cpp
// CONFIG: DLPF_CFG = 3 -> 44 Hz accel / 42 Hz gyro low-pass.
Wire.beginTransmission(addr);
Wire.write(0x1A);
Wire.write(0x03);
Wire.endTransmission();
```

Replace it with:

```cpp
// CONFIG: DLPF_CFG = 4 -> about 21 Hz accel / 20 Hz gyro low-pass.
// Good first setting for gait/walking classification at 100 Hz.
// If still too noisy, test 0x05 later. If fall/stumble transients look delayed,
// revert to 0x03 or keep 0x04 with higher software-filter alpha.
Wire.beginTransmission(addr);
Wire.write(0x1A);
Wire.write(0x04);
Wire.endTransmission();
```

Tuning values to test:

| `DLPF_CFG` | Accel bandwidth | Gyro bandwidth | Use case |
|---:|---:|---:|---|
| `3` | about 44 Hz | about 42 Hz | Current setting; more responsive, noisier |
| `4` | about 21 Hz | about 20 Hz | First recommended gait setting |
| `5` | about 10 Hz | about 10 Hz | Smoother, but more delay; use carefully |

Do not start with `6` unless testing proves it is safe, because it adds more delay and may suppress fast loss-of-balance signals.

---

## A2. Add Software Low-Pass Filters

Add this near the global variables in `DataLogger.ino`, after the existing globals such as `cachedAlt`:

```cpp
// ───────── FILTERING / SMOOTHING ─────────
// Set to 1 for smoother BLE/Serial output. Set to 0 to stream bias-corrected
// raw values while still computing filtered values internally for debugging.
static const bool STREAM_FILTERED_VALUES = true;

// Exponential moving average coefficients.
// Higher alpha = faster response but noisier output.
// Lower alpha = smoother output but more lag.
static const float ACC_ALPHA = 0.30f;
static const float GYR_ALPHA = 0.35f;
static const float ALT_ALPHA = 0.10f;

struct LowPass1 {
  bool ready = false;
  float y = 0.0f;

  float update(float x, float alpha) {
    if (!ready) {
      y = x;
      ready = true;
      return y;
    }
    y += alpha * (x - y);
    return y;
  }

  void reset() {
    ready = false;
    y = 0.0f;
  }
};

LowPass1 axLP, ayLP, azLP;
LowPass1 gxLP, gyLP, gzLP;
LowPass1 altLP;
```

Why these defaults:

- `ACC_ALPHA = 0.30` gives moderate smoothing at 100 Hz.
- `GYR_ALPHA = 0.35` keeps gyro response slightly faster.
- `ALT_ALPHA = 0.10` is slower because barometric altitude is already low-rate and noisy.

---

## A3. Smooth Altitude

In `loop()`, find this section:

```cpp
if (pBar > 0.0f) {
  float pPa = pBar * 100.0f;
  cachedAlt = 44330.0f * (1.0f - powf(pPa / baselinePressure, 1.0f / 5.255f));
}
```

Replace it with:

```cpp
if (pBar > 0.0f) {
  float pPa = pBar * 100.0f;
  float rawAlt = 44330.0f * (1.0f - powf(pPa / baselinePressure, 1.0f / 5.255f));
  cachedAlt = altLP.update(rawAlt, ALT_ALPHA);
}
```

---

## A4. Apply IMU Software Filtering After Bias Correction

In `loop()`, find this part:

```cpp
ax -= axBias;  ay -= ayBias;  az -= azBias;
gx -= gxBias;  gy -= gyBias;  gz -= gzBias;

if (!recording) return;
```

Replace it with:

```cpp
ax -= axBias;  ay -= ayBias;  az -= azBias;
gx -= gxBias;  gy -= gyBias;  gz -= gzBias;

// Keep filters warm even when not recording, so recording starts with stable values.
float axFilt = axLP.update(ax, ACC_ALPHA);
float ayFilt = ayLP.update(ay, ACC_ALPHA);
float azFilt = azLP.update(az, ACC_ALPHA);
float gxFilt = gxLP.update(gx, GYR_ALPHA);
float gyFilt = gyLP.update(gy, GYR_ALPHA);
float gzFilt = gzLP.update(gz, GYR_ALPHA);

if (STREAM_FILTERED_VALUES) {
  ax = axFilt;
  ay = ayFilt;
  az = azFilt;
  gx = gxFilt;
  gy = gyFilt;
  gz = gzFilt;
}

if (!recording) return;
```

Important:

- If the current phone-side model was trained on unfiltered sensor values, this change can shift the live input distribution.
- For production classification, either:
  - train and validate using the same filtered pipeline, or
  - stream raw values to the classifier and use filtered values only for UI/debug display.

Recommended first implementation:

```cpp
static const bool STREAM_FILTERED_VALUES = true;
```

Recommended A/B testing:

```cpp
static const bool STREAM_FILTERED_VALUES = false;
```

Run the same trials with both settings and compare classification stability and fall/stumble responsiveness.

---

## A5. Increase CSV Precision and Avoid Repeated String Concatenation

Current code:

```cpp
String csv = String(t) + "," +
             String(ax, 2) + "," +
             String(ay, 2) + "," +
             String(az, 2) + "," +
             String(gx, 2) + "," +
             String(gy, 2) + "," +
             String(gz, 2) + "," +
             String(cachedAlt, 3);

Serial.println(csv);

if (deviceConnected) {
  logChar->setValue(csv);
  logChar->notify();
}
```

Replace it with:

```cpp
char csv[180];

int n = snprintf(csv, sizeof(csv),
                 "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f",
                 t, ax, ay, az, gx, gy, gz, cachedAlt);

if (n <= 0) return;
if (n >= (int)sizeof(csv)) {
  // Truncated line. Do not send partial CSV.
  Serial.println("# CSV line truncated");
  return;
}

Serial.println(csv);

if (deviceConnected) {
  logChar->setValue((uint8_t*)csv, (size_t)n);
  logChar->notify();
}
```

Reason:

- Acceleration with only 2 decimals can make the signal look step-like and unstable.
- `%.4f` for acceleration gives enough detail for gait features.
- `%.3f` for gyro and altitude is usually enough.
- A fixed buffer avoids repeated dynamic string construction at 100 Hz.

Keep the BLE schema unchanged:

```text
t_ms,ax,ay,az,gx,gy,gz,alt
```

Do not add extra columns to the production BLE stream unless the phone app/parser is updated at the same time.

---

# Part B: Classification Smoothing Changes

## B1. Confirm Where Classification Runs

Before coding label smoothing, inspect the project and answer this:

```text
Does live classification run on the phone app, or on the ESP32?
```

If it runs on the phone:

```text
Implement Part B in the phone app classification module.
Do not put label smoothing in DataLogger.ino unless the ESP creates labels.
```

If it runs on the ESP32:

```text
Implement a 5-second rolling window, feature extraction, model prediction, and label smoothing on the ESP32.
```

---

## B2. Do Not Classify Every IMU Sample

The DataLogger samples at 100 Hz. One sample every 10 ms is too short for a stable movement label.

Use this windowing strategy:

```text
Sample rate:             100 Hz
Feature window length:   5 seconds
Samples per window:      500 samples
Prediction interval:     every 250 ms to 500 ms
```

Recommended first setting:

```text
window = 500 samples
prediction_step = 50 samples  // 500 ms at 100 Hz
```

If the UI needs faster feedback:

```text
prediction_step = 25 samples  // 250 ms at 100 Hz
```

Do not go faster until flicker is solved.

Pipeline:

```text
raw IMU
  -> bias correction
  -> optional light filtering
  -> rolling 5-second buffer
  -> extract training-compatible features
  -> classifier prediction
  -> decision smoother
  -> final displayed label
```

Do not do this:

```text
one IMU sample -> classifier -> displayed label
```

---

## B3. Add Majority-Vote Smoothing for Labels

Use this when classifier output is only a class label, not probabilities.

Add this logic in the classification component, not necessarily in `DataLogger.ino`:

```cpp
enum MovementLabel : uint8_t {
  LABEL_NORMAL = 0,
  LABEL_WALKING = 1,
  LABEL_FALL_RISK = 2
};

class LabelSmoother {
public:
  static const uint8_t N = 7;

  uint8_t buf[N] = {0};
  uint8_t head = 0;
  uint8_t filled = 0;

  uint8_t stableLabel = LABEL_NORMAL;
  uint32_t lastChangeMs = 0;

  uint8_t update(uint8_t rawLabel, uint32_t nowMs) {
    if (rawLabel > LABEL_FALL_RISK) {
      return stableLabel;
    }

    buf[head] = rawLabel;
    head = (head + 1) % N;
    if (filled < N) filled++;

    uint8_t votes[3] = {0, 0, 0};
    for (uint8_t i = 0; i < filled; i++) {
      if (buf[i] <= LABEL_FALL_RISK) votes[buf[i]]++;
    }

    // Safety behavior: enter fall-risk quickly.
    if (stableLabel != LABEL_FALL_RISK && votes[LABEL_FALL_RISK] >= 2) {
      stableLabel = LABEL_FALL_RISK;
      lastChangeMs = nowMs;
      return stableLabel;
    }

    // Safety behavior: leave fall-risk slowly.
    if (stableLabel == LABEL_FALL_RISK) {
      uint8_t nonFallCandidate =
        (votes[LABEL_WALKING] >= votes[LABEL_NORMAL]) ? LABEL_WALKING : LABEL_NORMAL;

      uint8_t nonFallVotes = votes[nonFallCandidate];

      if (votes[LABEL_FALL_RISK] == 0 &&
          nonFallVotes >= 5 &&
          nowMs - lastChangeMs >= 1500) {
        stableLabel = nonFallCandidate;
        lastChangeMs = nowMs;
      }
      return stableLabel;
    }

    // Normal/walking behavior: require strong agreement and minimum hold time.
    uint8_t candidate = stableLabel;
    uint8_t bestVotes = votes[stableLabel];

    if (votes[LABEL_NORMAL] > bestVotes) {
      candidate = LABEL_NORMAL;
      bestVotes = votes[LABEL_NORMAL];
    }
    if (votes[LABEL_WALKING] > bestVotes) {
      candidate = LABEL_WALKING;
      bestVotes = votes[LABEL_WALKING];
    }

    if (candidate != stableLabel &&
        bestVotes >= 5 &&
        nowMs - lastChangeMs >= 800) {
      stableLabel = candidate;
      lastChangeMs = nowMs;
    }

    return stableLabel;
  }
};
```

Use it like this:

```cpp
LabelSmoother smoother;

uint8_t rawPrediction = runClassifier(...);
uint8_t stablePrediction = smoother.update(rawPrediction, millis());

showOrSendLabel(stablePrediction);
```

Initial parameters:

| Parameter | Initial value | Meaning |
|---|---:|---|
| Label history | 7 predictions | Smooth over recent predictions |
| Non-fall switch threshold | 5 of 7 | Prevent walking/not-walking flicker |
| Non-fall minimum hold | 800 ms | Do not switch normal/walking too fast |
| Fall-risk entry | 2 of 7 | Enter quickly for safety |
| Fall-risk exit delay | 1500 ms | Do not blink the warning off immediately |

For a binary `walking` / `not_walking` classifier, use:

```text
history = 7 predictions
switch only if 5 of last 7 agree
minimum hold time = 800 ms to 1000 ms
```

---

## B4. Use Probability Hysteresis If Probabilities Are Available

If the classifier can output probabilities, smooth probabilities before converting to labels.

Example:

```cpp
float pWalkSmooth = 0.0f;
float pFallSmooth = 0.0f;
static const float PROB_ALPHA = 0.25f;

void updateProbabilities(float pWalk, float pFall) {
  pWalkSmooth = PROB_ALPHA * pWalk + (1.0f - PROB_ALPHA) * pWalkSmooth;
  pFallSmooth = PROB_ALPHA * pFall + (1.0f - PROB_ALPHA) * pFallSmooth;
}
```

Use different enter and exit thresholds:

```text
Walking:
  enter walking if pWalkSmooth > 0.70
  leave walking if pWalkSmooth < 0.35

Fall risk:
  enter fall-risk if pFallSmooth > 0.75
  leave fall-risk only if pFallSmooth < 0.35 for at least 1500 ms
```

This is hysteresis. It prevents label changes when the classifier confidence hovers around the decision boundary.

If both majority vote and probabilities are available, use probability hysteresis first and then apply a short majority-vote/hold-time smoother to the resulting label.

---

# Part C: ESP-Side Classification Branch, Only If Needed

Only execute this section if the project decides to run the classifier on the ESP32.

## C1. Implement Rolling Window

At 100 Hz:

```text
5 seconds = 500 samples
```

Maintain a circular buffer for the variables used by the model features.

Recommended buffer contents:

```cpp
struct GaitSample {
  float vertDyn;
  float mlDyn;
  float yaw;
};

static const int GAIT_WINDOW_SAMPLES = 500;
GaitSample gaitBuf[GAIT_WINDOW_SAMPLES];
int gaitHead = 0;
int gaitCount = 0;
```

## C2. Match Training Feature Extraction Exactly

Do not feed the Random Forest raw `ax, ay, az, gx, gy, gz` samples directly.

The exported model expects the same 18 features that `train_model.py` generated. The feature order must match `feature_order.txt` exactly.

Required order:

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

## C3. Classification Schedule

Do not call the model on every sample.

Recommended:

```text
Start predictions only after gaitCount >= 500.
Run prediction every 50 samples, which is 500 ms.
```

Pseudocode:

```cpp
if (gaitCount >= GAIT_WINDOW_SAMPLES && samplesSincePrediction >= 50) {
  samplesSincePrediction = 0;

  float features[18];
  extractGaitFeatures(gaitBuf, features);

  int rawLabel = classifier.predict(features);
  int stableLabel = smoother.update(rawLabel, millis());

  publishStableLabel(stableLabel);
}
```

---

# Part D: Tuning Plan

## D1. Collect Baseline Before Changes

Before changing the code, record these sessions with the current firmware:

```text
1. table_still_baseline.csv       30 seconds, device still on table
2. worn_standing_baseline.csv     30 seconds, belt worn while standing still
3. normal_walk_baseline.csv       60 seconds, normal walking
4. turn_walk_baseline.csv         60 seconds, walking with turns
5. stumble_sim_baseline.csv       30 seconds, safely simulated stumble/recovery if allowed
```

Log both:

```text
sensor CSV
raw classifier label stream, if available
final displayed label stream, if available
```

Do not do unsafe fall trials without a clinical/safety protocol.

---

## D2. Apply Firmware Changes and Re-Test

After A1 through A5, record the same sessions:

```text
1. table_still_filtered.csv
2. worn_standing_filtered.csv
3. normal_walk_filtered.csv
4. turn_walk_filtered.csv
5. stumble_sim_filtered.csv
```

Compare these metrics before/after:

```text
median sample interval, ms
95th percentile sample interval, ms
maximum sample gap, ms
stationary accel standard deviation per axis
stationary gyro standard deviation per axis
accel magnitude mean while still
accel magnitude standard deviation while still
label transitions per minute during steady walking
minimum duration of displayed label states
fall-risk entry latency in simulated stumble trials, if available
fall-risk exit delay after recovery
```

Suggested acceptance targets:

```text
Median sample interval: about 10 ms
95th percentile sample interval: <= 15 ms preferred
Maximum normal sample gap: <= 50 ms preferred
Stationary filtered accel std: at least 30% lower than baseline
Stationary filtered gyro std: at least 30% lower than baseline
Displayed walking/not-walking label switches: no more than 1 switch per second during steady behavior
Displayed non-fall state duration: usually >= 800 ms
Fall-risk warning: no rapid blinking; stay active at least 1500 ms after trigger
```

These are engineering targets, not clinical validation targets.

---

## D3. Tune DLPF Setting

Test in this order:

```text
DLPF_CFG = 4
DLPF_CFG = 5
DLPF_CFG = 3
```

Choose the lowest-noise setting that does not delay or hide important stumble/fall-warning motion.

Decision guide:

```text
If numbers are still noisy and labels flicker:
  try DLPF_CFG = 5

If walking looks stable but simulated stumble response is too slow:
  revert to DLPF_CFG = 4 or 3

If DLPF_CFG = 4 is good enough:
  keep DLPF_CFG = 4
```

Recommended default after tuning:

```text
DLPF_CFG = 4
```

---

## D4. Tune EMA Alphas

Initial values:

```cpp
ACC_ALPHA = 0.30f;
GYR_ALPHA = 0.35f;
ALT_ALPHA = 0.10f;
```

Tune using this table:

| Symptom | Change |
|---|---|
| Sensor values still visibly noisy | Decrease alpha by 0.05 |
| Walking/stumble response feels delayed | Increase alpha by 0.05 |
| Altitude jumps too much | Decrease `ALT_ALPHA` to `0.05` |
| Altitude is too slow | Increase `ALT_ALPHA` to `0.15` |

Do not reduce acceleration/gyro alpha below `0.15` without explicit validation, because too much lag can hide fast motion.

Recommended tested ranges:

```text
ACC_ALPHA: 0.20 to 0.40
GYR_ALPHA: 0.25 to 0.50
ALT_ALPHA: 0.05 to 0.15
```

---

## D5. Tune Label Smoother

Initial values:

```text
history size = 7
normal/walking switch threshold = 5 of 7
normal/walking minimum hold = 800 ms
fall-risk entry = 2 of 7
fall-risk exit = 1500 ms
```

Tune using this guide:

| Symptom | Change |
|---|---|
| Walking/not-walking still flickers | Increase min hold to 1000 ms or require 6 of 7 |
| UI feels too slow to switch between standing and walking | Reduce min hold to 600 ms, keep 5 of 7 |
| Fall-risk blinks on/off | Increase fall-risk exit delay to 2000 ms |
| Fall-risk triggers too often | Require 3 of 7 fall-risk votes or raise probability threshold |
| Fall-risk response is too slow | Keep 2 of 7, reduce prediction interval to 250 ms |

Safety preference:

```text
False warning is less dangerous than missed fall-risk warning.
Fall-risk should enter faster than it exits.
```

---

# Part E: Validation Checklist

## E1. Compile and Flash

- Firmware compiles without warnings related to missing includes or invalid `setValue()` overloads.
- Device advertises as `SafeStride-Logger`.
- BLE command `0x20` starts recording.
- BLE command `0x21` stops recording.
- CSV line count is consistent with 100 Hz recording.

## E2. CSV Schema Check

Each production BLE line must still have exactly 8 comma-separated fields:

```text
t_ms,ax,ay,az,gx,gy,gz,alt
```

Example expected format:

```text
1234,0.0123,0.9981,-0.0210,0.135,-0.244,0.017,0.032
```

Do not send comments, headers, or debug columns over the BLE log characteristic unless the receiver parser is updated.

## E3. Sensor Stability Check

On table-still data:

- Filtered accelerometer values should be visibly smoother than baseline.
- Filtered gyro should be close to zero and less jumpy.
- Altitude should not jump rapidly while the device is stationary.
- Acceleration magnitude at rest should remain physically reasonable and near 1 g if gravity is preserved by calibration.

## E4. Label Stability Check

During steady walking:

- Raw classifier may flicker.
- Displayed/stable label should not flicker.
- Normal/walking labels should not change faster than the configured hold time.

Expected behavior:

```text
Raw classifier:
walking, walking, not_walking, walking, not_walking, walking

Displayed/stable classifier:
walking, walking, walking, walking, walking, walking
```

For fall-risk:

```text
Raw classifier:
normal, normal, fall_risk, normal, fall_risk, fall_risk

Displayed/stable classifier:
normal, normal, fall_risk, fall_risk, fall_risk, fall_risk
```

---

# Part F: Important Warnings

## F1. Filtering Must Match Training

If the model was trained on raw data but live inference uses filtered data, accuracy can change.

Acceptable approaches:

1. Use filtered data only for display, while the classifier still receives raw/bias-corrected data.
2. Retrain the model using the same filter settings used in live firmware.
3. Run an A/B validation comparing raw-inference and filtered-inference on the same trials.

## F2. Current Model Is Not Necessarily a Fall-Risk Model

The current exported model is documented as `healthy` vs `stroke`. Do not claim it detects `about_to_fall` unless the training data and validation explicitly include near-fall/stumble/fall-risk examples.

Needed data for a real fall-risk classifier:

```text
normal walking
standing
sitting
turning
slow walking
limping
stumbling
loss of balance
recovery steps
clinically/safely simulated near-falls
```

## F3. Do Not Over-Smooth Fall Signals

Too much smoothing can make the output look nice while hiding fast instability.

The correct design is:

```text
light sensor filtering + robust window features + label hysteresis
```

not:

```text
very heavy raw filtering only
```

---

# Part G: Definition of Done

The implementation is done only when all of these are true:

- `DataLogger.ino` compiles and streams the same 8-column BLE CSV schema.
- MPU DLPF is changed to the selected tuned value, initially `0x04`.
- Software filters are implemented and configurable.
- CSV precision is increased to 4 decimals for accel and 3 decimals for gyro/altitude.
- Repeated `String` concatenation at 100 Hz is replaced with `snprintf()` and a fixed buffer.
- Classification is confirmed to run either phone-side or ESP-side.
- The component that actually creates labels has decision smoothing.
- Walking/not-walking labels do not flicker rapidly in steady trials.
- Fall-risk labels enter quickly and exit slowly.
- Before/after recordings and metrics are saved.
- Any change to filtered classifier input is validated or followed by retraining.

---

# References for the Agent

- Project README in this repository: documents the existing DataLogger, training pipeline, feature order, and classification location.
- InvenSense / TDK MPU-6000 and MPU-6050 Register Map and Descriptions, Register 26: DLPF configuration table.
  - https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
- Garcia-Ceja et al., "A Quantitative Comparison of Overlapping and Non-Overlapping Sliding Windows for Human Activity Recognition Using Inertial Sensors," Sensors, 2019.
  - https://www.mdpi.com/1424-8220/19/22/5026

