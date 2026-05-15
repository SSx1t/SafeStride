# SafeStride — Model Diagnosis & Retrain Plan

**Date:** 2026-05-15
**Trigger:** First end-to-end test of the SafeStride Random-Forest classifier on data captured through our own hardware (MYOSA + Y-vertical lower-back mount + StrideSafe Android app, Raw recording feature). Healthy walker, ~30 s clip → model classified every window as **STROKE-LIKE with P ≥ 0.93**.

This document records what we found, why the misclassification happened, what was changed, and what's needed to fix it properly.

---

## 1. Test setup

| Item | Value |
|---|---|
| Subject | Healthy adult (you), normal lower back mount |
| Recording | `stridesafe_20260515_135802.csv`, 3,062 rows, ~34.5 s effective |
| Sample rate observed | **88.7 Hz** (target 100 Hz) |
| App | StrideSafe → Raw tab → "Start recording" / "Export CSV" |
| Firmware | `Downloads/SafeStride/DataLogger/DataLogger.ino` (Tarek's latest) |
| Model | `Downloads/SafeStride/safestride_model.pkl` (RF, 20 trees, 18 features) |
| Trainer | `Downloads/SafeStride/final_dataset/train_model.py` |

A separate issue noted at ingest: the CSV's `t_ms` column jumped *backward* at row 48 (17371 → 111). That means a `0x20` "start recording" command fired twice — almost certainly because a Live session was already running when the Raw recording started. Mitigated in analysis by dropping the leading 48 rows; should be prevented at the app layer (don't let Raw start while Live is active, or stop Live first).

---

## 2. What the model actually said

Running the recording through the trainer's exact pipeline (10 s windows, 5 s stride, resampled to 100 Hz) and the loaded `safestride_model.pkl`:

```
Window 1  t= 0.0-10.0s  →  STROKE-LIKE  P(healthy)=0.053  P(stroke)=0.947
Window 2  t= 5.0-15.0s  →  STROKE-LIKE  P(healthy)=0.017  P(stroke)=0.983
Window 3  t=10.0-20.0s  →  STROKE-LIKE  P(healthy)=0.069  P(stroke)=0.931
Window 4  t=15.0-25.0s  →  STROKE-LIKE  P(healthy)=0.019  P(stroke)=0.981
Window 5  t=20.0-30.0s  →  STROKE-LIKE  P(healthy)=0.023  P(stroke)=0.977
```

Confidence is high in every window — this isn't a borderline call.

---

## 3. Feature comparison vs trainer's distributions

User's mean feature values vs the medians and 10–90% percentile ranges from `features.csv` (1,476 healthy windows, 8,405 stroke windows):

| Feature | USER | Healthy med [10–90] | Stroke med [10–90] | In which class? |
|---|---:|---|---|---|
| vert_mean | -0.13 | -0.39 [-1.5, -0.1] | -0.41 [-1.3, -0.1] | both |
| vert_std | 1.55 | 2.41 [1.0, 3.5] | 1.06 [0.4, 2.1] | both |
| vert_range | 8.49 | 12.4 [6.7, 19.1] | 7.26 [3.3, 13.1] | both |
| vert_skew | 0.37 | 0.56 [0.2, 1.5] | 1.03 [0.1, 2.0] | both |
| vert_kurt | -0.10 | -0.08 [-0.8, 3.5] | 2.05 [0.1, 7.8] | **only Healthy** |
| ml_std | 1.52 | 1.57 [0.8, 2.4] | 0.89 [0.5, 1.6] | both |
| ml_rms | 1.55 | 1.69 [1.0, 2.5] | 1.12 [0.6, 1.9] | both |
| ml_range | 11.46 | 11.5 [6.1, 19.7] | 6.55 [3.1, 13.2] | both |
| yaw_std | 23.6 | 43.9 [18.6, 58.1] | 18.4 [9.1, 37.6] | both |
| yaw_abs_mean | 19.2 | 33.0 [14.2, 46.3] | 15.2 [6.8, 29.3] | both |
| step_count | 8 | 17 [0, 20] | 3 [0, 17] | both |
| **cadence_spm** | **48.0** | 105 [0, 120] | 24 [0, 107] | both |
| step_interval_cv | 0.37 | 0.22 [0, 0.7] | 0.01 [0, 0.5] | both |
| **step_ac** | **0.31** | 0.75 [0.5, 0.8] | 0.45 [0.1, 0.7] | **only Stroke** |
| **stride_ac** | **0.15** | 0.68 [0.4, 0.8] | 0.38 [-0.0, 0.7] | **only Stroke** |
| gsi | 1.52 | 1.12 [1.0, 1.3] | 1.04 [0.0, 1.6] | only Stroke |
| jerk_std | 0.65 | 0.80 [0.3, 1.3] | 0.37 [0.1, 0.9] | both |
| jerk_max | 4.33 | 3.75 [1.8, 7.6] | 2.18 [0.9, 5.2] | both |

**15 of 18 features land in the healthy 10–90% range.** The model still picks STROKE because the three rhythm features (`step_ac`, `stride_ac`, plus low cadence) are stroke-side, AND the model's most important features happen to point that way. Specifically:

| Feature | Importance | User vs Healthy/Stroke |
|---|---:|---|
| **yaw_std** | 24% | low (23.6) — closer to stroke |
| **step_ac** | 12% | low (0.31) — only stroke range |
| vert_kurt | 8% | -0.10 — healthy |
| vert_std | 8% | 1.55 — both |
| yaw_abs_mean | 7% | 19.2 — closer to stroke |
| cadence_spm | 5% | 48 — closer to stroke |

The two highest-importance features point stroke; together that's enough.

---

## 4. Root causes

### 4.1 Peak-detection threshold is calibrated for Felius hardware, not ours

`train_model.py` and the firmware feature extractor both use:

```python
peaks, _ = signal.find_peaks(vert, height=0.35 * G, distance=25)
                                   #     ↑ 3.43 m/s²
```

Our recording's vertical signal has clear walking peaks at ~87 spm (FFT confirms — dominant freq 1.42 Hz). But because the chip's hardware DLPF=4 (~21 Hz cutoff) plus our resampling soften the signal, only **1.7 %** of samples cross 3.43 m/s². The peak detector misses most footstrikes and reports cadence as **48 spm**.

Threshold sensitivity on the same recording:

| Height threshold | Detected cadence |
|---|---|
| 0.20 G (1.96 m/s²) | **91 spm** ← matches FFT |
| 0.25 G (2.45 m/s²) | 78–91 spm |
| 0.30 G (2.94 m/s²) | 72–91 spm |
| **0.35 G ← trainer's choice** | **52 spm** ← what the model sees |
| 0.40 G | 14 spm (broken) |

The Felius dataset was collected on different hardware with a less aggressive filter chain. Their footstrikes register ~50 % larger than ours after our pipeline. This single bug explains the bad `cadence_spm`, partly explains the bad `step_ac`/`stride_ac` (peak-finding indirectly contributes), and definitely explains the bad `step_count`.

### 4.2 Top-importance feature `yaw_std` (24 %) is biased by walk geometry

Felius healthy walks include lap turns; their healthy `yaw_std` median is 43.9 °/s. Our test walk was straight-line: 23.6 °/s. The model's most important feature is therefore reading "low rotation = stroke-like" — not because the gait is pathological, but because of the walking course geometry.

### 4.3 Class-manifold imbalance

`features.csv` is **1,476 healthy vs 8,405 stroke windows** (5.7× more stroke). The trainer correctly passes `class_weight='balanced'` (line 206), so the *count* imbalance is handled — but the model has still seen 5.7× more *variations* of stroke walking than healthy walking. The boundary is conservative: ambiguous cases get pushed to stroke.

### 4.4 Sample-rate drift (88.7 Hz vs 100 Hz)

Firmware sets `SAMPLE_PERIOD_MS = 10` (100 Hz). Recording came in at 88.7 Hz. Causes investigated:

- Hardware DLPF and IMU read times — fine, well under 10 ms.
- BLE notification cadence — most likely culprit; under load some samples are getting dropped or coalesced.
- **Serial UART back-pressure** — suspected. Even at 921600 baud, snprintf + Serial.println on every sample takes meaningful time and contends with BLE. Confirmed by simply removing it (see §5).

### 4.5 Filter-chain mismatch (note, not yet fixed)

`STREAM_FILTERED_VALUES` is currently `false` in firmware, so the Android side gets bias-corrected raw samples — good. However, the chip's MPU6050 hardware DLPF=4 (~21 Hz cutoff) is more aggressive than what the Felius dataset likely used. Worth re-checking after retraining.

---

## 5. Code changes made this session

### 5.1 Firmware: `Downloads/SafeStride/DataLogger/DataLogger.ino`

Added a `SERIAL_DEBUG` master switch. With it `false` (default), the compiler dead-code-eliminates every `Serial.*` call, skips `Serial.begin` and the 1.5 s settle-delay at boot, and the recording hot path skips snprintf entirely when no consumer is listening.

```cpp
// Master Serial-Monitor switch. False (default) = no Serial.begin, no UART
// writes, no snprintf done purely for printing — entire path is dead-code-
// eliminated by the compiler. Flip to true only when debugging over USB.
static constexpr bool SERIAL_DEBUG = false;
```

Sites updated:
- `setup()` — Serial.begin gated; 1.5 s delay also skipped
- 8 `Serial.println` / `print` sites for calibration, recording start/stop, MPU not-found, ML feature dump
- Recording loop hot path now opens with `if (!deviceConnected && !SERIAL_DEBUG) return;` so we don't even snprintf when nobody's listening

To re-enable for USB debugging: flip the one constant to `true` and re-flash.

### 5.2 Android app: `RawDataViewModel.kt`

Pre-sized the recording buffer so a 20-min capture never triggers an ArrayList resize/copy:

```kotlin
private val recordedSamples = ArrayList<SensorSample>(150_000)
```

Was `8_192`. 100 Hz × 20 min = 120,000 samples; 150k pre-allocation covers up to ~25 min with no reallocations.

---

## 6. Recording protocol for retraining

20 min healthy walk, captured through StrideSafe → Raw tab.

**Setup:**
- ESP re-flashed with `SERIAL_DEBUG = false` firmware.
- Phone awake throughout (disable auto-lock, or hold the phone). BLE notifications can pause when Android dozes.
- Open StrideSafe, go straight to **Raw tab**. Do **not** start a Live session first — that caused the duplicate-`0x20` artifact in the test recording.

**Walking content** (mix is critical — see §4.2):
- ~10 min natural pace, straight walking
- ~4 min slower pace
- ~4 min slightly brisker pace
- ~2 min figure-8s / U-turns / corners (so `yaw_std` distribution covers both straight and turning)

**Verify after Stop:**
- Row counter on the Raw tab card should be **~120,000** for 20 min at 100 Hz. If it's significantly lower (~106,000 ≈ 88 Hz), the BLE transport is still dropping samples and we should investigate before retraining.
- Hit "Export CSV" and share the file.

---

## 7. Retraining plan (after the recording)

### 7.1 What retraining solves
- Cadence threshold mismatch — model re-learns the actual feature distribution from our pipeline.
- Filter-chain attenuation — same.
- yaw_std straight/curve bias — *if* the recording includes both.
- Manifold imbalance — partly (more healthy samples).

### 7.2 What retraining does NOT solve
- **88 Hz transport issue**: that's a BLE/firmware problem. Verify the new recording is at 100 Hz before training.
- **No stroke-pattern data on our hardware**: this is the bigger problem. If we naively retrain with our healthy + Felius stroke, the classifier may just learn "is this from our hardware?" — using the hardware/filter signature as a free shortcut. Cross-subject validation on real patients would then collapse.

### 7.3 Three retraining paths (best → quickest)

1. **Best — collect simulated stroke-pattern data on our hardware.** Have a healthy person walk with a limp / dragged foot / hesitation / shuffling — even a few minutes from 1–2 subjects. Combined with our healthy + Felius stroke, the model has to learn *gait pattern*, not hardware origin.
2. **Acceptable — augment Felius stroke through our filter chain in software** before retraining, so the hardware signature isn't a free feature. Then add our healthy data.
3. **Quick win for testing only** — train a binary "healthy / not-walking" detector on our data alone. Won't classify stroke, but will validate that the pipeline can produce a confident HEALTHY call.

### 7.4 Recommended order

1. Verify new 20-min recording is at ≥99 Hz native rate. If not, stop and debug BLE.
2. Lower the peak detector threshold from `0.35*G` to `0.20*G` in `train_model.py` AND the firmware feature extractor (must match). Recompute features for both Felius and our new data through the same pipeline.
3. Retrain RF on (our healthy + Felius stroke), `class_weight='balanced'`, `GroupKFold` by subject. Hold out the user's recording as a smoke test — should classify HEALTHY with high confidence.
4. If §7.3-1 data becomes available, fold it in and re-validate.

---

## 8. File reference

| File | Role |
|---|---|
| `final_dataset/train_model.py` | Trainer pipeline (resample, window, features, RF fit) |
| `final_dataset/features.csv` | Pre-extracted feature table (1476 H + 8405 S windows) |
| `safestride_model.pkl` | Trained RandomForest (20 trees, depth 9, min_leaf 10, balanced) |
| `feature_order.txt` | Canonical 18-feature order — firmware must match exactly |
| `DataLogger/DataLogger.ino` | ESP32 firmware (now SERIAL_DEBUG-gated) |
| `DataLogger/gait_model.h` | C++ port of the RF (m2cgen output) for on-device inference |
| `~/AndroidStudioProjects/StrideSafe/` | Phone app (Raw tab does the 20-min capture) |
