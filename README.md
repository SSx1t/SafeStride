# SafeStride - MYOSA 2026 (ONLY FOR DEVELOPMENT CURRENTLY)

The original (without removing irrelevant data) dataset used is taken from Zenodo which can be found [here](https://zenodo.org/records/11045239)

The pruned dataset (which focused on taking only data that matched our specifications) is uploaded to KU's OneDrive which can be found [here](https://kuacae-my.sharepoint.com/:x:/g/personal/100068517_ku_ac_ae/IQB86T1_jdhzRJL9dh7bPrMPAdoFInUusAvuch9Xyi3RMRI?e=LDc60Y)

---

## Quick map

```
SafeStride_Assets/
├── DataLogger/                  ← ESP32 firmware (older, single-file)
│   └── DataLogger.ino
├── SafeStride/                  ← ESP32 firmware (current direction)
│   ├── SafeStride.ino           (currently a diagnostic sketch — see notes)
│   ├── gait_model.h             (~140 KB — auto-generated RF classifier in C++)
│   ├── config.h, gait.h, sensors.h, sts.h, turn.h   (empty placeholders)
└── final_dataset/               ← Training data + Python pipeline + model
    ├── combined_lower_back.csv  (401 MB — pre-merged Felius dataset)
    ├── healthy/                 (59 CSVs — healthy walkers, lower-back IMU)
    ├── stroke/                  (320 CSVs — stroke patients, lower-back IMU)
    ├── calibration/             (10 CSVs — our own recordings for testing)
    ├── train_model.py           (feature extraction + Random Forest training)
    ├── convert_to_cpp.py        (.pkl → gait_model.h for the ESP32)
    ├── check_recording.py       (sanity-check a single CSV recording)
    ├── scan_ble.py              (find the ESP32 over BLE)
    ├── record_walk_ble.py       (record a walk to CSV over BLE)
    ├── safestride_model.pkl     (the trained sklearn classifier)
    ├── features.csv             (20k feature rows extracted from the dataset)
    ├── feature_order.txt        (canonical 18-feature order)
    └── feature_importance.png   (which features matter most)
```

---

## File-by-file reference

Every file in the folder, in the order you'd encounter them, with what's
inside and what it does. Skip to the section for the file you actually
opened.

### DataLogger/

#### `DataLogger.ino` *(~17 KB, ~600 lines total, ~225 active)*

The currently-working production firmware for the MYOSA ESP32 wearable.
It boots, calibrates, advertises over BLE as `SafeStride-Logger`, waits
for an `0x20` command from the host, and streams IMU+altitude CSV
samples at 100 Hz. The first ~380 lines of the file are commented-out
earlier drafts of the same code — the live code is at the bottom.

Key functions inside:

- `probeMPU(addr)` — checks WHO_AM_I to confirm the MPU6050 is present
  at a given I²C address.
- `wakeMPU(addr)` — clears the sleep bit, configures the gyro range to
  ±1000 °/s, accel to ±2 g, and DLPF to 44 Hz.
- `readMPU(...)` — single burst read of 14 bytes from register 0x3B,
  scaled into g and °/s. **This is the fix for the torn-read bug.**
- `calibrateMPU()` — averages 300 stationary samples to compute biases
  for all 6 axes; subtracts 1.0 from `ay` (the gravity axis) so the
  zero-motion baseline is true zero.
- `setupBLE()` — NimBLE init, service + 2 chars (cmd, log), advertising
  with the service UUID in the scan response.
- `loop()` — fixed-rate 10 ms cadence, reads MPU, applies biases, reads
  altitude, builds the CSV string, prints to Serial **and** sends as a
  BLE notification.

This is what was used to capture every CSV in `calibration/`.

### SafeStride/

#### `SafeStride.ino` *(~8 KB)*

**Currently a diagnostic tool, not production firmware.** Streams accel
+ altitude readings to USB Serial at 10 Hz, prints status tags like
`[OK]` / `[ACCEL STUCK ZERO]` / `[ALT DEAD]`. Built to confirm whether
clipped pins killed the MPU6050. No BLE in this version. The file was
intended to become the modular replacement for `DataLogger.ino` but
has not been wired up yet.

#### `gait_model.h` *(~140 KB)*

The trained Random Forest classifier exported to C++ source by
`micromlgen`. Defines:

```cpp
namespace Eloquent::ML::Port {
  class GaitClassifier {
    public: int predict(float *x);  // returns 0=healthy, 1=stroke
  };
}
```

`predict()` is a nested `if`/`else` tree over the 18-element feature
vector. The vector must be built in the exact order from
`feature_order.txt`. Auto-generated — don't edit by hand; regenerate
with `convert_to_cpp.py`.

#### `config.h`, `gait.h`, `sensors.h`, `sts.h`, `turn.h` *(0 bytes each)*

Empty placeholders. The intent was to split `DataLogger.ino` into modules
(sensor reading, gait analysis, sit-to-stand detection, turn detection,
shared config), but the split hasn't been done. Safe to ignore until
someone fills them in.

### final_dataset/

#### Training data

##### `combined_lower_back.csv` *(401 MB, ~5.27 M rows)*

The Felius lower-back IMU dataset pre-merged into a single file with
two extra columns (`subject`, `status`). This is what `train_model.py`
loads directly. Schema:
`index, T, ax, ay, az, gx, gy, gz, subject, status`.
Sample rate ~104 Hz, accel in g, gyro in °/s.

##### `healthy/` *(55 MB, 59 CSVs)*

The healthy subset of the Felius dataset, **un-merged**, one file per
recording. Filenames encode the protocol: `H##_Healthy_TestRetest_Test_<Yes|No>_lowback.csv`
or `..._Hertest_...` for the retest sessions. Each file is ~15k rows
(~2½ minutes at 104 Hz). Schema: `index, T, ax, ay, az, gx, gy, gz`.
**Not used directly by the training script** (which uses the combined
CSV), but kept as the source of truth.

##### `stroke/` *(267 MB, 320 CSVs)*

Same format as `healthy/`, but for stroke patients (`S###P` =
paretic side, `S###H` = healthier side). Much larger because more
subjects and more sessions per subject.

##### `calibration/` *(408 KB, 10 CSVs + 1 shortcut)*

Hardware-side recordings made with the actual MYOSA wearable via
`record_walk_ble.py`. Different schema than the Felius files —
header is `# t_ms,ax,ay,az,gx,gy,gz,alt`, includes a barometer-derived
altitude column. Five `healthy_N.csv` (real walking) and five
`limp_N.csv` (operator faking a limp for contrast). Use these to
sanity-check that hardware + firmware are producing data that
*looks like* what the model was trained on.

The `final_dataset - Shortcut.lnk` file is a leftover Windows symlink
from how the folder was copied. Harmless; ignore.

#### Python scripts

##### `scan_ble.py` *(< 1 KB)*

Tiny utility built on `bleak`. Runs a 15-second BLE discover scan and
prints every device found. Used once to find the ESP32's MAC address;
not part of the regular workflow.

##### `record_walk_ble.py` *(~3 KB)*

Connects to the ESP32 at a hard-coded MAC (`DEVICE_ADDRESS` near the
top), checks both BLE characteristics exist, subscribes to notifications,
waits for Enter, sends `0x20` (start), streams every NOTIFY packet into
the output CSV. Press Enter again to send `0x21` (stop). Writes a
`# RECORDING STOPPED. Samples: N` footer line.

Used like: `python record_walk_ble.py calibration/healthy_6.csv`

##### `check_recording.py` *(~5 KB)*

Quality gate for a single CSV. Loads the file, removes the
`t_ms = 4294967291` overflow rows, uses `ay` as the vertical axis
(matches the MYOSA mounting), low-pass filters at 5 Hz, runs
`scipy.signal.find_peaks` to count steps. Prints duration, sample
rate, accel magnitude stats, step count, cadence. Then runs six
pass/fail checks (sampling rate, timing gaps, accel sanity, gravity
magnitude, cadence range, step count) and prints `✓ GOOD` or
`⚠ NOT PERFECT` with specific complaints.

Usage: `python check_recording.py calibration/healthy_1.csv`

##### `train_model.py` *(~10 KB)*

The full training pipeline:

1. Loads `combined_lower_back.csv` (path hardcoded to
   `~/Downloads/final_dataset/...` — edit if your folder is elsewhere).
2. Per subject: resamples 104 Hz → 100 Hz to match the MYOSA hardware
   rate, computes body-frame variables (`vert = (ax - 1.0) * G`,
   `ml = ay * G`, `yaw = gx`).
3. Slides a 5 s window with 50 % overlap, extracts the 18 features
   defined in `extract_features()` (see Glossary at the end).
4. Writes `features.csv` and `feature_order.txt`.
5. Runs Group 5-fold cross-validation **by subject** (so no subject
   appears in both train and test). Prints per-fold accuracy and AUC.
6. Trains a final model on all data, prints a training-set sanity
   check (NOT validation), saves `safestride_model.pkl`, dumps
   `feature_importance.png`.

⚠ The CV block uses `n_estimators=30, max_depth=8, min_samples_leaf=10`,
but the final model uses `n_estimators=10, max_depth=6, min_samples_leaf=5`.
Decide which you want before reporting a number.

##### `convert_to_cpp.py` *(~600 B)*

Three-step conversion: `joblib.load('safestride_model.pkl')` →
`micromlgen.port(clf, classname='GaitClassifier')` → write
`gait_model.h` with header guards. Run this after every retrain if
the ESP32 is going to run the classifier on-device.

#### Model artifacts

##### `safestride_model.pkl` *(~87 KB)*

The trained sklearn `RandomForestClassifier`. Loadable with
`joblib.load()`. Pickled, so the sklearn version that opens it has to
roughly match the version that saved it.

##### `features.csv` *(~6 MB, 20 015 rows)*

Every 5-second window's feature vector, with `label` (0 = healthy,
1 = stroke) and `subject` columns appended. Useful for re-training
without re-extracting (the windowing step is the slow part).

##### `feature_order.txt` *(< 1 KB)*

Plain text, one feature name per line, in the canonical order. The
firmware feature builder, the C++ port, and the trainer all index by
position into this list. **If you reorder it, everything breaks
silently.**

##### `feature_importance.png` *(~35 KB)*

A horizontal bar chart of the top 15 features by Gini importance.
Quick visual: `vert_std`, `step_ac`, `vert_kurt`, `yaw_std`,
`cadence_spm` dominate; `step_count` and `ml_std` matter least.

##### `gait_model.h` *(~140 KB)*

Identical to the copy in `SafeStride/gait_model.h`. This is the
*build product* of `convert_to_cpp.py`; the SafeStride copy is just
where the Arduino IDE expects it.

---

## The pipeline at a glance

```
Felius dataset (CSV)
    ↓  train_model.py     (extract 18 features, 5-fold subject-held-out CV)
safestride_model.pkl      (the trained Random Forest)
    ↓  convert_to_cpp.py  (micromlgen port to C++)
gait_model.h              (drop into SafeStride/ and #include it)
    ↓  Arduino compile
ESP32 wearable
    ↓  BLE notify
Android app or record_walk_ble.py
    ↓  check_recording.py
verify the recording looks right
```

The **Android app** (referenced in `ESP_FIRMWARE_INTEGRATION.md`, not part of
this folder) does *all* classification on the phone side from raw IMU
samples — the on-device model in `gait_model.h` is currently unused by the
production app. If you go the other direction (run the model on the ESP),
keep `train_model.py` and `convert_to_cpp.py` in sync.

---

## DataLogger/

**Status:** working firmware, older.

`DataLogger.ino` is the version that has actually been used to record the
calibration CSVs (`final_dataset/calibration/`). It:

- Probes the MPU6050 at both `0x68` and `0x69` (in case the AD0 pin floats)
- Wakes the chip and sets gyro to ±1000 °/s, accel to ±2 g
- Runs a 300-sample stationary calibration to compute biases
- Burst-reads 14 bytes from the MPU in one I²C transaction (no torn reads)
- Advertises over **NimBLE** as `SafeStride-Logger`
- Service `4f5b1a00-...`, Cmd char `...01` (0x20 start / 0x21 stop),
  Log char `...04` (NOTIFY, one CSV line per sample)
- Streams at 100 Hz
- ~225 lines of active code, plus a large block of commented-out earlier
  drafts above it

**Quirks to know about:**

- CSV is `t_ms, ax, ay, az, gx, gy, gz, alt` with ax–az in **g** and gx–gz
  in **°/s** (divided by 32.8 — the LSB scale for ±1000 °/s range)
- Altitude is computed against `baselinePressure` captured at boot, so
  values are **relative** to wherever the device started, not absolute
- Uses `NimBLEDevice` (not the stock `BLEDevice`) — make sure the NimBLE
  Arduino library is installed
- The top half of the file is the same code re-written as comments. Ignore
  it; the live code starts further down.

---

## SafeStride/

**Status:** in-progress, *not* a drop-in replacement yet.

The directory layout (`config.h`, `gait.h`, `sensors.h`, `sts.h`, `turn.h`)
was set up for a modular split — sensors module, gait module, sit-to-stand
module, turn-detection module — but **all of those files are currently
0 bytes**. The intent was clearly to move pieces of `DataLogger.ino` into
their own files.

`SafeStride.ino` currently contains a **diagnostic Serial Monitor sketch**,
not production firmware. It streams accel + altitude to USB serial at
10 Hz with status flags like `[OK]` / `[ACCEL STUCK ZERO]` / `[ALT DEAD]`.
This is what was used to confirm whether clipped pins killed the MPU6050.
**Don't flash this if you want BLE streaming** — use `DataLogger/DataLogger.ino`
for that.

`gait_model.h` is the trained Random Forest exported to C++ via
`micromlgen`. About 140 KB of `if`/`else` branches under
`Eloquent::ML::Port::GaitClassifier::predict(float *x)`. Pass an 18-element
feature vector in the order from `feature_order.txt` and it returns `0`
(healthy) or `1` (stroke). Not currently called from `SafeStride.ino`.

**If you're working on this folder:**

1. Decide whether the ESP runs the classifier or just streams. The phone
   app (`ESP_FIRMWARE_INTEGRATION.md`) currently does it on-phone, so the
   ESP only needs to stream — same as `DataLogger.ino`.
2. If you do want on-device classification, you'll need a 5-second
   rolling window, a feature extractor that exactly matches the one in
   `train_model.py` (`extract_features()` — order matters!), and a call
   to `predict()`.

---

## final_dataset/

This is where most of the project's substance lives.

### The training data

| File / folder | What it is |
|---|---|
| `combined_lower_back.csv` | 5.27 M rows. **Single pre-merged file** that the training script consumes. Same schema as the per-subject files plus `subject` and `status` columns. |
| `healthy/` | 59 individual recordings from healthy subjects (~1 MB each). Public **Felius lower-back IMU dataset** — files like `H10_Healthy_TestRetest_Hertest_Yes_lowback.csv`. |
| `stroke/` | 320 recordings from stroke subjects, same naming convention (`S###` instead of `H##`). Roughly 5× the healthy count. |

**Source rate:** 104 Hz. **Schema:** `index, T, ax, ay, az, gx, gy, gz`. Accel is
in **g**, gyro in **°/s**. Discovered by inspecting means: **`ax` is the
vertical axis** in this dataset (mean ≈ +0.94 g). That detail is hard-coded
into `train_model.py` — don't change it without re-validating.

### Our own calibration recordings

`calibration/` — ten short CSVs captured with the actual MYOSA hardware via
`record_walk_ble.py`. Filenames are `healthy_1..5.csv` and `limp_1..5.csv`
(the operator faked a limp to generate a contrasting class for testing).

These files have a slightly different header (`# t_ms,ax,ay,az,gx,gy,gz,alt`)
and include the **altitude** column the Felius set lacks. Some early ones
have a known timestamp-overflow bug on the first row — `check_recording.py`
filters that out with `df = df[df["t"] < 1e9]`.

There's also a Windows shortcut (`.lnk`) file in here — harmless leftover
from how the folder was copied. Ignore it.

### Python scripts

| Script | Purpose | Typical use |
|---|---|---|
| `scan_ble.py` | List nearby BLE devices via `bleak` | One-off, used to discover the ESP32's MAC address |
| `record_walk_ble.py` | Connect to the ESP32 at a fixed MAC, send `0x20` to start, save NOTIFY samples to CSV, send `0x21` on second Enter | `python record_walk_ble.py calibration/healthy_6.csv` |
| `train_model.py` | Full pipeline: load `combined_lower_back.csv` → resample 104→100 Hz → 5 s windows with 50% overlap → 18 features per window → Group-5-fold CV by subject → train final model → save `.pkl` + `feature_importance.png` | `python train_model.py` (~5 min) |
| `convert_to_cpp.py` | Load `safestride_model.pkl`, run `micromlgen.port()`, wrap in header guards, write `gait_model.h` | `python convert_to_cpp.py` |

**Important:** `train_model.py` hard-codes the data path to
`~/Downloads/final_dataset/combined_lower_back.csv`. Change `DATA_FILE` near
the top of the script if your folder is elsewhere.

### Model artifacts

- `safestride_model.pkl` — the trained sklearn Random Forest. The
  final model in the script uses `n_estimators=10, max_depth=6,
  min_samples_leaf=5`, though the CV block tunes higher (`n_estimators=30,
  max_depth=8, min_samples_leaf=10`). If you retrain, decide which config
  you actually want and align them.
- `features.csv` — 20,014 windowed feature rows extracted from the full
  dataset. Skip re-extracting if you just want to retrain the classifier.
- `feature_order.txt` — the **canonical 18 feature names** in the order the
  model and the C++ port expect. Any firmware that calls `predict()` must
  build the feature vector in this exact order.
- `feature_importance.png` — bar chart of which features the model relies
  on. Top 5: `vert_std`, `step_ac`, `vert_kurt`, `yaw_std`, `cadence_spm`.
- `gait_model.h` — same file as `SafeStride/gait_model.h`, kept here as the
  build product of `convert_to_cpp.py`.

---

## Common tasks

### "I want to record a new walk"

1. Plug in the MYOSA, make sure `DataLogger/DataLogger.ino` is flashed.
2. `python scan_ble.py` — confirm it shows up as `SafeStride-Logger`.
   Note the MAC address.
3. If the MAC differs from `4C:C3:82:36:80:42` in `record_walk_ble.py`,
   update `DEVICE_ADDRESS`.
4. `python record_walk_ble.py final_dataset/calibration/<FILE_NAME>.csv`
   — press Enter to start, walk, press Enter to stop.
5. A CSV file will be created inside the `calibration` folder with whatever file name you used in the previous command.

### "I want to retrain the model"

1. Make sure `combined_lower_back.csv` is reachable (or fix `DATA_FILE` in
   `train_model.py`).
2. `python train_model.py` — prints CV scores per fold, then trains the
   final model on everything and saves `safestride_model.pkl`,
   `features.csv`, `feature_importance.png`.
3. `python convert_to_cpp.py` — regenerates `gait_model.h`.
4. Copy `gait_model.h` to `SafeStride/` if firmware will use it.

### "Recording values look frozen / stuck"

The MPU6050 module's VCC/GND/SDA/SCL/AD0 pins are easy to damage on a
MYOSA daisy-chain. Flash `SafeStride/SafeStride.ino` (the diagnostic
sketch) and open the Serial Monitor at 115200 baud. It will print one of:

- `[OK]` — sensor fine, look at firmware instead
- `[ACCEL STUCK ZERO]` — chip not responding; check pins / re-seat JST
- `[ACCEL DEAD]` — chip not on the I²C bus at all
- `[ALT DEAD]` — BMP180 unreachable

If the MPU shows up at `0x69` instead of `0x68`, the AD0 pin was clipped —
firmware will still work (`DataLogger.ino` already probes both addresses).

### "I want the ESP to run the classifier"

You'll need to:

1. Implement a 5-second rolling buffer (500 samples at 100 Hz) of
   `vert_dyn`, `ml_dyn`, `yaw`.
2. Port the body-frame transform from `train_model.py`'s `process_subject`:
   `vert_dyn = (ax - 1.0) * 9.81`, `ml_dyn = ay * 9.81`, `yaw = gx`.
3. Port `extract_features()` 1:1 — same order, same definitions
   (mean/std/range/skew/kurt/RMS/autocorrelation/peak count/jerk).
4. Pass the 18-element array to `GaitClassifier::predict(x)` from
   `gait_model.h`.
5. Return 0 = healthy, 1 = stroke.

The skew/kurtosis and autocorrelation pieces are the easy ones to get
wrong — use the same lag range (30..80) and the same gravity-removal
convention as the trainer.

---

## What's missing / known issues

- `SafeStride/config.h`, `gait.h`, `sensors.h`, `sts.h`, `turn.h` are all
  **empty placeholders**. The split was planned but not executed.
- `SafeStride/SafeStride.ino` is currently a **diagnostic sketch**, not
  the production firmware. Use `DataLogger/DataLogger.ino` to collect all test data and export it as CSV. After collection of data, shift towards implementing everything in `SafeStride.ino`
- `train_model.py` final-model hyperparameters differ from the CV
  block — pick one config before reporting a number.
- `DEVICE_ADDRESS` in `record_walk_ble.py` is hard-coded to one
  particular ESP32's MAC. Change it for any other device.
- `calibration/` Files have a known bad first
  row with `t_ms = 4294967291` (millis() overflow before recording
  zeroed). Fix not implemented.
- The training-set classification report in `train_model.py` is a
  **sanity check, not validation** — it's intentionally fitting on the
  same data it trained on. The CV block above it is the real number.

---

## Glossary of the 18 features

In the order `train_model.py` and `gait_model.h` expect:

| # | Name | What it captures |
|---|---|---|
| 0 | `vert_mean` | Mean vertical accel (post gravity removal) |
| 1 | `vert_std` | Vertical accel variability — biggest single signal |
| 2 | `vert_range` | Peak-to-peak vertical accel |
| 3 | `vert_skew` | Asymmetry of vertical accel distribution |
| 4 | `vert_kurt` | Heavy-tail-ness of vertical accel (impact spikes) |
| 5 | `ml_std` | Medio-lateral (side-to-side) wobble |
| 6 | `ml_rms` | RMS of medio-lateral accel |
| 7 | `ml_range` | Peak-to-peak medio-lateral accel |
| 8 | `yaw_std` | Yaw rate variability — turning + body-twist |
| 9 | `yaw_abs_mean` | Mean magnitude of yaw rotation |
| 10 | `step_count` | Peaks detected in the window |
| 11 | `cadence_spm` | Steps per minute |
| 12 | `step_interval_cv` | Step-timing variability (rhythm consistency) |
| 13 | `step_ac` | Autocorrelation at step lag — periodicity of stepping |
| 14 | `stride_ac` | Autocorrelation at 2× step lag — left/right symmetry |
| 15 | `gsi` | Gait Symmetry Index (`step_ac / stride_ac`) |
| 16 | `jerk_std` | Variability of acceleration's derivative — smoothness |
| 17 | `jerk_max` | Peak jerk magnitude |

`vert_std`, `step_ac`, `vert_kurt`, `yaw_std`, `cadence_spm` carry most of
the discriminative weight per `feature_importance.png`.
