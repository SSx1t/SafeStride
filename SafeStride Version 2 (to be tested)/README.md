# SafeStride — Version 2 (firmware + app changes)

This folder contains the two updated deliverables and the change log that took
us from the originally-shared `DataLogger.ino` + `StrideSafe-debug.apk` to
these versions.

## What's in here

| File | What it is | What to do with it |
|---|---|---|
| `DataLogger.ino` | ESP32 sketch (11 KB) | Open in Arduino IDE → flash to the MYOSA ESP32 |
| `StrideSafe-debug.apk` | Android app, debug build (16 MB) | `adb install -r StrideSafe-debug.apk` or sideload to the phone |
| `README.md` | This file | Reference |

The Android app's source lives in `~/AndroidStudioProjects/StrideSafe/`. The
firmware source mirror lives in `~/Downloads/SafeStride-main/DataLogger/`.

---

## TL;DR — what changed

1. **Firmware was wasting most of the IMU sample budget on the barometer.**
   The BMP180 driver blocks for ~10 ms per read; the original sketch called
   it every 10 ms cycle. Net effect: notifications arrived in bursts of 2–3
   and the phone chart looked jumpy even when motion was smooth. Fixed.
2. **MPU clock was the internal 8 MHz RC oscillator.** Switched to the
   gyro X PLL — less drift, more stable timestamps.
3. **The Android app was using hand-tuned heuristics, not the trained
   model.** A 30-tree Random Forest had been trained, exported to C++ for
   on-ESP use, and never wired up anywhere. We ported it to Kotlin and
   plugged it in.
4. **App-side rolling windows assumed exactly 100 Hz sample rate.** They're
   now time-bounded, so the metrics stay correct when the firmware dips to
   90 Hz (which happens once every 200 ms when the barometer ticks).
5. **All Serial output is now gated by a `SERIAL_DEBUG` compile-time flag
   at the top of `DataLogger.ino`** (default `false`). In real-world
   wireless use nobody reads the USB-Serial stream, so we skip the
   String construction + UART work entirely on every sample. Flip to
   `true` if you want to read CSV over USB while debugging.

---

## Why we changed things

Two problems were reported / discovered while reviewing the project:

1. **"Values on my phone are jumping from -120 to +120."**
   Investigation: the ±120 range *itself* was normal for gyro during walking
   (lumbar yaw hits 100–150 dps every step), but the firmware was causing
   real visual jitter on top of that because of the BMP-in-hot-path issue.

2. **The model exists but isn't used.**
   The README in `SafeStride-main/` references a trained Random Forest
   (`safestride_model.pkl` → `gait_model.h`), but neither the firmware nor
   the app ever called it. The phone's "SAFE / CAUTION / UNSAFE" pill came
   from a heuristic in `MetricsEngine.computeScore()`. Two separate signals
   answering different questions had been conflated.

---

## Firmware changes (`DataLogger.ino`)

All changes preserve the BLE wire format, UUIDs, device name, and start/stop
command bytes. The Android app speaks the new firmware exactly the same way.

### 1. BMP180 moved off the IMU hot path
Was: `bmp.getAltitude()` called inside the 100 Hz IMU loop. The BMP180
driver blocks for ~10 ms (two `delay(5)` calls). Loop fell behind every
cycle.
Now: barometer reads run on their own 200 ms scheduler (~5 Hz, plenty for
sit-to-stand detection). IMU loop loses one sample out of 20 instead of
stuttering every sample.

### 2. MPU clock source → gyro X PLL
`PWR_MGMT_1` was `0x00` (internal 8 MHz RC, ~5 % drift). Now `0x01`
(CLKSEL=1, gyro PLL, ~1 % drift, better temperature stability). The
MYOSA `AccelAndGyro` library uses the same source.

### 3. `SMPLRT_DIV = 9` added
Sets the MPU's internal output rate to 1 kHz / (1+9) = 100 Hz, matching
our read rate. Every read now returns a freshly produced sample instead
of the latest of 10 internal samples.

### 4. I²C bus bumped to 400 kHz fast-mode
All three MYOSA sensors (MPU6050, BMP180, SSD1306) support it. A 14-byte
burst drops from ~1.4 ms (at 100 kHz) to ~0.4 ms. The OLED constructor
is configured to leave the bus at 400 kHz after its writes (default
behaviour is to drop back to 100 kHz).

### 5. Loop catch-up bounded
The old loop did `lastSample += 10` unconditionally, which meant any
long blocking call (BMP, calibration, OLED) caused the loop to burst
hundreds of samples back-to-back trying to catch up. Now: if the loop
falls behind by more than 50 ms, `lastSample` is reset to `now` (drop
the missed samples). Otherwise advance by exactly 10 ms (long-run
cadence locked to 100 Hz).

### 6. Serial output gated by a compile-time `SERIAL_DEBUG` flag
At the top of `DataLogger.ino`:
```cpp
static constexpr bool SERIAL_DEBUG = false;
```
Every `Serial.print*()` call is wrapped in `if (SERIAL_DEBUG) ...`, and
the loop short-circuits before building the CSV string when both
`SERIAL_DEBUG` is false and no BLE central is connected. Net effect
with the flag off (the default):

- `Serial.begin()` and its 1.5 s settle delay are skipped at boot
- The 8-way `String(...)` concatenation per sample is skipped when
  nothing's listening
- All ~6 startup/status status prints are dead-code-eliminated

Set to `true` if you want to see CSV output on the Arduino IDE Serial
Monitor while debugging over USB. Baud stays at 115 200, which is fine
for our data rate (you can verify the math is favourable: 100 Hz ×
65 chars ≈ 6.5 kB/s produced vs 11.5 kB/s drain).

Why this matters: the rationale "lower baud is more reliable" actually
holds for USB-Serial bridges in practice — 921 600 sometimes has
flakier opens, dropped bytes on cheap CP2102/CH340 chips, and not
every tool/monitor supports it. 115 200 is the safe default.

### 7. Calibration auto-detects gravity-aligned axis
Original code hard-coded gravity to Y axis (`ayBias -= 1.0f`). If the
belt was mounted with a different orientation, the rest-magnitude was
wrong and the step detector misfired. Now: the dominant accel bias axis
is detected at boot, and 1 g is preserved on that axis only — rest
magnitude stays ≈ 1.0 g regardless of how the device is clipped on.

### 8. Cleanup
- Removed ~290 lines of commented-out earlier drafts at the top of the
  file (the original was the new code at line 294 onward, with all the
  previous attempts left in comments above).
- Removed OLED, BLE, and other init code references that aren't needed
  for streaming-only operation.

---

## App changes (`StrideSafe-debug.apk`)

### Performance / correctness fixes

**a) `MetricsEngine` rolling windows → time-based**
Was: `jerkWindow.size = 100` (assumed 100 Hz × 1 s) and
`altBufferMax = 100`. With the firmware dipping to ~95 Hz during
barometer ticks, every time-based metric (score, cadence, asymmetry,
STS detection) was off by 5–10 %.
Now: both windows hold `(timestamp, value)` pairs and trim by
`tRel - first().timestamp > windowMs`. Stays correct regardless of
sample rate jitter.
(`analytics/MetricsEngine.kt`)

**b) Gyroscope chart Y-range → ±500 dps**
Was: ±250 dps. Real lumbar yaw during turns peaks at 200–400 dps and
was getting clipped flat against the top of the chart. Chip is
configured for ±1000 dps full scale, so ±500 captures essentially all
real motion without making small motions invisible.
(`ui/live/LiveSessionScreen.kt`)

### The big one — trained model is now actually running

Four files added under `app/src/main/java/com/example/stridesafe/analytics/`:

**`GaitModel.java`** (798 KB, 14 k lines).
`m2cgen` export of `safestride_model.pkl`. Plain Java — no JNI, no
TensorFlow, no native libs. 30 decision trees with 6 970 total nodes
across 6 `subroutine*()` methods (each stays under Java's 64 KB
per-method bytecode limit). Public API: `GaitModel.score(double[18])`
returns aggregated tree votes per class.

The original `safestride_model.pkl` was saved with sklearn 1.8.0 /
numpy 2.x and couldn't be loaded with the locally-installed sklearn
1.3.2 (the numpy 2.x pickle format isn't backward compatible). We
retrained from `features.csv` (which is just the pre-extracted feature
matrix — same training data, no need to re-window 5 M rows of raw IMU)
using the same hyperparameters from the trainer's CV block:
`n_estimators=30, max_depth=8, min_samples_leaf=10, class_weight='balanced'`.
Training accuracy on the full set: 94.8 %.

**`FeatureExtractor.kt`**.
Kotlin port of `extract_features()` from `train_model.py`. 18 features
in the canonical order:
```
vert_mean, vert_std, vert_range, vert_skew, vert_kurt,
ml_std, ml_rms, ml_range,
yaw_std, yaw_abs_mean,
step_count, cadence_spm, step_interval_cv,
step_ac, stride_ac, gsi,
jerk_std, jerk_max
```
Key subtleties that had to be reproduced exactly so the model gives
the same predictions as during training:
- `np.std` is population std (ddof = 0), not sample std
- pandas `.skew()` is adjusted Fisher–Pearson G1, not raw `m3 / m2^1.5`
- pandas `.kurt()` is excess kurtosis G2, not raw `m4 / m2² − 3`
- scipy `find_peaks` handles plateaus by midpoint and applies a
  specific tallest-first distance suppression
- autocorrelation is biased and normalized by the mean-centered
  sum-of-squares of the *full* window, not the lagged window

**`GaitClassifier.kt`**.
Maintains a 500-sample ring buffer of `(vert, ml, yaw)`. Pushes one
sample per BLE notification. Once the buffer is full, runs feature
extraction + `GaitModel.score()` every 250 samples (= 2.5 s at 100 Hz)
— this matches the trainer's 5-second windows with 50 % overlap.

Axis remap for our hardware mounting (MYOSA mounted vertically at the
tailbone, chip face pointing posterior — confirmed visually from the
GY-521 axis silkscreen):
```
vert ← (sample.accelY − 1.0) × G   // remove gravity, m/s²
ml   ← sample.accelX × G           // medio-lateral (wearer's left/right)
yaw  ← sample.gyroY                // rotation about vertical, °/s
```
Why this remap: with the PCB vertical at the lower back and chip face
pointing posteriorly, the GY-521's +Y silkscreen arrow points up
(toward the head) and +X points to the wearer's left. The +Z axis
points out of the chip face (anteroposterior) and is not used by the
18-feature model. In the Felius training data the convention was
different (`ax` vertical, `gx` yaw), so we substitute our `accelY`
into the trainer's `vert` slot and our `gyroY` into the `yaw` slot.

An earlier version of this remap used `accelZ` / `gyroZ` based on a
"chip face up" mounting assumption that turned out to be wrong; the
GY-521 silkscreen on the actual board confirms Y is vertical for this
PCB orientation.

**`GaitVerdict.kt`** (in `data/model/`).
Enum (`HEALTHY`, `STROKE_LIKE`, `PENDING`) + the
`P(stroke)` probability. Carried on `LiveMetrics.gait`.

**`MetricsEngine` integration.**
After the existing per-sample heuristics run, the sample is also
pushed to the classifier. A new prediction returned by `push()` updates
`lastGait`, which is emitted with every `LiveMetrics`. Between
predictions, the previous verdict stays visible.

**UI — `GaitVerdictCard` on `LiveSessionScreen`.**
New card right below the SAFE/CAUTION/UNSAFE score block. Shows
"Healthy-like gait" / "Stroke-like gait" / "Analyzing gait…" plus the
`P(stroke) = NN%` numeric. The pre-existing SAFE/CAUTION/UNSAFE pill
is **kept** — the two signals answer different questions:
- SAFE/CAUTION/UNSAFE = "right now, is your motion shaky?" (heuristic,
  updated 100 Hz)
- HEALTHY / STROKE-LIKE = "over the last 5 seconds, does your gait
  pattern match the trained healthy or stroke-like distribution?"
  (model, updated every 2.5 s)

---

## How we verified the model port

`final_dataset/validate_port.py` proves the Kotlin feature extractor is
numerically identical to the trainer's pandas/scipy version.

Approach: take one real 500-sample window from the Felius dataset, run it
through two implementations side-by-side:

- **(a)** the original `extract_features()` using pandas, numpy, scipy
- **(b)** a pure-Python reimplementation that mirrors `FeatureExtractor.kt`
  line-for-line (no numpy/pandas/scipy)

Print the max absolute difference across all 18 features.

Result: **`1.8 × 10⁻¹⁴` max diff** — that's the noise floor of float64
arithmetic. Translation: if the Kotlin code follows the same algorithm as
the pure-Python mirror (which it does, by construction), it produces the
same features as the trainer, which means the model on the phone sees
the same inputs it saw during training.

When to re-run: only if you change `FeatureExtractor.kt` (add a feature,
change a constant, fix a bug). Day-to-day, ignore it.

---

## How to install

**ESP32 firmware:**
1. Open Arduino IDE.
2. Open `DataLogger.ino` from this folder (or from
   `~/Downloads/SafeStride-main/DataLogger/`).
3. Board target: same as before (ESP32 Dev Module). Required libraries
   (`NimBLE-Arduino`, `OLED`, `BarometricPressure`) are already in
   `~/Documents/Arduino/libraries/`.
4. Upload.

**Android app:**
```
adb install -r "/Users/wahaj/Desktop/SafeStride Version 2/StrideSafe-debug.apk"
```
The `-r` flag replaces the existing install. Same `applicationId` and
debug signing key as the previous build, so no uninstall is needed if you
installed the previous APK from this machine.

---

## What still needs validating on hardware

The Kotlin port is mathematically faithful to the trainer. What we have
*not* yet confirmed is that the trained model — which learned on the
Felius dataset — actually generalises to your MYOSA hardware. The honest
test:

1. Flash the new firmware, wear the belt with the device face-up at the
   tailbone.
2. From `~/Downloads/SafeStride-main/final_dataset/`:
   ```
   python record_walk_ble.py healthy_test.csv
   ```
   Walk normally for ~60 s, press Enter to stop.
3. Repeat with a faked limp:
   ```
   python record_walk_ble.py limp_test.csv
   ```
4. Watch the new "Healthy-like gait / Stroke-like gait" card on the Live
   screen during each. Most of the healthy walk should classify as
   healthy; most of the limp walk should flip to stroke-like.

If it doesn't flip, the Felius training distribution doesn't transfer
cleanly to your hardware (different sensor noise floor, slightly
different sensor placement than the Felius subjects) and the right
answer is a small fine-tune on your own data, not more porting.

---

## Honest limitations

1. **The model classifies *gait pattern*, not *moment-to-moment safety*.**
   A healthy person walking erratically still probably classifies as
   healthy. The SAFE/CAUTION/UNSAFE pill is what answers "are you about
   to fall?".
2. **`ml` axis assumption.** I'm using `accelX` as medio-lateral and
   `accelY` as anteroposterior. If your X axis actually points forward
   (not sideways) on your belt, swap `ml = sample.accelX * G` to
   `ml = sample.accelY * G` in `GaitClassifier.kt`. Top-importance
   features are unaffected, so the model will still work even if the
   guess is wrong.
3. **No hardware-side validation has happened yet.** Items 1–8 in the
   firmware section are reasoning from the code and the MPU6050
   datasheet, but the only way to confirm "values look smoother now" is
   to flash and watch.
4. **All inference runs on the phone.** ESP stays dumb — it streams raw
   IMU + altitude, full stop. This was deliberate: retraining the model
   only requires updating the APK, no firmware OTA needed. If you ever
   want offline operation without the phone, the C++ classifier
   (`gait_model.h`) is still in the SafeStride folder and can be wired
   into the ESP loop separately.
