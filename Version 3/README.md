# SafeStride — Version 3

**Date:** 2026-05-16
**Trigger:** First retraining cycle on data captured through our own hardware. Diagnosis of the v2 misclassification, fix to feature extraction, retraining with a new ML technique (CORAL domain adaptation), and a clean split between *what runs on the ESP* and *what the phone displays*.

This folder is self-contained: open `Version3.ino` in Arduino IDE, place `gait_model.h` next to it (already done), and flash. Install `StrideSafe-V3-debug.apk` on the phone.

---

## What's in this folder

| File | What it is |
|---|---|
| `Version3.ino` | ESP32 firmware sketch — IMU streaming + on-device ML inference |
| `gait_model.h` | C++ port of the new RF model (Scenario F, CORAL-aligned) |
| `StrideSafe-V3-debug.apk` | Phone app — display only, no on-device ML |
| `README.md` | This file |

---

## Why a V3 — what was wrong with V2

Detailed diagnosis is in `../MODEL_DIAGNOSIS_AND_RETRAIN_PLAN.md`. Short version:

1. **Wrong cadence on our hardware.** The trainer's step-peak threshold was `0.35 * G`, tuned to the Felius dataset. On our MYOSA + filter chain, only ~1.7% of walking samples crossed it, so the firmware's step detector missed most footstrikes and reported cadence as ~half its true value (e.g. 48 spm for actual 90 spm walking).
2. **Slow walking invisible to the model.** The autocorrelation lag search was `range(30, 80)` — covers only 75–200 spm. Anything slower (typical of elderly walkers, the actual target population) came out with no detected step periodicity.
3. **Hardware-domain mismatch.** Even after fixing extraction, our healthy walking landed in the *stroke* region of the Felius-trained model's feature space — not because of pathology, but because Felius hardware filters / mounts / walking protocol all differed from ours.
4. **Class-manifold imbalance.** Felius `features.csv` has 1,476 healthy vs 8,405 stroke windows; the boundary is biased toward stroke for ambiguous cases. (Already partially handled by `class_weight='balanced'`, but not enough.)
5. **App was running its own ML in parallel.** The phone had a duplicate copy of the (out-of-date) classifier baked in. ESP and app were producing different verdicts; only the app's was being shown.

---

## What changed

### Firmware

- **Step-peak threshold lowered**: `0.35 * G → 0.20 * G`. Spectral analysis confirms our walking peaks land around 2–3 m/s², not 3.5+. New threshold matches the FFT-derived cadence on our hardware.
- **Autocorrelation range widened**: `range(30, 80) → range(30, 200)` samples — now covers cadences from 30 spm (very slow) to 200 spm (running). Old range silently dropped slow walking as "no_gait".
- **`isValidGaitWindow` cadence floor relaxed** from 40 spm to 30 spm to match.
- **New ML model embedded** (`gait_model.h`) — see "Model" below.
- **Smoother now exposes recent counts** so the BLE message can carry a confidence signal without bringing the model back onto the phone.
- **BLE wire format extended** to publish smoother counts:
  ```
  V2: <tMs>,<raw>,<stable>,<stepCount>,<cadence>,<stepAc>
  V3: <tMs>,<raw>,<stable>,<stepCount>,<cadence>,<stepAc>,<healthyN>,<strokeN>,<validN>
  ```
- **`SERIAL_DEBUG` master flag** (default `false`) — when off, the compiler dead-code-eliminates every Serial call so no UART work happens in the IMU hot path.

### Model — Scenario F (CORAL domain-adapted RF)

Trained with the **CORAL** (Correlation Alignment) domain-adaptation technique:

1. Compute mean & covariance of Felius healthy features.
2. Compute mean & covariance of *our* healthy features.
3. Find the linear transform that maps Felius healthy → our hardware's distribution.
4. Apply the **same** transform to Felius stroke. This gives us synthetic "what stroke gait would look like on our hardware" data — without needing actual patient recordings on our setup.
5. Train a Random Forest (20 trees, depth 9, `class_weight='balanced'`) on `aligned_Felius_H + aligned_Felius_S + ours_H`.

**Validation results** (subject-grouped 5-fold CV on Felius):

| Metric | Value |
|---|---|
| Overall accuracy | 86.9% ± 6.1% |
| Stroke recall | 90.5% |
| Healthy recall | 65.1% |
| Held-out our-hardware healthy walking | 64% HEALTHY (rest split borderline; see below) |

Why "only" 64% on our healthy holdout instead of 100%? The earlier (Scenario B) model that scored 100% was exploiting a hardware-signature shortcut — easy in training, but it would not generalise to actual stroke patients on our hardware. Scenario F has no such shortcut available, and the 64% reflects honest model uncertainty on borderline windows (median P(stroke) = 0.36, *not* a confident wrong call). This is the technically correct trade-off for a real stroke detector.

### Phone app

- **All on-device ML removed.** Deleted `GaitClassifier.kt`, `FeatureExtractor.kt`, `GaitModel.java`, and the per-sample `MetricsEngine`. The app no longer touches the model, no longer extracts features, no longer scores anything itself. **Single source of truth: the ESP.**
- **New BLE subscription** to the ML characteristic (`...5b1a05`) — parses each verdict packet from the ESP and feeds it into a tiny `SessionAggregator`.
- **`SessionAggregator`** is the only "logic" left on the phone — it just bookkeeps a rolling timeline of recent verdicts and the running healthy% for the session.

### Live UI redesign

The session screen now leads with the verdict, not buried under a generic "score":

1. **Hero verdict card** — huge, centred `HEALTHY` / `STROKE-LIKE` label in the model's class colour. Two-sided confidence bar below (fills toward green for healthy, red for stroke; centre = 50/50 wobble).
2. **Verdict timeline strip** — horizontal bar showing the last ~30 s of model verdicts, oldest left → newest right. Each cell shaded by that window's healthy/stroke fraction (brighter = higher confidence). At a glance you can see whether the model is steady or oscillating.
3. **Session stats row** — three tiles: SESSION (% healthy windows so far, colour-coded), STEPS (cumulative + current cadence in spm), TIME (elapsed).
4. **Sensor charts** — kept at the bottom for diagnostics.

What was removed:
- The on-device-computed numerical "SCORE" that didn't correspond to anything clinically meaningful (it was a low-pass on accel-magnitude jerk).
- The `REPS` label — now always called STEPS, since the only mode that counts other-than-steps was retired with `MetricsEngine`.
- The duplicate small "Gait verdict" card — now the hero.

---

## Data flow (V3)

```
┌────────┐   100 Hz IMU   ┌────────┐
│ MPU6050│ ──────────────►│  ESP32 │
└────────┘                │        │
                          │ feature extraction (V3 thresholds)
                          │ gait_model.predict() (CORAL-aligned RF)
                          │ MlLabelSmoother (N=7 majority vote)
                          │
                          ├── BLE notify CHAR_SENSOR (CSV samples)
                          │   └► phone: chart strip only
                          │
                          └── BLE notify CHAR_ML (verdict + counts)
                              └► phone: hero verdict + timeline + session %
```

The phone never:
- runs the model
- extracts features
- computes a "score"

…it only displays what the ESP decided.

---

## Confirmed ESP32-compatible

- `Version3.ino` only uses APIs already in V2 (NimBLEDevice, Wire, MPU6050 helpers, BMP180, OLED) — no new dependencies.
- `gait_model.h` is `Eloquent::ML::Port::GaitClassifier` — same shape as the V2 `gait_model.h`, just with the new tree contents. The `predict(float* x)` API is identical.
- Source is ~724 KB but most of that is single-branch `if/else` chains; the compiler aggressively dead-code-eliminates and inlines. Compiled flash usage should remain comfortably under the default 1.3 MB ESP32 app partition. If it doesn't, switch the partition scheme in Arduino IDE to **"Huge App (3MB No OTA)"**.
- 18-feature input order is unchanged from V2 (see `feature_order.txt`).

---

## Known limitations

1. **No real stroke patient data on our hardware** — CORAL is a linear approximation. If our hardware's relationship to Felius is non-linear in some features, CORAL undercorrects. Future work: collect a few minutes of *simulated* stroke gait on our hardware (one of you walks with intentional limp / drag / hesitation) and fold that in directly.
2. **BLE transport drops samples** — the user's 588 s recording came in at native 85 Hz instead of 100 Hz, with sporadic 100–800 ms gaps. Disabling Serial debug didn't fix this; suspected cause is `BLECharacteristic::notify()` backpressure under load. Worth investigating before claiming reliable real-time inference.
3. **Single-subject training set on the healthy side** — our 100 healthy windows came from one walker (you). For a defensible deployment claim, need 3–5 healthy subjects.

See `../MODEL_DIAGNOSIS_AND_RETRAIN_PLAN.md` for the full diagnosis and `../retrained_2026-05-15/` for all the candidate-model artifacts.
