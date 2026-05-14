"""
SafeStride — train Random Forest classifier on the Felius stroke gait dataset.

Pipeline:
  1. Load combined_lower_back.csv (5.27M rows)
  2. Resample 104 Hz → 100 Hz per subject
  3. Cut into 5-second windows
  4. Extract 18 gait features per window
  5. Cross-validate by SUBJECT (group K-fold)
  6. Train final model on all data and save
"""

import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
from pathlib import Path
from scipy import signal
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold
from sklearn.metrics import (classification_report, confusion_matrix,
                              accuracy_score, roc_auc_score)

# =============================================================
# CONFIG
# =============================================================
DATA_FILE = Path.home() / "Downloads" / "final_dataset" / "combined_lower_back.csv"

SOURCE_HZ = 104                                  # dataset sample rate
TARGET_HZ = 100                                  # MYOSA sample rate
WINDOW_SEC = 5.0
WINDOW_SIZE = int(WINDOW_SEC * TARGET_HZ)        # 500 samples per window
STRIDE = WINDOW_SIZE // 2                        # 50% overlap

# CRITICAL: dataset axis convention.
# Discovered by inspecting means: AX has mean +0.94g → AX is the vertical axis.
# When mapping to body-frame variables for feature extraction:
#   vert  ← ax  (axis aligned with gravity)
#   ml    ← ay  (left/right sway)
#   sag   ← az  (forward/back, optional)
#   yaw   ← gx  (the gyro axis perpendicular to gravity)
DATASET_VERT = 'ax'
DATASET_ML   = 'ay'
DATASET_YAW  = 'gx'
G = 9.81

# Feature names — ORDER MATTERS. Firmware must match this exact order.
FEATURE_COLS = [
    'vert_mean', 'vert_std', 'vert_range', 'vert_skew', 'vert_kurt',
    'ml_std', 'ml_rms', 'ml_range',
    'yaw_std', 'yaw_abs_mean',
    'step_count', 'cadence_spm', 'step_interval_cv',
    'step_ac', 'stride_ac', 'gsi',
    'jerk_std', 'jerk_max'
]


# =============================================================
# FEATURE EXTRACTION (must match firmware exactly)
# =============================================================
def autocorr_at_lag(x, lag):
    if lag >= len(x): return 0.0
    x = x - np.mean(x)
    denom = np.sum(x * x)
    if denom < 1e-6: return 0.0
    return float(np.sum(x[:len(x)-lag] * x[lag:]) / denom)


def extract_features(vert, ml, yaw):
    """18-feature vector. Order = FEATURE_COLS."""
    f = {}
    f['vert_mean']  = float(np.mean(vert))
    f['vert_std']   = float(np.std(vert))
    f['vert_range'] = float(np.ptp(vert))
    f['vert_skew']  = float(pd.Series(vert).skew())
    f['vert_kurt']  = float(pd.Series(vert).kurt())
    f['ml_std']   = float(np.std(ml))
    f['ml_rms']   = float(np.sqrt(np.mean(ml**2)))
    f['ml_range'] = float(np.ptp(ml))
    f['yaw_std']      = float(np.std(yaw))
    f['yaw_abs_mean'] = float(np.mean(np.abs(yaw)))

    peaks, _ = signal.find_peaks(vert, height=0.35*G, distance=25)
    f['step_count'] = len(peaks)
    if len(peaks) > 1:
        intervals = np.diff(peaks) / TARGET_HZ
        f['cadence_spm']      = float(60.0 / np.mean(intervals))
        f['step_interval_cv'] = float(np.std(intervals) / np.mean(intervals))
    else:
        f['cadence_spm'] = 0.0
        f['step_interval_cv'] = 0.0

    best_ac, best_lag = 0.0, 0
    for lag in range(30, 80):
        ac = autocorr_at_lag(vert, lag)
        if ac > best_ac:
            best_ac, best_lag = ac, lag
    f['step_ac'] = best_ac

    stride_lag = best_lag * 2
    stride_ac = autocorr_at_lag(vert, stride_lag) if stride_lag < len(vert) else 0.0
    f['stride_ac'] = stride_ac
    f['gsi'] = float(best_ac / stride_ac) if stride_ac > 0.1 else 0.0

    jerk = np.diff(vert)
    f['jerk_std'] = float(np.std(jerk))
    f['jerk_max'] = float(np.max(np.abs(jerk)))

    return f


# =============================================================
# PROCESS DATASET
# =============================================================
def process_subject(group):
    """One subject's continuous recording → list of feature rows."""
    rows = []
    # Resample 104 Hz → 100 Hz
    n_out = int(len(group) * TARGET_HZ / SOURCE_HZ)
    if n_out < WINDOW_SIZE:
        return rows
    
    vert_raw = signal.resample(group[DATASET_VERT].values, n_out)
    ml_raw   = signal.resample(group[DATASET_ML].values, n_out)
    yaw      = signal.resample(group[DATASET_YAW].values, n_out)
    
    # Body-frame transform
    vert_dyn = (vert_raw - 1.0) * G   # remove gravity, m/s^2
    ml_dyn = ml_raw * G
    
    # Window
    for start in range(0, len(vert_dyn) - WINDOW_SIZE + 1, STRIDE):
        end = start + WINDOW_SIZE
        feat = extract_features(vert_dyn[start:end], ml_dyn[start:end], yaw[start:end])
        rows.append(feat)
    
    return rows


def main():
    if not DATA_FILE.exists():
        print(f"ERROR: {DATA_FILE} not found.")
        print("Update DATA_FILE at the top to point to combined_lower_back.csv")
        return
    
    print(f"Loading {DATA_FILE} (may take ~30 seconds, 5M rows)...")
    df = pd.read_csv(DATA_FILE)
    print(f"  Loaded: {len(df):,} rows, {df['subject'].nunique()} subjects")
    
    print("\nExtracting features per subject...")
    rows = []
    for subject, group in df.groupby('subject'):
        status = group['status'].iloc[0]
        label = 0 if status == 'Healthy' else 1
        subject_rows = process_subject(group)
        for r in subject_rows:
            r['label'] = label
            r['subject'] = subject
        rows.extend(subject_rows)
        print(f"  {subject}: {len(subject_rows)} windows ({status})")
    
    feat_df = pd.DataFrame(rows).replace([np.inf, -np.inf], 0).fillna(0)
    print(f"\nTotal: {len(feat_df)} windows")
    print(f"  Healthy (0): {sum(feat_df['label']==0)}")
    print(f"  Stroke  (1): {sum(feat_df['label']==1)}")
    
    feat_df.to_csv('features.csv', index=False)
    print("Saved features.csv")
    
    # Save feature order for firmware
    with open('feature_order.txt', 'w') as f:
        for c in FEATURE_COLS:
            f.write(c + '\n')
    
    X = feat_df[FEATURE_COLS].values.astype(np.float32)
    y = feat_df['label'].values.astype(np.int32)
    groups = feat_df['subject'].values
    
    # =========================================================
    # Subject-held-out cross-validation
    # =========================================================
    print(f"\n=== 5-Fold Subject-Held-Out Cross-Validation ===")
    gkf = GroupKFold(n_splits=5)
    
    fold_accs = []
    fold_aucs = []
    
    for fold, (train_idx, test_idx) in enumerate(gkf.split(X, y, groups), 1):
        # Find this block in train_model.py and update:
        clf = RandomForestClassifier(
            n_estimators=30,           # was 10 — more trees = more accurate
            max_depth=8,               # was 6 — slightly deeper
            min_samples_leaf=10,       # was 5 — bigger leaves resist overfitting
            class_weight='balanced',   # keep
            random_state=42, n_jobs=-1
        )
        clf.fit(X[train_idx], y[train_idx])
        
        y_pred = clf.predict(X[test_idx])
        y_prob = clf.predict_proba(X[test_idx])[:, 1]
        
        acc = accuracy_score(y[test_idx], y_pred)
        try:
            auc = roc_auc_score(y[test_idx], y_prob)
        except ValueError:
            auc = float('nan')
        fold_accs.append(acc)
        fold_aucs.append(auc)
        
        print(f"  Fold {fold}: acc={acc:.3f}, auc={auc:.3f}, "
              f"test_subjects={len(set(groups[test_idx]))}")
    
    print(f"\nMean accuracy: {np.mean(fold_accs):.3f} ± {np.std(fold_accs):.3f}")
    print(f"Mean AUC:      {np.nanmean(fold_aucs):.3f} ± {np.nanstd(fold_aucs):.3f}")
    
    # =========================================================
    # Final model on all data
    # =========================================================
    print(f"\n=== Training Final Model on All Data ===")
    clf_final = RandomForestClassifier(
        n_estimators=10, max_depth=6, min_samples_leaf=5,
        class_weight='balanced', random_state=42, n_jobs=-1
    )
    clf_final.fit(X, y)
    
    y_pred = clf_final.predict(X)
    print("\nTraining-set sanity check (not validation):")
    print(classification_report(y, y_pred, target_names=['Healthy', 'Stroke']))
    print("Confusion matrix:")
    print(confusion_matrix(y, y_pred))
    
    # Feature importance
    importance = pd.DataFrame({
        'feature': FEATURE_COLS,
        'importance': clf_final.feature_importances_
    }).sort_values('importance', ascending=False)
    print(f"\nTop 10 features:")
    print(importance.head(10).to_string(index=False))
    
    plt.figure(figsize=(8, 6))
    top = importance.head(15)
    plt.barh(top['feature'][::-1], top['importance'][::-1])
    plt.xlabel('Importance')
    plt.title('SafeStride — Feature Importance')
    plt.tight_layout()
    plt.savefig('feature_importance.png', dpi=120)
    print("Saved feature_importance.png")
    
    joblib.dump(clf_final, 'safestride_model.pkl')
    print("Saved safestride_model.pkl")
    
    total_nodes = sum(e.tree_.node_count for e in clf_final.estimators_)
    print(f"\nModel: {len(clf_final.estimators_)} trees, {total_nodes} total nodes")
    print(f"Estimated C++ code size: ~{total_nodes * 50} bytes ({total_nodes * 50 / 1024:.1f} KB)")


if __name__ == '__main__':
    main()