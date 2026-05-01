import os
from pathlib import Path

import numpy as np
import joblib
import librosa
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    accuracy_score,
    classification_report,
    confusion_matrix,
)
from sklearn.model_selection import train_test_split


DATASET_DIR = Path(r"D:\Acoustic\sliced\New_sliced")  # folder direction
CLASS_NAMES = ["Acrylic", "Wood", "Gravel", "Slide"]         # File name
SR = 16000                                  #Sampling rate


def extract_features(wav_path: Path, sr: int = SR) -> np.ndarray:
    y, sr = librosa.load(wav_path, sr=sr, mono=True)

    if len(y) < 100:
        raise ValueError(f"Audio too short: {wav_path}")

    features = []

    # MFCC
    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
    features.extend(np.mean(mfcc, axis=1))
    features.extend(np.std(mfcc, axis=1))

    # Delta MFCC with adaptive width
    n_frames = mfcc.shape[1]
    if n_frames >= 3:
        delta_width = min(9, n_frames)
        if delta_width % 2 == 0:
            delta_width -= 1
        if delta_width >= 3:
            mfcc_delta = librosa.feature.delta(mfcc, width=delta_width)
            features.extend(np.mean(mfcc_delta, axis=1))
            features.extend(np.std(mfcc_delta, axis=1))
        else:
            features.extend([0.0] * 13)
            features.extend([0.0] * 13)
    else:
        features.extend([0.0] * 13)
        features.extend([0.0] * 13)

    # Spectral features
    spec_centroid = librosa.feature.spectral_centroid(y=y, sr=sr)
    spec_bandwidth = librosa.feature.spectral_bandwidth(y=y, sr=sr)
    spec_rolloff = librosa.feature.spectral_rolloff(y=y, sr=sr)
    zcr = librosa.feature.zero_crossing_rate(y)
    rms = librosa.feature.rms(y=y)

    for feat in [spec_centroid, spec_bandwidth, spec_rolloff, zcr, rms]:
        features.append(np.mean(feat))
        features.append(np.std(feat))

    return np.array(features, dtype=np.float32)


def load_dataset(dataset_dir: Path, class_names: list[str]):
    X = []
    y = []
    file_paths = []
    loaded_counts = {class_name: 0 for class_name in class_names}

    for label_idx, class_name in enumerate(class_names):
        class_dir = dataset_dir / class_name
        if not class_dir.exists():
            raise FileNotFoundError(f"Class folder not found: {class_dir}")

        wav_files = sorted(class_dir.glob("*.wav"))
        print(f"[INFO] {class_name}: found {len(wav_files)} wav files")

        for wav_path in wav_files:
            try:
                feat = extract_features(wav_path)
                X.append(feat)
                y.append(label_idx)
                file_paths.append(str(wav_path))
                loaded_counts[class_name] += 1
            except Exception as e:
                print(f"[WARN] Skipped {wav_path.name}: {e}")

    print("\n[INFO] Successfully loaded samples:")
    for class_name in class_names:
        print(f"  {class_name}: {loaded_counts[class_name]}")

    if len(X) == 0:
        raise RuntimeError("No valid audio samples loaded.")

    return np.array(X), np.array(y), file_paths


def main():
    print(f"[INFO] Loading dataset from: {DATASET_DIR}")
    X, y, file_paths = load_dataset(DATASET_DIR, CLASS_NAMES)

    print(f"[INFO] Total valid samples: {len(X)}")
    print(f"[INFO] Feature dimension: {X.shape[1]}")

    # Random split for quick test only
    unique, counts = np.unique(y, return_counts=True)
    print("\n[INFO] Label counts:")
    for u, c in zip(unique, counts):
        print(f"  {CLASS_NAMES[u]}: {c}")

    if np.min(counts) < 2:
        raise ValueError("At least one class has fewer than 2 valid samples. Check skipped files.")
    X_train, X_test, y_train, y_test, train_files, test_files = train_test_split(
        X,
        y,
        file_paths,
        test_size=0.2,
        random_state=42,
        stratify=y,
    )

    print(f"[INFO] Train samples: {len(X_train)}")
    print(f"[INFO] Test samples: {len(X_test)}")

    model = RandomForestClassifier(
        n_estimators=200,
        random_state=42,
        class_weight="balanced",
    )
    model.fit(X_train, y_train)
    from pathlib import Path

    MODEL_DIR = Path(r"D:\Acoustic\models")
    MODEL_DIR.mkdir(parents=True, exist_ok=True)

    joblib.dump(model, MODEL_DIR / "impact_rf_model_newslide.joblib")
    joblib.dump(CLASS_NAMES, MODEL_DIR / "impact_class_names_newslide.joblib")
    y_pred = model.predict(X_test)

    acc = accuracy_score(y_test, y_pred)
    print("\n=== RESULTS ===")
    print(f"Accuracy: {acc:.4f}\n")

    print("Classification report:")
    print(classification_report(y_test, y_pred, target_names=CLASS_NAMES))

    cm = confusion_matrix(y_test, y_pred)
    print("Confusion matrix:")
    print(cm)

    print("\nSample predictions:")
    for i in range(min(10, len(X_test))):
        true_label = CLASS_NAMES[y_test[i]]
        pred_label = CLASS_NAMES[y_pred[i]]
        print(f"{Path(test_files[i]).name} | true={true_label} | pred={pred_label}")


if __name__ == "__main__":
    main()