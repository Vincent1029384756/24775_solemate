# Sole-Mates: Surface Adaptive Feet For Enhanced Locomotion
This repository accompanies a course project for 24-775 Bioinspired Robot and Experimentation

## Overview
This project develops an experimental test rig to evaluate terrain-adaptive foot design for legged robots. The system enables automatic switching between interchangeable foot soles based on detected surface type, allowing controlled comparison across different terrains, including wet acrylic, wood, and gravel.

The goal is to investigate whether adapting foot properties to surface conditions can improve locomotion efficiency compared to a single, non-adaptive foot design.

## System Architecture
- **Arduino Mega**: High level control logic and communication
- **RoboClaw Motor Controller**: Controls hip motor and sole switching motor. Comes with built in encoder support and PID controller.
- **TB6612 Motor Dtiver**: Drives the terrain sensing mechanism motor using PWM and digital control signals from the Arduino; control is implemented in software on the Arduino (no on-board PID).
- **Laptop**: Runs terrain classification algorithm

![alt text](image.png)

## Setup
### Arduino Programs

This repository includes two separate Arduino programs for different operating modes:

#### 1. Manual Foot Switching (`manual_mode.ino`)
- Upload this program to the Arduino for manual control of the sole-switching mechanism
- Used for testing, debugging, and validation of the mechanical system

**Controls**
| Key | Function |
|-----|----------|
| `a` | Start continuously walking |
| `b` | Rotate switch plate 120 deg |
| `c` | Rotate switch plate 240 deg|
| `s` | Check status |
| `0` | Switch to sole 1 |
| `1` | Switch to sole 2 |
| `2` | Switch to sole 3 |

#### 2. Automatic Foot Switching (`auto_mode.ino`)
- Upload this program to enable terrain-adaptive operation
- Requires the terrain classification python code running on the laptop
- The Arduino receives terrain information via serial communication and switches soles accordingly

### Terrain Classification Python Code
This mode requires the terrain classification Python code to run on the laptop. The Arduino receives terrain information through serial communication and switches soles accordingly.

## Terrain Classification Python Code

The terrain classification module implements an impact-based acoustic classification pipeline. A contact microphone or piezo sensor is used to record terrain interaction signals. Impact events are automatically detected and segmented, then classified using a Random Forest model.

### Folder Structure

The terrain classification folder is organized as follows:

```text
Acoustic Terrain Classifier/
│
├── Model/
│  └── impact_class_names_newslide.joblib
│
├── Sliced Sample/
│   ├── Acrylic/
│   ├── Wood/
│   ├── Gravel/
│   └── Slide/
│
├── Unsliced Sample/
│   └── *.wav
│
└──  Training/
   ├── auto_slice_impacts_auto_folder.py
   ├── train_impact_classifier.py
   └── real_time_acoustic_strict_com4_slide1.py


```

Only a subset of the full audio dataset is included in this repository for demonstration.

### Pipeline Overview

The acoustic terrain classification pipeline consists of three stages:

1. Impact detection and slicing
2. Feature extraction and model training
3. Real-time classification and control integration

### 1. Automatic Impact Slicing

The script `auto_slice_impacts_auto_folder.py` detects impact events from raw audio recordings and extracts fixed-length segments.

Example command:

```bash
py -3.12 auto_slice_impacts_auto_folder.py ^
  --input-dir D:\Acoustic\unsliced\New\Slide ^
  --output-dir D:\Acoustic\sliced\New_sliced ^
  --threshold-sigma 10 ^
  --min-gap-ms 900 ^
  --envelope-ms 12 ^
  --pre-ms 60 ^
  --post-ms 300 ^
  --auto-subfolders
```

Parameters:

- `threshold-sigma = 10`: Controls impact detection sensitivity. Higher values reduce false detections from background noise.
- `min-gap-ms = 900`: Enforces a minimum interval between detected events to prevent multiple slices from a single physical impact.
- `envelope-ms = 12`: Sets the smoothing window for the signal envelope.
- `pre-ms = 60`, `post-ms = 300`: Defines the audio window retained before and after each detected impact.
- `auto-subfolders`: Automatically organizes sliced audio into class folders based on file names.

### 2. Model Training

The script `train_impact_classifier.py` trains a Random Forest classifier using sliced audio samples.

Features used:

- MFCC mean and standard deviation
- Delta MFCC
- Spectral centroid
- Spectral bandwidth
- Spectral rolloff
- Zero-crossing rate
- RMS energy

Model configuration:

- Classifier: Random Forest
- Number of trees: 200
- Sampling rate during feature extraction: 16 kHz
- Train/test split: 80/20, stratified

Expected model output:

```text
Model/
├── impact_rf_model_newslide.joblib
└── impact_class_names_newslide.joblib
```

Run training:

```bash
py -3.12 train_impact_classifier.py
```

### 3. Real-Time Classification

The script `real_time_acoustic_strict_com4_slide1.py` performs real-time terrain classification and communicates with the Arduino controller.

Before running, verify the model paths and Arduino port inside the script:

```python
MODEL_PATH = Path(r"D:\Acoustic\models\impact_rf_model_newslide.joblib")
CLASS_PATH = Path(r"D:\Acoustic\models\impact_class_names_newslide.joblib")
ARDUINO_PORT = "COM4"
```

Run real-time classification:

```bash
py -3.12 real_time_acoustic_strict_com4_slide1.py
```

The real-time system includes:

- Real-time audio acquisition
- Impact event detection
- Feature extraction
- Confidence-based filtering
- Short-term voting for prediction stability
- Serial communication with the Arduino

Terrain command mapping:

```text
gravel  -> 0
acrylic -> 1
wood    -> 2
slide   -> no sole switching
```

The `slide` class is used to suppress unnecessary sole switching when the sensor detects sliding-induced signals rather than clear terrain impacts.

## Dependencies

Install the required Python packages:

```bash
pip install numpy scipy librosa scikit-learn joblib sounddevice pyserial
```

## Notes

The acoustic pipeline is designed primarily for impact-based signals. Continuous signals such as sliding can be more difficult to segment and classify reliably because they may not contain clear transient peaks.

In real-time operation, confidence thresholding, short-term voting, and gait-cycle-level filtering are used to improve robustness and reduce false sole-switching commands.
