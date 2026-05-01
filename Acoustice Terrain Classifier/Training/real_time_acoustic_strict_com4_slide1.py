from pathlib import Path
from collections import deque, Counter
import queue
import time
import serial

import numpy as np
import joblib
import librosa
import sounddevice as sd
from scipy.ndimage import uniform_filter1d

# =========================
# Paths
# =========================
MODEL_PATH = Path(r"D:\Acoustic\models\impact_rf_model_newslide.joblib")
CLASS_PATH = Path(r"D:\Acoustic\models\impact_class_names_newslide.joblib")

# =========================
# Serial / Arduino settings
# =========================
ARDUINO_PORT = "COM4"
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT = 0.2

# Prevent rapid repeated switching if predictions fluctuate
SWITCH_COOLDOWN = 3.0  # seconds

# =========================
# Audio / detection settings
# =========================
SR = 16000
CHANNELS = 1
DTYPE = "float32"
BLOCKSIZE = 800
BUFFER_SECONDS = 3.0

# Trigger / slice settings
THRESHOLD_SIGMA = 10.0
MIN_GAP_MS = 900
ENVELOPE_MS = 12.0
PRE_MS = 60
POST_MS = 300
WARMUP_SECONDS = 1.5
REARM_BELOW_RATIO = 0.50

# Optional filtering of printed events
CONF_THRESHOLD = 0.45
USE_VOTING = True
VOTE_SIZE = 3
VOTE_MIN_COUNT = 2

# =========================
# Audio utils
# =========================
def moving_envelope(x: np.ndarray, sr: int, win_ms: float = 5.0) -> np.ndarray:
    win_samples = max(1, int(sr * win_ms / 1000.0))
    return uniform_filter1d(np.abs(x), size=win_samples, mode="nearest")


def extract_features_from_signal(y: np.ndarray, sr: int = SR) -> np.ndarray:
    if len(y) < 100:
        raise ValueError("Audio too short for feature extraction.")

    features = []

    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
    features.extend(np.mean(mfcc, axis=1))
    features.extend(np.std(mfcc, axis=1))

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
            features.extend([0.0] * 26)
    else:
        features.extend([0.0] * 26)

    spec_centroid = librosa.feature.spectral_centroid(y=y, sr=sr)
    spec_bandwidth = librosa.feature.spectral_bandwidth(y=y, sr=sr)
    spec_rolloff = librosa.feature.spectral_rolloff(y=y, sr=sr)
    zcr = librosa.feature.zero_crossing_rate(y)
    rms = librosa.feature.rms(y=y)

    for feat in [spec_centroid, spec_bandwidth, spec_rolloff, zcr, rms]:
        features.append(np.mean(feat))
        features.append(np.std(feat))

    return np.array(features, dtype=np.float32)


# =========================
# Arduino serial controller
# =========================
class ArduinoSoleController:
    TERRAIN_TO_CMD = {
        "acrylic": "1",
        "gravel": "0",
        "wood": "2",
    }

    def __init__(self, port=ARDUINO_PORT, baudrate=ARDUINO_BAUDRATE, timeout=ARDUINO_TIMEOUT):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2.0)  # Arduino often resets when port opens
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[INFO] Arduino connected on {port} @ {baudrate}")

    def send_raw(self, cmd: str):
        if not isinstance(cmd, str) or len(cmd) != 1:
            raise ValueError(f"Command must be a single character, got: {cmd!r}")

        print(f"[PY->ARDUINO] {cmd}")
        # Do NOT send newline, because Arduino code reads a single char
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()

    def read_available_lines(self, duration=0.5):
        lines = []
        t0 = time.time()
        while time.time() - t0 < duration:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    lines.append(line)
            else:
                time.sleep(0.01)
        return lines

    def drain_serial(self, duration=0.1):
        """Read and print any pending Arduino output."""
        lines = self.read_available_lines(duration=duration)
        for line in lines:
            print("[ARDUINO]", line)
        return lines

    def get_status(self):
        self.send_raw("s")
        lines = self.read_available_lines(0.5)
        for line in lines:
            print("[ARDUINO]", line)
        return lines

    def switch_to(self, terrain_name: str, wait_done=False, timeout=20.0):
        terrain_name = terrain_name.lower().strip()
        if terrain_name not in self.TERRAIN_TO_CMD:
            raise ValueError(f"Unknown terrain: {terrain_name}")

        cmd = self.TERRAIN_TO_CMD[terrain_name]
        self.send_raw(cmd)

        # Try to read immediate reply from Arduino
        t0 = time.time()
        saw_switch_msg = False

        while time.time() - t0 < 1.0:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                print("[ARDUINO]", line)

                if line == "BUSY":
                    return False

                if line == "DONE":
                    return True

                if line.startswith("SOLE="):
                    # Status response or immediate state report
                    if "BUSY=0" in line:
                        return True

                if line.startswith("Switching to sole"):
                    saw_switch_msg = True
                    if not wait_done:
                        return True
                    break
            else:
                time.sleep(0.01)

        if not wait_done:
            # Non-blocking mode: if Arduino didn't explicitly reject, consider command sent
            return not False if saw_switch_msg else True

        # Optional blocking wait for DONE
        t1 = time.time()
        while time.time() - t1 < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                print("[ARDUINO]", line)

                if line == "DONE":
                    return True
                if line == "BUSY":
                    return False
            else:
                time.sleep(0.01)

        return False

    def start_motion(self):
        self.send_raw("a")

    def stop_motion(self):
        self.send_raw("d")

    def close(self):
        if hasattr(self, "ser") and self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Arduino serial closed.")


# =========================
# Main realtime logic
# =========================
def main():
    print("[INFO] Loading model...")
    model = joblib.load(MODEL_PATH)
    class_names = joblib.load(CLASS_PATH)

    sole_ctrl = ArduinoSoleController(
        port=ARDUINO_PORT,
        baudrate=ARDUINO_BAUDRATE,
        timeout=ARDUINO_TIMEOUT,
    )
    time.sleep(0.5)  # small delay for stability

    sole_ctrl.start_motion()   # send 'a'
    print("[PYTHON] Sent start command to Arduino")

    time.sleep(1.0)  # give Arduino time to react
    sole_ctrl.get_status()     # optional: see current state

    last_requested_terrain = None
    last_switch_time = 0.0

    audio_queue = queue.Queue()

    def audio_callback(indata, frames, time_info, status):
        if status:
            print(f"[AUDIO STATUS] {status}")
        try:
            audio_queue.put_nowait(indata[:, 0].copy())
        except queue.Full:
            pass

    buffer_len = int(SR * BUFFER_SECONDS)
    rolling = np.zeros(buffer_len, dtype=np.float32)
    total_samples = 0

    warmup_env_values = []
    threshold = None
    armed = True
    active_event = None
    last_event_peak_sample = -10**12

    min_gap_samples = int(SR * MIN_GAP_MS / 1000.0)
    pre_samples = int(SR * PRE_MS / 1000.0)
    post_samples = int(SR * POST_MS / 1000.0)

    vote_buffer = deque(maxlen=VOTE_SIZE)
    stable_label = None

    print("[INFO] Starting real-time acoustic classification...")
    print("[INFO] Press Ctrl+C to stop.\n")

    try:
        with sd.InputStream(
            samplerate=SR,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=BLOCKSIZE,
            callback=audio_callback,
        ):
            while True:
                # Drain any background Arduino messages without blocking audio logic
                sole_ctrl.drain_serial(duration=0.01)

                block = audio_queue.get()
                n = len(block)

                if n >= buffer_len:
                    rolling = block[-buffer_len:].copy()
                else:
                    rolling[:-n] = rolling[n:]
                    rolling[-n:] = block

                total_samples += n
                valid_len = min(total_samples, buffer_len)
                current = rolling[-valid_len:]

                env = moving_envelope(current, SR, win_ms=ENVELOPE_MS)
                current_abs_sample = total_samples

                # Warmup stage
                if threshold is None:
                    warmup_env_values.extend(env.tolist())
                    if total_samples >= int(SR * WARMUP_SECONDS):
                        warmup_arr = np.asarray(warmup_env_values, dtype=np.float32)
                        median = float(np.median(warmup_arr))
                        mad = float(np.median(np.abs(warmup_arr - median)) + 1e-9)
                        threshold = median + THRESHOLD_SIGMA * mad
                        print(
                            f"[INFO] Warmup done. "
                            f"median={median:.6f}, mad={mad:.6f}, threshold={threshold:.6f}"
                        )
                    continue

                # Re-arm once quiet enough
                if not armed and env[-1] < threshold * REARM_BELOW_RATIO:
                    armed = True

                # Trigger detection
                if armed and active_event is None and env[-1] > threshold:
                    if current_abs_sample - last_event_peak_sample >= min_gap_samples:
                        search_back = min(len(env), int(0.12 * SR))  # last 120 ms
                        recent = env[-search_back:]
                        local_peak_rel = int(np.argmax(recent))
                        peak_in_buffer = len(env) - search_back + local_peak_rel
                        peak_abs = total_samples - len(env) + peak_in_buffer

                        active_event = {
                            "peak_abs": int(peak_abs),
                            "started_at": current_abs_sample,
                        }
                        armed = False

                # When enough audio is available, slice and classify
                if active_event is not None:
                    peak_abs = active_event["peak_abs"]

                    if total_samples >= peak_abs + post_samples:
                        start_abs = peak_abs - pre_samples
                        end_abs = peak_abs + post_samples

                        if start_abs < total_samples - len(current):
                            # too old for rolling buffer
                            active_event = None
                        else:
                            start_idx = start_abs - (total_samples - len(current))
                            end_idx = end_abs - (total_samples - len(current))
                            segment = current[start_idx:end_idx]

                            if len(segment) == (pre_samples + post_samples):
                                try:
                                    feat = extract_features_from_signal(segment, sr=SR).reshape(1, -1)
                                    pred_idx = model.predict(feat)[0]
                                    pred_name = class_names[pred_idx]

                                    conf = None
                                    if hasattr(model, "predict_proba"):
                                        probs = model.predict_proba(feat)[0]
                                        conf = float(np.max(probs))

                                    event_time_s = peak_abs / SR
                                    rms_val = float(np.sqrt(np.mean(segment ** 2)))

                                    emit_label = pred_name
                                    stable_changed = False
                                    if USE_VOTING and conf is not None and conf >= CONF_THRESHOLD:
                                        vote_buffer.append(pred_name)

                                        counts = Counter(vote_buffer)
                                        top_label, top_count = counts.most_common(1)[0]

                                        if top_count >= VOTE_MIN_COUNT:
                                            if stable_label != top_label:
                                                stable_label = top_label
                                                stable_changed = True
                                            emit_label = stable_label
                                    if conf is not None and conf >= CONF_THRESHOLD:
                                        msg = f"[EVENT] t={event_time_s:.3f}s | pred={emit_label}"
                                        msg += f" | conf={conf:.3f}"
                                        msg += f" | rms={rms_val:.4f}"
                                        if USE_VOTING:
                                            msg += f" | votes={list(vote_buffer)} | stable={stable_label}"
                                            if stable_changed:
                                                msg += " <-- STABLE UPDATED"
                                        print(msg)

                                        terrain_label = emit_label.lower()
                                        now = time.time()

                                        if terrain_label == "slide":
                                            print("[PYTHON] slide detected -> no sole change needed")
                                        elif (
                                            terrain_label in ArduinoSoleController.TERRAIN_TO_CMD
                                            and (now - last_switch_time) > SWITCH_COOLDOWN
                                        ):
                                            ok = sole_ctrl.switch_to(terrain_label, wait_done=False)
                                            print(f"[PYTHON] switch_to({terrain_label}) -> {ok}")

                                            if ok:
                                                last_requested_terrain = terrain_label
                                                last_switch_time = now
                                    else:
                                        print(
                                            f"[SKIP] t={event_time_s:.3f}s | pred={pred_name} "
                                            f"| conf={conf:.3f} below threshold"
                                        )

                                except Exception as e:
                                    print(f"[WARN] Failed to classify event at {peak_abs / SR:.3f}s: {e}")

                            last_event_peak_sample = peak_abs
                            active_event = None

    finally:
        sole_ctrl.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")