import argparse
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.ndimage import uniform_filter1d


def read_wav_mono(path: Path):
    sr, data = wavfile.read(str(path))

    if data.dtype == np.int16:
        data = data.astype(np.float32) / 32768.0
    elif data.dtype == np.int32:
        data = data.astype(np.float32) / 2147483648.0
    elif data.dtype == np.uint8:
        data = (data.astype(np.float32) - 128.0) / 128.0
    else:
        data = data.astype(np.float32)

    if data.ndim == 2:
        data = np.mean(data, axis=1)

    return sr, data


def write_wav(path: Path, sr: int, data: np.ndarray):
    data = np.clip(data, -1.0, 1.0)
    out = (data * 32767.0).astype(np.int16)
    wavfile.write(str(path), sr, out)


def moving_envelope(x: np.ndarray, sr: int, win_ms: float = 5.0):
    win_samples = max(1, int(sr * win_ms / 1000.0))
    env = uniform_filter1d(np.abs(x), size=win_samples, mode="nearest")
    return env


def estimate_noise_floor(env: np.ndarray, warmup_ratio: float = 0.2):
    n = max(1, int(len(env) * warmup_ratio))
    early = env[:n] if n < len(env) else env
    median = np.median(early)
    mad = np.median(np.abs(early - median)) + 1e-9
    return median, mad


def find_impacts(
    x: np.ndarray,
    sr: int,
    threshold_sigma: float = 8.0,
    min_gap_ms: float = 250.0,
    envelope_ms: float = 5.0,
    search_start_ms: float = 0.0,
):
    env = moving_envelope(x, sr, win_ms=envelope_ms)

    start_idx = int(sr * search_start_ms / 1000.0)
    env_search = env[start_idx:]

    if len(env_search) == 0:
        return []

    median, mad = estimate_noise_floor(env_search)
    threshold = median + threshold_sigma * mad

    above = env_search > threshold
    if not np.any(above):
        return []

    rising = np.where(np.logical_and(above[1:], ~above[:-1]))[0] + 1
    if above[0]:
        rising = np.concatenate(([0], rising))

    min_gap = int(sr * min_gap_ms / 1000.0)
    peaks = []
    last_keep = -10**12

    for r in rising:
        local_end = min(len(env_search), r + int(0.05 * sr))
        local_peak = r + np.argmax(env_search[r:local_end])
        global_peak = local_peak + start_idx

        if global_peak - last_keep >= min_gap:
            peaks.append(global_peak)
            last_keep = global_peak

    return peaks


def slice_around_peaks(
    x: np.ndarray,
    sr: int,
    peaks,
    pre_ms: float,
    post_ms: float,
    pad_if_needed: bool = True,
):
    pre = int(sr * pre_ms / 1000.0)
    post = int(sr * post_ms / 1000.0)

    segments = []
    for p in peaks:
        start = p - pre
        end = p + post

        if not pad_if_needed:
            if start < 0 or end > len(x):
                continue
            seg = x[start:end]
        else:
            seg_len = pre + post
            seg = np.zeros(seg_len, dtype=np.float32)

            src_start = max(0, start)
            src_end = min(len(x), end)

            dst_start = src_start - start
            dst_end = dst_start + (src_end - src_start)
            seg[dst_start:dst_end] = x[src_start:src_end]

        segments.append((p, seg))

    return segments


def infer_label_from_filename(stem: str, separator: str | None = None) -> str:
    """
    Infer class folder name from filename.

    Examples:
        Wood_01.wav      -> Wood
        Gravel-hit-02    -> Gravel   (with --label-separator -)
        Acrylic.wav      -> Acrylic
    """
    if separator:
        label = stem.split(separator)[0].strip()
    else:
        parts = stem.replace("-", "_").split("_")
        label = parts[0].strip()

    return label if label else "Unknown"


def process_file(
    wav_path: Path,
    out_dir: Path,
    threshold_sigma: float,
    min_gap_ms: float,
    envelope_ms: float,
    search_start_ms: float,
    pre_ms: float,
    post_ms: float,
    keep_only_first: bool,
    auto_subfolders: bool,
    label_separator: str | None,
):
    sr, x = read_wav_mono(wav_path)

    peaks = find_impacts(
        x=x,
        sr=sr,
        threshold_sigma=threshold_sigma,
        min_gap_ms=min_gap_ms,
        envelope_ms=envelope_ms,
        search_start_ms=search_start_ms,
    )

    if keep_only_first and len(peaks) > 0:
        peaks = [peaks[0]]

    segments = slice_around_peaks(
        x=x,
        sr=sr,
        peaks=peaks,
        pre_ms=pre_ms,
        post_ms=post_ms,
        pad_if_needed=True,
    )

    if not segments:
        print(f"[NO IMPACT FOUND] {wav_path.name}")
        return 0

    target_dir = out_dir
    if auto_subfolders:
        label = infer_label_from_filename(wav_path.stem, separator=label_separator)
        target_dir = out_dir / label
        target_dir.mkdir(parents=True, exist_ok=True)

    stem = wav_path.stem
    count = 0
    for i, (peak, seg) in enumerate(segments, start=1):
        peak_ms = 1000.0 * peak / sr
        out_name = f"{stem}_impact_{i:03d}_{peak_ms:08.1f}ms.wav"
        out_path = target_dir / out_name
        write_wav(out_path, sr, seg)
        count += 1

    location_msg = f" -> {target_dir}" if auto_subfolders else ""
    print(f"[OK] {wav_path.name} -> {count} slice(s){location_msg}")
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Automatically slice impact sounds from WAV files."
    )
    parser.add_argument("--input-dir", type=str, required=True, help="Folder with WAV files")
    parser.add_argument("--output-dir", type=str, required=True, help="Folder to save slices")
    parser.add_argument("--threshold-sigma", type=float, default=8.0,
                        help="Higher = stricter detection")
    parser.add_argument("--min-gap-ms", type=float, default=250.0,
                        help="Minimum time between two impacts")
    parser.add_argument("--envelope-ms", type=float, default=5.0,
                        help="Envelope smoothing window")
    parser.add_argument("--search-start-ms", type=float, default=0.0,
                        help="Ignore the beginning if needed")
    parser.add_argument("--pre-ms", type=float, default=80.0,
                        help="Keep this much audio before impact")
    parser.add_argument("--post-ms", type=float, default=420.0,
                        help="Keep this much audio after impact")
    parser.add_argument("--first-only", action="store_true",
                        help="Only keep the first detected impact in each file")
    parser.add_argument("--auto-subfolders", action="store_true",
                        help="Automatically save slices into subfolders inferred from file names")
    parser.add_argument("--label-separator", type=str, default=None,
                        help="Optional separator used to infer label from file name, e.g. '_' or '-' ")

    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    wav_files = sorted(input_dir.glob("*.wav"))
    if not wav_files:
        print(f"No WAV files found in: {input_dir}")
        return

    total_slices = 0
    for wav_path in wav_files:
        total_slices += process_file(
            wav_path=wav_path,
            out_dir=output_dir,
            threshold_sigma=args.threshold_sigma,
            min_gap_ms=args.min_gap_ms,
            envelope_ms=args.envelope_ms,
            search_start_ms=args.search_start_ms,
            pre_ms=args.pre_ms,
            post_ms=args.post_ms,
            keep_only_first=args.first_only,
            auto_subfolders=args.auto_subfolders,
            label_separator=args.label_separator,
        )

    print(f"\nDone. Total slices saved: {total_slices}")


if __name__ == "__main__":
    main()
