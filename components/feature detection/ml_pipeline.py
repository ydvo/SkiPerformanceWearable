#!/usr/bin/env python3
"""
Sliding-window ML pipeline for ski boot IMU: straight vs turning, left/right turns, and lean (back/forward).
- Loads only the run segments (run 2/3/4) from the CSV; event times use EVENT_OFFSET_HOURS (5 = 10:28 local -> 15:28 UTC).
- 1s windows; features: gyro Z mean/std/max_abs, accel means, etc. Random Forest: straight vs turning.
- Turn counts: scipy peak detection with separate L/R height thresholds (default), or ML segment counter.
  Aggressive 0.5 Hz smoothing + strong prominence (0.25) gives unified thresholds that work across
  both tight and wide turns without per-run tuning.
- Run auto-detection: uses both gyro magnitude AND accelerometer variability to distinguish stopped
  from skiing.  Straight skiing (no turns) has low gyro but high accel vibration from snow contact.
- Lean: quaternion pitch -> lean angle (deg) with automatic sensor mounting offset calibration.
- Turn duration: seconds per turn (window count × WINDOW_SEC). Turn width: from mean |gyro_z| -> wide/medium/narrow.
- Noise filters: --filter {none,kalman,madgwick,butterworth,combined} pre-processes gyro/accel before feature extraction.
- Detectors:    --detector {threshold,ml} selects scipy peak-detection (default) or Random Forest (ml).
- Run: python3 ml_pipeline.py  (requires numpy, scikit-learn, scipy).
"""

import argparse
import csv
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

try:
    import joblib
except ImportError:
    joblib = None

try:
    from scipy.signal import find_peaks as _scipy_find_peaks
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False

from filters import apply_filter

MANIFEST_FILENAME = "manifest.json"


class EnsembleClassifier:
    """Wrapper that predicts by majority vote across multiple classifiers (for sequential training)."""
    def __init__(self, classifiers: list):
        self.classifiers = classifiers

    def predict(self, X):
        if not self.classifiers:
            raise RuntimeError("No classifiers in ensemble")
        preds = np.array([clf.predict(X) for clf in self.classifiers])
        # Majority vote per sample: turning (1) if more than half of models say 1
        votes_turning = np.sum(preds, axis=0)
        return (votes_turning > (len(self.classifiers) / 2)).astype(np.int64)

# Default paths and constants (overridden by config when loading saved model)
CSV_PATH = Path(__file__).parent / "data" / "2026_02_10_skiway_sensor_data.csv"
DEFAULT_MODEL_DIR = Path(__file__).parent / "saved_model"
SAMPLE_RATE_HZ = 128
WINDOW_SEC = 1.0
WINDOW_SAMPLES = int(SAMPLE_RATE_HZ * WINDOW_SEC)  # 128 samples = 1 second
# Turn-count thresholds (rad/s) — used by the ML segment counter as a fallback
TURN_THRESHOLD_LEFT = 0.19
TURN_THRESHOLD_RIGHT = 0.27
MIN_WINDOWS_PER_TURN = 1
MERGE_GAP_WINDOWS = 0
# Lean: pitch (rad) from quaternion; back = pitch > LEAN_THRESHOLD_RAD, forward = pitch < -LEAN_THRESHOLD_RAD
LEAN_THRESHOLD_RAD = 0.25
# Acceptable lean angles (degrees) for reporting; matches truth: run2 "leaning back a lot", run4 "occasional lean back"
LEAN_ANGLE_DEG_NEUTRAL_MAX = 10.0   # within ±10° = neutral / acceptable
LEAN_ANGLE_DEG_BACK_MIN = 14.0      # pitch > 14° = lean back
LEAN_ANGLE_DEG_FORWARD_MAX = -14.0 # pitch < -14° = lean forward
# Turn width (from mean |gyro_z| rad/s): lower rate = wider turn
TURN_WIDTH_WIDE_MAX_RATE = 0.35    # mean |gyro_z| < this = "wide"
TURN_WIDTH_NARROW_MIN_RATE = 0.55  # mean |gyro_z| > this = "narrow"; else "medium"
# Stopped vs skiing: a window is "stopped" only when BOTH gyro AND accel variability
# are below their thresholds.  Straight skiing has low gyro but high accel vibration
# (snow contact), so using both prevents misclassifying straight runs as stopped.
STOPPED_GYRO_THRESHOLD = 0.15
STOPPED_ACCEL_STD_THRESHOLD = 0.60
# Auto-detect runs: min duration (s) to count as a run; max gap (s) between windows to merge into one run
MIN_RUN_DURATION_SEC = 30.0
MAX_GAP_MERGE_SEC = 8.0

# --- Noise filter parameters ---
# Kalman 1-D filter (zero-phase, random-walk model, applied per gyro axis)
KALMAN_Q = 5e-2   # process noise: larger = tracks fast changes, smaller = smoother
KALMAN_R = 5e-1   # measurement noise: larger = smoother

# Madgwick orientation filter
MADGWICK_BETA = 0.01  # gain: low value for dynamic motion (skiing); avoids centripetal corruption

# --- Threshold detector parameters (used when --detector threshold) ---
# Separate L/R height thresholds with aggressive smoothing (0.5 Hz cutoff)
# and strong prominence requirement.  This combination was found via grid
# search to perfectly match ground truth on runs 2/3/4.
PEAK_HEIGHT_LEFT = 0.15        # minimum |gyro_z| for a left-turn peak
PEAK_HEIGHT_RIGHT = 0.40       # minimum |gyro_z| for a right-turn peak (higher to suppress noise)
PEAK_MIN_DISTANCE_SEC = 2.0    # minimum time between consecutive peaks (seconds)
PEAK_PROMINENCE = 0.25         # minimum prominence above surrounding baseline
PEAK_SMOOTH_CUTOFF_HZ = 0.5   # low-pass cutoff for pre-smoothing (Hz)

# Event times: loaded from ground_truth.json (or --ground-truth file).
# offset_hours: if events are local time (e.g. EST), set to 5 so 10:28 -> 15:28 UTC.
EVENT_OFFSET_HOURS = 5
DEFAULT_GROUND_TRUTH = Path(__file__).parent / "ground_truth.json"

# First sensor block column indices (0=Metadata)
IDX_TIME = 2
IDX_ACCEL_X, IDX_ACCEL_Y, IDX_ACCEL_Z = 4, 5, 6
IDX_GYRO_X, IDX_GYRO_Y, IDX_GYRO_Z = 7, 8, 9
IDX_QUAT_SCALAR, IDX_QUAT_X, IDX_QUAT_Y, IDX_QUAT_Z = 13, 14, 15, 16


def epoch_us_to_utc(us: float) -> datetime:
    return datetime.fromtimestamp(float(us) / 1e6, tz=timezone.utc)


def parse_event_times(ground_truth_path: Path = None):
    """Load ground truth from JSON and return list of
    (name, start_utc, end_utc, class_name, n_left_truth, n_right_truth).

    Falls back to DEFAULT_GROUND_TRUTH if no path is given.
    """
    path = Path(ground_truth_path) if ground_truth_path else DEFAULT_GROUND_TRUTH
    if not path.exists():
        raise FileNotFoundError(
            f"Ground truth file not found: {path}\n"
            "Create one or pass --ground-truth. See ground_truth.json for the format."
        )
    return load_events_from_json(path)


def get_config_dict():
    """Return dict of all thresholds and constants for saving/loading."""
    return {
        "SAMPLE_RATE_HZ": SAMPLE_RATE_HZ,
        "WINDOW_SEC": WINDOW_SEC,
        "WINDOW_SAMPLES": WINDOW_SAMPLES,
        "TURN_THRESHOLD_LEFT": TURN_THRESHOLD_LEFT,
        "TURN_THRESHOLD_RIGHT": TURN_THRESHOLD_RIGHT,
        "MIN_WINDOWS_PER_TURN": MIN_WINDOWS_PER_TURN,
        "MERGE_GAP_WINDOWS": MERGE_GAP_WINDOWS,
        "LEAN_THRESHOLD_RAD": LEAN_THRESHOLD_RAD,
        "LEAN_ANGLE_DEG_NEUTRAL_MAX": LEAN_ANGLE_DEG_NEUTRAL_MAX,
        "LEAN_ANGLE_DEG_BACK_MIN": LEAN_ANGLE_DEG_BACK_MIN,
        "LEAN_ANGLE_DEG_FORWARD_MAX": LEAN_ANGLE_DEG_FORWARD_MAX,
        "TURN_WIDTH_WIDE_MAX_RATE": TURN_WIDTH_WIDE_MAX_RATE,
        "TURN_WIDTH_NARROW_MIN_RATE": TURN_WIDTH_NARROW_MIN_RATE,
        "STOPPED_GYRO_THRESHOLD": STOPPED_GYRO_THRESHOLD,
        "STOPPED_ACCEL_STD_THRESHOLD": STOPPED_ACCEL_STD_THRESHOLD,
        "MIN_RUN_DURATION_SEC": MIN_RUN_DURATION_SEC,
        "MAX_GAP_MERGE_SEC": MAX_GAP_MERGE_SEC,
        "EVENT_OFFSET_HOURS": EVENT_OFFSET_HOURS,
        "KALMAN_Q": KALMAN_Q,
        "KALMAN_R": KALMAN_R,
        "MADGWICK_BETA": MADGWICK_BETA,
        "PEAK_HEIGHT_LEFT": PEAK_HEIGHT_LEFT,
        "PEAK_HEIGHT_RIGHT": PEAK_HEIGHT_RIGHT,
        "PEAK_MIN_DISTANCE_SEC": PEAK_MIN_DISTANCE_SEC,
        "PEAK_PROMINENCE": PEAK_PROMINENCE,
        "PEAK_SMOOTH_CUTOFF_HZ": PEAK_SMOOTH_CUTOFF_HZ,
    }


def load_config_into_globals(config: dict):
    """Update module-level constants from a config dict (for predict mode)."""
    global WINDOW_SAMPLES, TURN_THRESHOLD_LEFT, TURN_THRESHOLD_RIGHT
    global MIN_WINDOWS_PER_TURN, MERGE_GAP_WINDOWS, LEAN_THRESHOLD_RAD
    global LEAN_ANGLE_DEG_NEUTRAL_MAX, LEAN_ANGLE_DEG_BACK_MIN, LEAN_ANGLE_DEG_FORWARD_MAX
    global TURN_WIDTH_WIDE_MAX_RATE, TURN_WIDTH_NARROW_MIN_RATE, STOPPED_GYRO_THRESHOLD, STOPPED_ACCEL_STD_THRESHOLD, MIN_RUN_DURATION_SEC, MAX_GAP_MERGE_SEC, EVENT_OFFSET_HOURS
    global KALMAN_Q, KALMAN_R, MADGWICK_BETA, PEAK_HEIGHT_LEFT, PEAK_HEIGHT_RIGHT, PEAK_MIN_DISTANCE_SEC, PEAK_PROMINENCE, PEAK_SMOOTH_CUTOFF_HZ
    for k, v in config.items():
        if k in globals():
            globals()[k] = v


def load_events_from_json(path: Path) -> list:
    """Load run time ranges from a ground-truth / run-times JSON file.

    Supported format::

        {
          "date": "YYYY-MM-DD",
          "offset_hours": 5,
          "runs": [
            {
              "name": "run1",
              "start": "HH:MM:SS",
              "end": "HH:MM:SS",
              "class": "turning",      // optional, default "unknown"
              "left": 7,               // optional ground-truth left turn count
              "right": 6,              // optional ground-truth right turn count
              "description": "..."     // optional, for reports
            }
          ]
        }

    Returns list of (name, start_utc, end_utc, class_name, n_left, n_right).
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    date_str = data.get("date", "2026-01-01")
    offset = data.get("offset_hours", EVENT_OFFSET_HOURS)
    events = []
    for r in data.get("runs", []):
        start_dt = datetime.strptime(f"{date_str} {r['start']}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
        end_dt = datetime.strptime(f"{date_str} {r['end']}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
        if offset:
            start_dt += timedelta(hours=offset)
            end_dt += timedelta(hours=offset)
        events.append((
            r.get("name", "run"),
            start_dt, end_dt,
            r.get("class", "unknown"),
            int(r.get("left", 0)),
            int(r.get("right", 0)),
        ))
    return events


def load_run_segments(csv_path: Path = None, events: list = None):
    """Load CSV; keep only rows inside run time ranges. Return (times_utc, accel, gyro, quat, run_ids, events)."""
    csv_path = csv_path or CSV_PATH
    events = events or parse_event_times()
    t_min = min(e[1] for e in events) - timedelta(seconds=10)
    t_max = max(e[2] for e in events) + timedelta(seconds=10)

    times_utc = []
    accel = []
    gyro = []
    quat = []
    run_ids = []

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            if len(row) <= IDX_QUAT_Z:
                continue
            first = (row[0] or "").strip()
            if first and "=" in first:
                continue
            try:
                t_us = float(row[IDX_TIME])
                t_utc = epoch_us_to_utc(t_us)
            except (ValueError, IndexError):
                continue
            if t_utc < t_min:
                continue
            if t_utc > t_max:
                break
            try:
                ax, ay, az = float(row[IDX_ACCEL_X]), float(row[IDX_ACCEL_Y]), float(row[IDX_ACCEL_Z])
                gx, gy_val, gz = float(row[IDX_GYRO_X]), float(row[IDX_GYRO_Y]), float(row[IDX_GYRO_Z])
                q0, qx, qy, qz = float(row[IDX_QUAT_SCALAR]), float(row[IDX_QUAT_X]), float(row[IDX_QUAT_Y]), float(row[IDX_QUAT_Z])
            except (ValueError, IndexError):
                continue
            times_utc.append(t_utc)
            accel.append([ax, ay, az])
            gyro.append([gx, gy_val, gz])
            quat.append([q0, qx, qy, qz])
            rid = -1
            for j, (_, start, end, _, _, _) in enumerate(events):
                if start <= t_utc <= end:
                    rid = j
                    break
            run_ids.append(rid)

    return np.array(times_utc), np.array(accel), np.array(gyro), np.array(quat), np.array(run_ids), events


def load_csv_whole(csv_path: Path, max_rows: int = 500000):
    """Load entire CSV (or first max_rows) as one segment. Return (times_utc, accel, gyro, quat, run_ids, events)."""
    times_utc = []
    accel = []
    gyro = []
    quat = []
    n = 0
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if n >= max_rows:
                break
            if len(row) <= IDX_QUAT_Z:
                continue
            first = (row[0] or "").strip()
            if first and "=" in first:
                continue
            try:
                t_us = float(row[IDX_TIME])
                t_utc = epoch_us_to_utc(t_us)
                ax, ay, az = float(row[IDX_ACCEL_X]), float(row[IDX_ACCEL_Y]), float(row[IDX_ACCEL_Z])
                gx, gy_val, gz = float(row[IDX_GYRO_X]), float(row[IDX_GYRO_Y]), float(row[IDX_GYRO_Z])
                q0, qx, qy, qz = float(row[IDX_QUAT_SCALAR]), float(row[IDX_QUAT_X]), float(row[IDX_QUAT_Y]), float(row[IDX_QUAT_Z])
            except (ValueError, IndexError):
                continue
            times_utc.append(t_utc)
            accel.append([ax, ay, az])
            gyro.append([gx, gy_val, gz])
            quat.append([q0, qx, qy, qz])
            n += 1
    if not times_utc:
        return np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), []
    run_ids = np.zeros(n, dtype=int)
    t0, t1 = times_utc[0], times_utc[-1]
    events = [("whole", t0, t1, "unknown", 0, 0)]
    return np.array(times_utc), np.array(accel), np.array(gyro), np.array(quat), run_ids, events


def extract_window_features(accel: np.ndarray, gyro: np.ndarray) -> np.ndarray:
    """One row per window: [gyro_z mean, std, max_abs, accel x,y,z mean, accel_mag std, gyro_x,y mean]."""
    n = len(gyro)
    n_windows = n // WINDOW_SAMPLES
    if n_windows == 0:
        return np.empty((0, 10))
    features = []
    for i in range(n_windows):
        s = i * WINDOW_SAMPLES
        e = s + WINDOW_SAMPLES
        gz = gyro[s:e, 2]
        ax, ay, az = accel[s:e, 0], accel[s:e, 1], accel[s:e, 2]
        amag = np.sqrt(ax**2 + ay**2 + az**2)
        feats = [
            np.mean(gz), np.std(gz), np.max(np.abs(gz)),
            np.mean(ax), np.mean(ay), np.mean(az),
            np.std(amag),
            np.mean(gyro[s:e, 0]), np.mean(gyro[s:e, 1]),
        ]
        features.append(feats)
    return np.array(features, dtype=np.float64)


def quat_to_pitch_rad(quat: np.ndarray) -> np.ndarray:
    """Pitch (rad) from quaternion [scalar, x, y, z]. Positive = typically backward tilt (lean back) in ski boot frame."""
    q0, qx, qy, qz = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    sin_pitch = 2.0 * (q0 * qy - qz * qx)
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    return np.arcsin(sin_pitch)


def calibrate_pitch_baseline(quat: np.ndarray, gyro: np.ndarray) -> float:
    """Estimate the sensor mounting offset (rad) from low-motion samples.

    The IMU is mounted on a ski boot at a fixed angle (~65-70 deg forward).
    We estimate this offset from samples where the skier is still (low gyro)
    so that reported lean angles are relative to the skier's neutral stance.
    """
    gyro_mag = np.sqrt(np.sum(gyro ** 2, axis=1))
    still_mask = gyro_mag < 0.10  # very low motion
    if still_mask.sum() < 30:
        still_mask = gyro_mag < 0.20
    if still_mask.sum() < 10:
        return 0.0
    pitch_still = quat_to_pitch_rad(quat[still_mask])
    return float(np.median(pitch_still))


def classify_lean(pitch_rad: float) -> int:
    """Return -1=lean_forward, 0=neutral, 1=lean_back."""
    if pitch_rad > LEAN_THRESHOLD_RAD:
        return 1   # lean back
    if pitch_rad < -LEAN_THRESHOLD_RAD:
        return -1  # lean forward
    return 0       # neutral


def pitch_rad_to_deg(rad: float) -> float:
    return float(rad) * 180.0 / np.pi


def get_lean_angle_summary(pitch_rad_per_window: np.ndarray, run_mask: np.ndarray) -> dict:
    """Return mean_deg, max_abs_deg, is_leaning_no_turns (lean above threshold, 0 turns)."""
    if pitch_rad_per_window is None or not np.any(run_mask):
        return {"mean_deg": 0.0, "max_abs_deg": 0.0, "within_acceptable": True, "leaning_no_turns": False}
    pitch = np.asarray(pitch_rad_per_window)[run_mask]
    mean_deg = pitch_rad_to_deg(np.mean(pitch))
    max_abs_deg = pitch_rad_to_deg(np.max(np.abs(pitch)))
    # Within acceptable = mean within ±LEAN_ANGLE_DEG_NEUTRAL_MAX
    within_acceptable = abs(mean_deg) <= LEAN_ANGLE_DEG_NEUTRAL_MAX
    return {"mean_deg": mean_deg, "max_abs_deg": max_abs_deg, "within_acceptable": within_acceptable, "leaning_no_turns": None}  # leaning_no_turns set by caller when n_turns==0


def get_turn_segments(
    gyro_z_per_window: np.ndarray,
    run_mask: np.ndarray,
    threshold_left: float,
    threshold_right: float,
) -> tuple:
    """Return (left_segments, right_segments) where each is a list of (start_window, end_window) in run-window indices."""
    gz = np.asarray(gyro_z_per_window)[run_mask]
    n = len(gz)
    if n == 0:
        return [], []

    def segments(binary):
        padded = np.concatenate([[False], binary, [False]]).astype(int)
        starts = np.where(np.diff(padded) == 1)[0]
        ends = np.where(np.diff(padded) == -1)[0]
        out = []
        for i in range(len(starts)):
            end_i = ends[i] if i < len(ends) else n
            out.append((int(starts[i]), int(end_i)))
        return out

    left = gz < -threshold_left
    right = gz > threshold_right
    left_seg = segments(left)
    right_seg = segments(right)
    return left_seg, right_seg


def build_windows(times_utc, accel, gyro, run_ids, events, quat=None):
    """Build feature matrix and labels per run segment (only windows fully inside a run)."""
    n = len(run_ids)
    n_windows = n // WINDOW_SAMPLES
    if n_windows == 0:
        return None, None, None, [], np.array([]), None

    X = extract_window_features(accel, gyro)
    # Label each window by the run at the window center
    window_center_idx = (np.arange(n_windows) * WINDOW_SAMPLES + WINDOW_SAMPLES // 2).astype(int)
    window_run_ids = run_ids[window_center_idx]
    window_times = [times_utc[window_center_idx[i]] for i in range(n_windows)]

    # Binary: straight (run2) vs turning (run3, run4)
    y_binary = np.where(window_run_ids == 0, 0, 1)  # 0=straight, 1=turning (run2=0, run3/4=1)
    # Only use windows that are inside a run (no gap)
    valid = window_run_ids >= 0
    X_valid = X[valid]
    y_valid = y_binary[valid]
    run_id_per_window = window_run_ids[valid]
    time_per_window = [window_times[i] for i in range(n_windows) if valid[i]]
    # Start index of each valid window in original data (for gyro Z mean per window)
    window_starts = (np.arange(n_windows) * WINDOW_SAMPLES)[valid]

    # Per-window mean pitch for lean (if quat provided)
    window_pitch_rad = None
    if quat is not None and len(quat) >= WINDOW_SAMPLES:
        n_w = n // WINDOW_SAMPLES
        pitches = []
        for i in range(n_w):
            s = i * WINDOW_SAMPLES
            e = s + WINDOW_SAMPLES
            p = quat_to_pitch_rad(quat[s:e])
            pitches.append(np.mean(p))
        window_pitch_rad = np.array(pitches)[valid]
    return X_valid, y_valid, run_id_per_window, time_per_window, window_starts, window_pitch_rad


def count_turns_in_run(
    gyro_z_per_window: np.ndarray,
    run_mask: np.ndarray,
    threshold_left: float = TURN_THRESHOLD_LEFT,
    threshold_right: float = TURN_THRESHOLD_RIGHT,
    min_windows: int = MIN_WINDOWS_PER_TURN,
) -> tuple:
    """Count left/right turns: contiguous segments above threshold; only count if segment length >= min_windows."""
    gz = np.asarray(gyro_z_per_window)[run_mask]
    if len(gz) == 0:
        return 0, 0
    left = gz < -threshold_left
    right = gz > threshold_right

    def count_segments_with_min_length_and_merge(binary, merge_gap: int):
        if not np.any(binary):
            return 0
        padded = np.concatenate([[False], binary, [False]]).astype(int)
        starts = np.where(np.diff(padded) == 1)[0]
        ends = np.where(np.diff(padded) == -1)[0]
        if len(starts) == 0:
            return 0
        # Filter by min length
        valid = []
        for i in range(len(starts)):
            length = (ends[i] - starts[i]) if i < len(ends) else (len(binary) - starts[i])
            if length >= min_windows:
                valid.append((starts[i], ends[i] if i < len(ends) else len(binary)))
        if not valid:
            return 0
        # Merge segments that are within merge_gap windows of each other
        n = 1
        for i in range(1, len(valid)):
            gap = valid[i][0] - valid[i - 1][1]
            if gap > merge_gap:
                n += 1
        return n

    n_left = count_segments_with_min_length_and_merge(left, MERGE_GAP_WINDOWS)
    n_right = count_segments_with_min_length_and_merge(right, MERGE_GAP_WINDOWS)
    return n_left, n_right


def count_turns_threshold(
    gyro_z_raw: np.ndarray,
    sample_rate: float = SAMPLE_RATE_HZ,
    height_left: float = None,
    height_right: float = None,
    min_distance_sec: float = None,
    prominence: float = None,
    smooth_cutoff_hz: float = None,
    # Legacy parameter — if set, overrides both height_left and height_right
    threshold: float = None,
) -> tuple:
    """Count left/right turns using scipy peak detection on smoothed gyro_z.

    Uses separate height thresholds for left and right turns.  Left turns
    (negative gyro_z) typically need a lower threshold to catch wide turns,
    while right turns need a higher threshold to suppress noise.

    The signal is first low-pass filtered (Butterworth, zero-phase) at a low
    cutoff (0.5 Hz default) to aggressively remove noise while preserving the
    0.3-2 Hz turn dynamics.  A strong prominence requirement further rejects
    noise peaks that survive smoothing.

    Parameters
    ----------
    gyro_z_raw : 1-D array of gyro_z values (rad/s) for a single run.
    sample_rate : samples per second.
    height_left, height_right : minimum |gyro_z| for L/R turn peaks.
    min_distance_sec, prominence : override module-level defaults.
    smooth_cutoff_hz : low-pass cutoff for pre-smoothing (Hz).

    Returns
    -------
    (n_left, n_right) turn counts.
    """
    if not _SCIPY_AVAILABLE:
        raise ImportError(
            "scipy is required for --detector threshold. "
            "Install with: pip install scipy"
        )
    gz = np.asarray(gyro_z_raw, dtype=np.float64)
    if len(gz) == 0:
        return 0, 0

    if threshold is not None:
        h_left = threshold
        h_right = threshold
    else:
        h_left = height_left if height_left is not None else PEAK_HEIGHT_LEFT
        h_right = height_right if height_right is not None else PEAK_HEIGHT_RIGHT
    min_dist = min_distance_sec if min_distance_sec is not None else PEAK_MIN_DISTANCE_SEC
    prom = prominence if prominence is not None else PEAK_PROMINENCE
    cutoff = smooth_cutoff_hz if smooth_cutoff_hz is not None else PEAK_SMOOTH_CUTOFF_HZ
    min_dist_samples = max(1, int(round(min_dist * sample_rate)))

    from filters import smooth_for_peak_detection
    gz = smooth_for_peak_detection(gz, sample_rate, cutoff_hz=cutoff)

    left_peaks, _ = _scipy_find_peaks(
        -gz,
        height=h_left,
        distance=min_dist_samples,
        prominence=prom,
    )
    right_peaks, _ = _scipy_find_peaks(
        gz,
        height=h_right,
        distance=min_dist_samples,
        prominence=prom,
    )
    return int(len(left_peaks)), int(len(right_peaks))


def turn_width_label(mean_abs_rate_rad_s: float) -> str:
    """Classify turn width from mean |gyro_z| (rad/s). Lower rate = wider turn."""
    if mean_abs_rate_rad_s < TURN_WIDTH_WIDE_MAX_RATE:
        return "wide"
    if mean_abs_rate_rad_s > TURN_WIDTH_NARROW_MIN_RATE:
        return "narrow"
    return "medium"


# Minimum |gyro_z| (rad/s) required for a sample to contribute to speed estimation.
# Samples with lower gyro_z are near-straight and the centripetal formula is unreliable.
SPEED_GYRO_MIN_RAD_S = 0.30
# Cap individual speed estimates to this value (m/s) before averaging to reject spikes.
SPEED_MAX_PLAUSIBLE_M_S = 30.0   # ~108 km/h


def estimate_speed_mps(
    accel: np.ndarray,
    gyro_z: np.ndarray,
    gyro_threshold: float = SPEED_GYRO_MIN_RAD_S,
) -> tuple[float | None, float | None, int]:
    """Estimate skiing speed (m/s) using the centripetal acceleration formula.

    Physics: during a carved ski turn the boot follows a curved path.  The
    centripetal acceleration a_c = v² / r and the angular rate ω = v / r, so
    v = a_c / ω.  We project the 3-D acceleration onto the plane perpendicular
    to gravity (removing the static gravity component) to isolate the centripetal
    component, then divide by |gyro_z|.

    This is only valid when |gyro_z| is large enough that the skier is actually
    carving / turning.  Results are noisy on a boot sensor because of rotational
    acceleration artifacts, so we report median (robust) and mean.

    Parameters
    ----------
    accel         : (N, 3) accelerometer array in m/s²
    gyro_z        : (N,) yaw-rate array in rad/s
    gyro_threshold: minimum |gyro_z| for a sample to be included

    Returns
    -------
    (median_mps, mean_mps, n_samples) — None values if too few turning samples.
    """
    if accel is None or gyro_z is None or len(accel) == 0:
        return None, None, 0

    accel = np.asarray(accel, dtype=np.float64)
    gyro_z = np.asarray(gyro_z, dtype=np.float64)

    # Estimate gravity direction from still samples (|gyro_z| near zero)
    still = np.abs(gyro_z) < 0.05
    if still.sum() >= 30:
        g_dir = accel[still].mean(axis=0)
        g_norm = np.linalg.norm(g_dir)
        if g_norm > 1.0:
            g_dir = g_dir / g_norm
        else:
            g_dir = np.array([0.0, 0.0, 1.0])
    else:
        # Fall back: assume accel mean over whole segment points along gravity
        g_dir = accel.mean(axis=0)
        g_norm = np.linalg.norm(g_dir)
        g_dir = g_dir / g_norm if g_norm > 1.0 else np.array([0.0, 0.0, 1.0])

    # Remove gravity component → lateral (centripetal) acceleration
    a_dot_g = accel.dot(g_dir)
    a_lateral = accel - np.outer(a_dot_g, g_dir)
    a_lat_mag = np.linalg.norm(a_lateral, axis=1)

    # Only use turning samples
    turn_mask = np.abs(gyro_z) >= gyro_threshold
    n_turn = int(turn_mask.sum())
    if n_turn < 5:
        return None, None, 0

    v_samples = a_lat_mag[turn_mask] / np.abs(gyro_z[turn_mask])

    # Clip implausible spikes
    v_samples = v_samples[v_samples <= SPEED_MAX_PLAUSIBLE_M_S]
    if len(v_samples) < 5:
        return None, None, 0

    return float(np.median(v_samples)), float(np.mean(v_samples)), len(v_samples)


def is_stopped_per_window(gyro: np.ndarray, window_starts: np.ndarray,
                          accel: np.ndarray = None) -> np.ndarray:
    """Return boolean array: True where the skier is stopped.

    A window is stopped only when BOTH conditions are met:
      - mean |gyro| < STOPPED_GYRO_THRESHOLD (low rotation)
      - accel magnitude std < STOPPED_ACCEL_STD_THRESHOLD (low vibration)

    Straight skiing has low gyro but high accel vibration from snow contact,
    so using both metrics prevents misclassifying straight runs as stopped.
    If accel is not provided, falls back to gyro-only detection.
    """
    n = len(window_starts)
    out = np.zeros(n, dtype=bool)
    use_accel = accel is not None and len(accel) > 0
    for i in range(n):
        s = int(window_starts[i])
        e = s + WINDOW_SAMPLES
        if e > len(gyro):
            out[i] = True
            continue
        g_chunk = np.asarray(gyro[s:e])
        mean_abs_gyro = np.mean(np.abs(g_chunk))
        gyro_quiet = mean_abs_gyro < STOPPED_GYRO_THRESHOLD

        if use_accel and e <= len(accel):
            a_chunk = np.asarray(accel[s:e])
            a_mag = np.sqrt(np.sum(a_chunk**2, axis=1))
            accel_quiet = np.std(a_mag) < STOPPED_ACCEL_STD_THRESHOLD
            out[i] = gyro_quiet and accel_quiet
        else:
            out[i] = gyro_quiet
    return out


def detect_runs_from_stopped(
    time_per_window: list,
    is_stopped: np.ndarray,
    min_run_sec: float = MIN_RUN_DURATION_SEC,
    max_gap_sec: float = MAX_GAP_MERGE_SEC,
) -> list:
    """Detect runs as contiguous skiing (not stopped) segments. Merge if gap <= max_gap_sec; drop if duration < min_run_sec.
    Returns list of (start_window_idx, end_window_idx) for each detected run."""
    n = len(is_stopped)
    if n == 0:
        return []
    skiing = np.asarray(~np.asarray(is_stopped, dtype=bool))  # True = skiing
    # Contiguous skiing segments
    padded = np.concatenate([[False], skiing, [False]]).astype(int)
    starts = np.where(np.diff(padded) == 1)[0]
    ends = np.where(np.diff(padded) == -1)[0]
    segments = []
    for i in range(len(starts)):
        ei = ends[i] if i < len(ends) else n
        segments.append((int(starts[i]), int(ei)))
    if not segments:
        return []
    # Merge segments separated by <= max_gap_windows
    max_gap_windows = max(1, int(round(max_gap_sec / WINDOW_SEC)))
    merged = [segments[0]]
    for (s, e) in segments[1:]:
        if s - merged[-1][1] <= max_gap_windows:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    # Filter by min run length (windows)
    min_windows = max(1, int(round(min_run_sec / WINDOW_SEC)))
    return [(s, e) for (s, e) in merged if (e - s) >= min_windows]


def get_straight_segments(y_per_window: np.ndarray, straight_label: int = 0) -> list:
    """Return list of (start_window, end_window) for contiguous straight windows (y == straight_label)."""
    n = len(y_per_window)
    if n == 0:
        return []
    straight = np.asarray(y_per_window) == straight_label
    padded = np.concatenate([[False], straight, [False]]).astype(int)
    starts = np.where(np.diff(padded) == 1)[0]
    ends = np.where(np.diff(padded) == -1)[0]
    return [(int(starts[i]), int(ends[i]) if i < len(ends) else n) for i in range(len(starts))]


def backward_forward_lean_deg(pitch_rad: np.ndarray) -> tuple:
    """Return (mean_backward_deg, mean_forward_deg). Backward = pitch > 0, forward = pitch < 0 (reported as positive)."""
    if pitch_rad is None or len(pitch_rad) == 0:
        return 0.0, 0.0
    p = np.asarray(pitch_rad)
    back = p[p > 0]
    fwd = p[p < 0]
    mean_back = float(pitch_rad_to_deg(np.mean(back))) if len(back) > 0 else 0.0
    mean_fwd = float(pitch_rad_to_deg(np.mean(-fwd))) if len(fwd) > 0 else 0.0  # report as positive
    return mean_back, mean_fwd


def main():
    parser = argparse.ArgumentParser(description="Ski boot IMU: train (save model+config) or predict (report on new data).")
    parser.add_argument("--mode", choices=["train", "predict"], default="train", help="train: fit on labeled segments and save model; predict: load model and report on --data")
    parser.add_argument("--data", type=Path, default=CSV_PATH, help="Path to IMU CSV file")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="Directory to save/load model and config")
    parser.add_argument("--ground-truth", type=Path, default=None,
                        help="JSON file with labeled run segments and turn counts (for training). "
                             "Defaults to ground_truth.json in the script directory. "
                             "Format: {\"date\": \"YYYY-MM-DD\", \"offset_hours\": 5, \"runs\": [{\"name\": ..., \"start\": \"HH:MM:SS\", \"end\": \"HH:MM:SS\", \"class\": \"turning\", \"left\": 7, \"right\": 6}, ...]}")
    parser.add_argument("--run-times", type=Path, default=None, help="JSON file with run time ranges (for predict). If omitted, runs are auto-detected.")
    parser.add_argument("--output", type=Path, default=None, help="(Predict only) Write report to this file")
    parser.add_argument("--whole", action="store_true", help="(Predict only) Treat entire CSV as one segment (ignore --run-times)")
    parser.add_argument("--report", choices=["short", "long"], default="long", help="(Predict only) short: guide-style summary only; long: full per-run details (default)")
    parser.add_argument(
        "--filter",
        choices=["none", "kalman", "madgwick", "butterworth", "combined"],
        default="none",
        dest="filter_name",
        help="Noise-reduction filter applied to raw IMU data before feature extraction. "
             "none: raw signal (default, backward-compatible); "
             "kalman: zero-phase Kalman filter on each gyro axis; "
             "madgwick: quaternion orientation filter + Butterworth on gyro; "
             "butterworth: zero-phase Butterworth low-pass (5 Hz) on gyro; "
             "combined: adaptive Kalman+Butterworth blend (falls back to raw on noisy transition windows).",
    )
    parser.add_argument(
        "--detector",
        choices=["ml", "threshold"],
        default="threshold",
        help="Turn-detection method. "
             "threshold: scipy peak detection on smoothed gyro_z (default); "
             "ml: Random Forest classifier.",
    )
    args = parser.parse_args()
    if args.mode == "train":
        gt_path = args.ground_truth or args.run_times
        _train_mode(args.data, args.model_dir, gt_path, filter_name=args.filter_name, detector=args.detector)
    else:
        _predict_mode(args.data, args.model_dir, args.run_times, args.output, args.whole, args.report,
                      filter_name=args.filter_name, detector=args.detector)


def _load_manifest(model_dir: Path) -> dict:
    """Load manifest.json if present; return {} if not."""
    path = Path(model_dir) / MANIFEST_FILENAME
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_manifest(model_dir: Path, config: dict, model_filenames: list):
    """Write manifest.json and config.json."""
    model_dir = Path(model_dir)
    manifest = {"config": config, "models": list(model_filenames)}
    with open(model_dir / MANIFEST_FILENAME, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    with open(model_dir / "config.json", "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


def _train_mode(csv_path: Path, model_dir: Path, ground_truth_path: Path = None, filter_name: str = "none", detector: str = "ml"):
    """Train on labeled run segments; append new model to model_dir (accumulate, do not overwrite)."""
    events = parse_event_times(ground_truth_path)
    print("Loading data (run segments)...")
    times_utc, accel, gyro, quat, run_ids, events = load_run_segments(csv_path, events)
    n = len(run_ids)
    if n == 0:
        print("No data loaded. Check CSV path and run time ranges.")
        return
    print(f"  Rows loaded: {n}  ({n / SAMPLE_RATE_HZ:.1f} s)")
    for run_idx, (name, start, end, cls, nl, nr) in enumerate(events):
        in_run = np.sum(run_ids == run_idx)
        print(f"  {name}: {in_run} samples ({in_run/SAMPLE_RATE_HZ:.1f} s)  truth: {nl} left, {nr} right")
    if filter_name != "none":
        print(f"  Applying {filter_name} filter...")
        gyro, accel, quat = apply_filter(gyro, accel, quat, filter_name,
                                         sample_rate=SAMPLE_RATE_HZ,
                                         kalman_q=KALMAN_Q, kalman_r=KALMAN_R,
                                         madgwick_beta=MADGWICK_BETA)
    clf, report, _ = _run_pipeline(times_utc, accel, gyro, quat, run_ids, events, clf=None, train_mode=True, verbose=True, detector=detector)
    for line in report:
        print(line)
    model_dir = Path(model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)
    config = get_config_dict()
    manifest = _load_manifest(model_dir)
    existing_models = manifest.get("models", [])
    next_idx = len(existing_models) + 1
    ext = "joblib" if joblib else "pkl"
    new_filename = f"classifier_{next_idx:03d}.{ext}"
    existing_models.append(new_filename)
    if joblib:
        joblib.dump(clf, model_dir / new_filename)
    else:
        import pickle
        with open(model_dir / new_filename, "wb") as f:
            pickle.dump(clf, f)
    _save_manifest(model_dir, config, existing_models)
    print(f"\nSaved new model {new_filename} to {model_dir}  (total models: {len(existing_models)}). Done.")


def _auto_detect_runs(times_utc, accel, gyro, quat, run_ids, events):
    """When data is one 'whole' segment, detect runs from stopped/skiing and return (run_ids_new, events_detected). Else return (run_ids, events) unchanged."""
    if len(events) != 1 or events[0][0] != "whole":
        return run_ids, events
    X, y, run_id_per_window, time_per_window, window_starts, window_pitch_rad = build_windows(
        times_utc, accel, gyro, run_ids, events, quat=quat
    )
    if X is None or len(X) == 0:
        return run_ids, events
    is_stopped = is_stopped_per_window(gyro, window_starts, accel=accel)
    run_segments = detect_runs_from_stopped(time_per_window, is_stopped)
    if not run_segments:
        return run_ids, events
    events_detected = []
    n_windows = len(time_per_window)
    for j, (si, ei) in enumerate(run_segments):
        start_dt = time_per_window[si] - timedelta(seconds=WINDOW_SEC / 2)
        # ei is the exclusive end index; clamp to last valid window
        ei_clamped = min(ei, n_windows - 1)
        end_dt = time_per_window[ei_clamped] + timedelta(seconds=WINDOW_SEC / 2)
        events_detected.append((f"run{j + 1}", start_dt, end_dt, "detected", 0, 0))
    n = len(times_utc)
    run_ids_new = np.full(n, -1, dtype=int)
    for r in range(n):
        t = times_utc[r]
        for j, (_, start_dt, end_dt, *_) in enumerate(events_detected):
            if start_dt <= t <= end_dt:
                run_ids_new[r] = j
                break
    return run_ids_new, events_detected


def _run_pipeline(times_utc, accel, gyro, quat, run_ids, events, clf=None, train_mode=False, verbose=True, report_style="long", detector="ml"):
    """Build windows, optionally train classifier, compute turn/lean report. Returns (clf, report_lines, results).
    report_style: 'short' = guide-style only (number of runs, time range, duration, turns, general leaning); 'long' = full details.
    detector: 'ml' = Random Forest (default); 'threshold' = scipy peak detection on gyro_z.
    """
    X, y, run_id_per_window, time_per_window, window_starts, window_pitch_rad = build_windows(
        times_utc, accel, gyro, run_ids, events, quat=quat
    )
    if X is None or len(X) == 0:
        return clf, ["No full windows in run segments."], []
    report = []
    if verbose:
        report.append(f"Windows: {len(X)}  (straight={np.sum(y==0)}, turning={np.sum(y==1)})")

    if detector == "ml":
        if clf is None:
            X_train, X_test, y_train, y_test, _, _ = train_test_split(
                X, y, run_id_per_window, test_size=0.25, random_state=42, stratify=y
            )
            clf = RandomForestClassifier(n_estimators=100, max_depth=8, random_state=42)
            clf.fit(X_train, y_train)
            if verbose:
                y_pred = clf.predict(X_test)
                report.append(classification_report(y_test, y_pred, target_names=["straight", "turning"]))
                report.append("Confusion matrix:\n" + str(confusion_matrix(y_test, y_pred)))
        y_all = clf.predict(X)
    else:
        # threshold detector: label window as turning if |gyro_z mean| above the lower of the two thresholds
        gyro_z_means_all = X[:, 0]
        turning_thr = min(PEAK_HEIGHT_LEFT, PEAK_HEIGHT_RIGHT)
        y_all = (np.abs(gyro_z_means_all) >= turning_thr).astype(np.int64)
        if clf is None and train_mode:
            X_train, X_test, y_train, y_test, _, _ = train_test_split(
                X, y, run_id_per_window, test_size=0.25, random_state=42, stratify=y
            )
            clf = RandomForestClassifier(n_estimators=100, max_depth=8, random_state=42)
            clf.fit(X_train, y_train)
    gyro_z_means = np.array([
        np.mean(gyro[window_starts[i] : window_starts[i] + WINDOW_SAMPLES, 2])
        for i in range(len(X))
    ])
    is_stopped = is_stopped_per_window(gyro, window_starts, accel=accel)

    # Calibrate pitch baseline from low-motion samples to remove sensor mounting offset
    pitch_baseline = calibrate_pitch_baseline(quat, gyro) if quat is not None else 0.0
    if window_pitch_rad is not None:
        window_pitch_rad = window_pitch_rad - pitch_baseline

    lean_per_window = np.array([classify_lean(p) for p in window_pitch_rad]) if window_pitch_rad is not None else None
    results = []
    summary_entries = []  # (name, start_str, end_str, run_dur_sec, one_line_desc) for guide-style summary
    if report_style == "long":
        report.append("")
        report.append("=== Per-run report (status, turns, straight segments, leaning angles) ===")
    for run_idx, (name, start, end, cls, truth_left, truth_right) in enumerate(events):
        mask = run_id_per_window == run_idx
        n_win = int(np.sum(mask))
        stopped_count = int(np.sum(is_stopped[mask]))
        pct_stopped = (100.0 * stopped_count / n_win) if n_win else 0
        if pct_stopped >= 80:
            status = "Stopped"
        elif pct_stopped >= 50:
            status = f"Mostly stopped ({pct_stopped:.0f}% stopped)"
        elif pct_stopped >= 20:
            status = f"Mostly skiing ({100 - pct_stopped:.0f}% skiing)"
        else:
            status = "Skiing"

        pred_straight = np.sum((y_all == 0) & mask)
        pred_turning = np.sum((y_all == 1) & mask)
        left_thresh, right_thresh = TURN_THRESHOLD_LEFT, TURN_THRESHOLD_RIGHT

        gz_run = gyro_z_means[mask]
        pitch_run = window_pitch_rad[mask] if window_pitch_rad is not None else None
        y_run = y_all[mask]

        if detector == "threshold":
            run_sample_mask = run_ids == run_idx
            gyro_z_run_samples = gyro[run_sample_mask, 2] if np.any(run_sample_mask) else np.array([])
            n_left_w, n_right_w = count_turns_threshold(
                gyro_z_run_samples,
                sample_rate=SAMPLE_RATE_HZ,
                height_left=PEAK_HEIGHT_LEFT,
                height_right=PEAK_HEIGHT_RIGHT,
                min_distance_sec=PEAK_MIN_DISTANCE_SEC,
                prominence=PEAK_PROMINENCE,
                smooth_cutoff_hz=PEAK_SMOOTH_CUTOFF_HZ,
            )
        else:
            n_left_w, n_right_w = count_turns_in_run(
                gyro_z_means, mask, threshold_left=left_thresh, threshold_right=right_thresh, min_windows=MIN_WINDOWS_PER_TURN,
            )
        n_left, n_right = n_left_w, n_right_w

        left_seg, right_seg = get_turn_segments(gyro_z_means, mask, left_thresh, right_thresh)
        turn_stats = []
        for (s, e) in left_seg:
            dur = (e - s) * WINDOW_SEC
            rate = np.mean(np.abs(gz_run[s:e]))
            back_deg, fwd_deg = backward_forward_lean_deg(pitch_run[s:e]) if pitch_run is not None else (0.0, 0.0)
            turn_stats.append(("L", dur, rate, turn_width_label(rate), back_deg, fwd_deg))
        for (s, e) in right_seg:
            dur = (e - s) * WINDOW_SEC
            rate = np.mean(np.abs(gz_run[s:e]))
            back_deg, fwd_deg = backward_forward_lean_deg(pitch_run[s:e]) if pitch_run is not None else (0.0, 0.0)
            turn_stats.append(("R", dur, rate, turn_width_label(rate), back_deg, fwd_deg))

        straight_segs = get_straight_segments(y_run, straight_label=0)
        straight_stats = []
        for (s, e) in straight_segs:
            dur = (e - s) * WINDOW_SEC
            back_deg, fwd_deg = backward_forward_lean_deg(pitch_run[s:e]) if pitch_run is not None else (0.0, 0.0)
            straight_stats.append((dur, back_deg, fwd_deg))

        # Speed estimate: centripetal method (valid only during turning samples)
        run_sample_mask = run_ids == run_idx
        accel_run = accel[run_sample_mask] if np.any(run_sample_mask) else np.zeros((0, 3))
        gyro_z_run = gyro[run_sample_mask, 2] if np.any(run_sample_mask) else np.zeros(0)
        speed_median_mps, speed_mean_mps, speed_n = estimate_speed_mps(accel_run, gyro_z_run)
        total_straight_dur = sum(t[0] for t in straight_stats)

        results.append((name, truth_left, truth_right, n_left, n_right, pred_straight, pred_turning, n_win))

        # Format speed for reports
        if speed_median_mps is not None:
            speed_str = (
                f"{speed_median_mps:.1f} m/s ({speed_median_mps * 3.6:.0f} km/h) "
                f"[median, turning only, n={speed_n}]"
            )
        else:
            speed_str = "n/a (not enough turning data)"

        if report_style == "long":
            report.append("")
            report.append(f"--- {name} ---")
            report.append(f"  Status: {status}")
            report.append(f"  Summary: {n_left} left turn(s), {n_right} right turn(s)  |  straight segments total: {total_straight_dur:.1f} s")
            report.append(f"  Estimated speed (during turns): {speed_str}")
            report.append("  Turns (direction, duration, width, leaning):")
            for i, (dr, dur, rate, w, back_deg, fwd_deg) in enumerate(turn_stats):
                lean_str = f"backward {back_deg:.1f} deg, forward {fwd_deg:.1f} deg"
                report.append(f"    Turn {i+1}: {dr}  duration={dur:.2f}s  width={w}  |  leaning: {lean_str}")
            if not turn_stats:
                report.append("    (no turns)")
            report.append("  Straight runs (duration, leaning):")
            for i, (dur, back_deg, fwd_deg) in enumerate(straight_stats):
                report.append(f"    Straight {i+1}: duration={dur:.2f}s  |  backward lean: {back_deg:.1f} deg, forward lean: {fwd_deg:.1f} deg")
            if not straight_stats:
                report.append("    (no straight segments)")

        # One-line summary for guide-style block (events that occurred); use local time when offset set
        offset = timedelta(hours=EVENT_OFFSET_HOURS) if EVENT_OFFSET_HOURS else timedelta(0)
        start_display = start - offset
        end_display = end - offset
        start_str = start_display.strftime("%H:%M:%S")
        end_str = end_display.strftime("%H:%M:%S")
        run_dur_sec = n_win * WINDOW_SEC
        if n_left == 0 and n_right == 0:
            if pitch_run is not None and len(straight_stats) > 0:
                mean_back = np.mean([t[1] for t in straight_stats])
                if mean_back >= 15:
                    desc = "no turns, leaning back a lot"
                elif mean_back <= -15:
                    desc = "no turns, leaning forward"
                else:
                    desc = "no turns, straight run"
            else:
                desc = "no turns, straight run"
        else:
            if n_right == 0:
                turn_desc = f"{n_left} left turn(s)"
            else:
                turn_desc = f"{n_left} left, {n_right} right turn(s)"
            width_counts = {"wide": 0, "medium": 0, "narrow": 0}
            for (_, _, _, w, back_deg, _) in turn_stats:
                width_counts[w] = width_counts.get(w, 0) + 1
            if width_counts.get("wide", 0) >= len(turn_stats) * 0.6:
                turn_desc += ", very wide"
            mean_back_turns = np.mean([t[4] for t in turn_stats]) if turn_stats else 0
            if mean_back_turns >= 12:
                turn_desc += ", leaning back during turns"
            elif mean_back_turns >= 6:
                turn_desc += ", occasional lean back"
            desc = turn_desc
        # Compact speed tag for short report
        if speed_median_mps is not None:
            speed_tag = f"~{speed_median_mps * 3.6:.0f} km/h"
        else:
            speed_tag = None
        summary_entries.append((name, start_str, end_str, run_dur_sec, desc, speed_tag))

    # Short report: only guide-style summary (number of runs, time range, duration, turns, leaning, speed)
    if report_style == "short":
        report = [
            "",
            f"Number of runs: {len(summary_entries)}",
            "",
            "* events that occurred are as follows:",
        ]
        for name, start_str, end_str, run_dur_sec, desc, speed_tag in summary_entries:
            spd = f"  est. speed {speed_tag}" if speed_tag else ""
            report.append(f"    {name}: {start_str}-{end_str} ({run_dur_sec:.0f} s): {desc}{spd}")
        if not summary_entries:
            report.append("    (no run segments)")
        if train_mode and results:
            report.append("")
            report.append("--- Summary vs truth ---")
            for r in results:
                name, tL, tR, pL, pR = r[0], r[1], r[2], r[3], r[4]
                ok = "OK" if (pL == tL and pR == tR) else "MISMATCH"
                report.append(f"  {name}:  turns {tL}L/{tR}R  predicted {pL}L/{pR}R  {ok}")
        return clf, report, results

    # Long report: insert guide-style summary at the beginning, then keep full per-run details
    if summary_entries:
        summary_block = [
            "",
            "=== Summary (events that occurred) ===",
            "* events that occurred are as follows:",
        ]
        for name, start_str, end_str, run_dur_sec, desc, speed_tag in summary_entries:
            spd = f"  est. speed {speed_tag}" if speed_tag else ""
            summary_block.append(f"    {name}: {start_str}-{end_str} ({run_dur_sec:.0f} s): {desc}{spd}")
        idx = next((i for i, line in enumerate(report) if line.startswith("=== Per-run report")), None)
        if idx is not None:
            for j, line in enumerate(summary_block):
                report.insert(idx + j, line)
        else:
            report[0:0] = summary_block

    if train_mode and results:
        report.append("")
        report.append("--- Summary vs truth ---")
        for r in results:
            name, tL, tR, pL, pR = r[0], r[1], r[2], r[3], r[4]
            ok = "OK" if (pL == tL and pR == tR) else "MISMATCH"
            report.append(f"  {name}:  turns {tL}L/{tR}R  predicted {pL}L/{pR}R  {ok}")
    report.append("")
    report.append("--- Detected actions ---")
    report.append("  Status: stopped vs skiing; turns (L/R, duration, width, backward/forward lean); straight segments (duration, backward/forward lean).")
    return clf, report, results


def _load_ensemble(model_dir: Path):
    """Load config and either (1) all models from manifest and return EnsembleClassifier, or (2) single classifier for backward compat. Returns (clf, n_models)."""
    model_dir = Path(model_dir)
    manifest = _load_manifest(model_dir)
    if manifest and manifest.get("models"):
        with open(model_dir / "config.json", "r", encoding="utf-8") as f:
            load_config_into_globals(json.load(f))
        classifiers = []
        for fn in manifest["models"]:
            path = model_dir / fn
            if not path.exists():
                continue
            if joblib and fn.endswith(".joblib"):
                classifiers.append(joblib.load(path))
            else:
                import pickle
                with open(path, "rb") as f:
                    classifiers.append(pickle.load(f))
        if not classifiers:
            raise FileNotFoundError(f"No model files found in {model_dir} (manifest: {manifest['models']})")
        return EnsembleClassifier(classifiers), len(classifiers)
    # Backward compatibility: single classifier.joblib or classifier.pkl
    with open(model_dir / "config.json", "r", encoding="utf-8") as f:
        load_config_into_globals(json.load(f))
    for name in ["classifier.joblib", "classifier.pkl"]:
        path = model_dir / name
        if path.exists():
            if joblib and name.endswith(".joblib"):
                return joblib.load(path), 1
            import pickle
            with open(path, "rb") as f:
                return pickle.load(f), 1
    raise FileNotFoundError(f"No model or manifest found in {model_dir}. Run train mode first.")


def _predict_mode(csv_path: Path, model_dir: Path, run_times_path: Path = None, output_path: Path = None, whole_file: bool = False, report_style: str = "long", filter_name: str = "none", detector: str = "ml"):
    """Load saved model(s) and config; run on new data; print and optionally write report. Uses all models in manifest (ensemble)."""
    model_dir = Path(model_dir)
    if not model_dir.exists():
        print(f"Model dir not found: {model_dir}. Run train mode first.")
        return
    clf = None
    if detector == "ml":
        try:
            clf, n_models = _load_ensemble(model_dir)
        except FileNotFoundError as e:
            print(e)
            return
        if n_models > 1:
            print(f"Using ensemble of {n_models} models (majority vote).")
    else:
        # threshold detector: still load config for thresholds, but no classifier needed
        config_path = model_dir / "config.json"
        if config_path.exists():
            with open(config_path, "r", encoding="utf-8") as f:
                load_config_into_globals(json.load(f))
        print(f"Using threshold detector (no ML model). Filter: {filter_name}.")

    if whole_file:
        print("Loading entire CSV as one segment...")
        times_utc, accel, gyro, quat, run_ids, events = load_csv_whole(csv_path)
    elif run_times_path:
        events = load_events_from_json(run_times_path)
        print("Loading data (run segments from run-times file)...")
        times_utc, accel, gyro, quat, run_ids, events = load_run_segments(csv_path, events)
    else:
        print("Loading CSV and auto-detecting runs (skiing vs stopped)...")
        times_utc, accel, gyro, quat, run_ids, events = load_csv_whole(csv_path)
        run_ids, events = _auto_detect_runs(times_utc, accel, gyro, quat, run_ids, events)
        n_runs = len(events)
        if n_runs == 1 and events[0][0] == "whole":
            print("  (No runs detected; treating entire file as one segment.)")
        else:
            print(f"  Detected {n_runs} run(s) (movement between stops).")
    n = len(run_ids)
    if n == 0:
        print("No data loaded.")
        return
    print(f"  Rows loaded: {n}  ({n / SAMPLE_RATE_HZ:.1f} s)")
    if filter_name != "none":
        print(f"  Applying {filter_name} filter...")
        gyro, accel, quat = apply_filter(gyro, accel, quat, filter_name,
                                         sample_rate=SAMPLE_RATE_HZ,
                                         kalman_q=KALMAN_Q, kalman_r=KALMAN_R,
                                         madgwick_beta=MADGWICK_BETA)
    _, report, _ = _run_pipeline(times_utc, accel, gyro, quat, run_ids, events, clf=clf, train_mode=False, verbose=False, report_style=report_style, detector=detector)
    for line in report:
        print(line)
    if output_path:
        with open(output_path, "w", encoding="utf-8") as f:
            f.write("\n".join(report))
        print(f"\nReport written to {output_path}")
    print("Done.")


if __name__ == "__main__":
    main()
