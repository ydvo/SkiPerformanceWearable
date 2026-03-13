#!/usr/bin/env python3
"""
compare_filters.py — Run all filter + detector combinations on the labeled runs
and print a comparison table vs ground truth.

This script is self-contained: it calls the ml_pipeline internals directly so
no subprocess or second Python process is needed.

Usage
-----
# Default: use built-in run boundaries and the sample CSV
python compare_filters.py

# Use a custom run-times JSON
python compare_filters.py --run-times run_times_example.json

# Use a different CSV
python compare_filters.py --data data/2026_02_10_skiway_sensor_data.csv --run-times run_times_example.json

# Also save per-combo short reports to files
python compare_filters.py --save-reports

Output
------
A table like:

  filter     detector  run2 (truth 0L/0R)   run3 (truth 7L/6R)   run4 (truth 5L/0R)   total_err
  ---------  --------  -------------------  -------------------  -------------------  ---------
  none       ml        0L/0R  ✓             7L/6R  ✓             5L/0R  ✓             0
  none       threshold 0L/0R  ✓             8L/5R  ✗             6L/1R  ✗             4
  kalman     ml        0L/0R  ✓             7L/6R  ✓             5L/0R  ✓             0
  kalman     threshold 0L/0R  ✓             7L/6R  ✓             5L/0R  ✓             0
  madgwick   ml        ...
  madgwick   threshold ...

And an SNR table showing the signal-to-noise improvement per filter.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Import pipeline internals
# ---------------------------------------------------------------------------
try:
    import ml_pipeline as _ml
    from filters import apply_filter
except ImportError as exc:
    print(f"ERROR: Cannot import pipeline modules: {exc}")
    print("Make sure you run this script from the skiboot project directory.")
    sys.exit(1)

# Ground truth per run (index matches RUNS order in ml_pipeline.py)
# Format: (name, n_left_truth, n_right_truth)
TRUTH = [
    ("run2", 0, 0),
    ("run3", 7, 6),
    ("run4", 5, 0),
]

COMBOS = [
    ("none",        "ml"),
    ("none",        "threshold"),
    ("kalman",      "ml"),
    ("kalman",      "threshold"),
    ("butterworth", "ml"),
    ("butterworth", "threshold"),
    ("madgwick",    "ml"),
    ("madgwick",    "threshold"),
    ("combined",    "ml"),
    ("combined",    "threshold"),
]


def _run_combo(
    filter_name: str,
    detector: str,
    times_utc,
    accel_raw,
    gyro_raw,
    quat_raw,
    run_ids,
    events,
    clf,
) -> list[tuple]:
    """Run one filter+detector combination.  Returns list of (n_left, n_right) per run."""
    accel = accel_raw.copy()
    gyro  = gyro_raw.copy()
    quat  = quat_raw.copy()

    if filter_name != "none":
        gyro, accel, quat = apply_filter(
            gyro, accel, quat, filter_name,
            sample_rate=_ml.SAMPLE_RATE_HZ,
            kalman_q=_ml.KALMAN_Q,
            kalman_r=_ml.KALMAN_R,
            madgwick_beta=_ml.MADGWICK_BETA,
        )

    _, report_lines, results = _ml._run_pipeline(
        times_utc, accel, gyro, quat, run_ids, events,
        clf=clf,
        train_mode=False,
        verbose=False,
        report_style="short",
        detector=detector,
    )
    # results: list of (name, truth_left, truth_right, n_left, n_right, ...)
    return [(r[3], r[4]) for r in results], report_lines


def _snr(gyro_z: np.ndarray, run_ids: np.ndarray, events: list, filter_name: str,
         accel: np.ndarray, quat: np.ndarray) -> dict:
    """Compute a simple SNR proxy per run: peak |gyro_z| during turns / mean |gyro_z| during straight."""
    g = gyro_z.copy()
    if filter_name != "none":
        gyro_full = np.zeros((len(g), 3))
        gyro_full[:, 2] = g
        accel_full = accel.copy()
        quat_full = quat.copy()
        gyro_full, _, _ = apply_filter(
            gyro_full, accel_full, quat_full, filter_name,
            sample_rate=_ml.SAMPLE_RATE_HZ,
            kalman_q=_ml.KALMAN_Q,
            kalman_r=_ml.KALMAN_R,
            madgwick_beta=_ml.MADGWICK_BETA,
        )
        g = gyro_full[:, 2]

    snr = {}
    # run2 = straight reference; run3/4 = turning
    run2_mask = run_ids == 0
    noise_level = np.std(g[run2_mask]) if np.any(run2_mask) else 1.0
    for idx, (name, *_) in enumerate(events):
        m = run_ids == idx
        if not np.any(m):
            snr[name] = float("nan")
            continue
        signal = np.max(np.abs(g[m]))
        snr[name] = round(signal / noise_level, 2) if noise_level > 0 else float("nan")
    return snr


def main():
    parser = argparse.ArgumentParser(description="Compare filter+detector combinations against ground truth.")
    parser.add_argument("--data", type=Path, default=_ml.CSV_PATH, help="Path to IMU CSV")
    parser.add_argument("--run-times", type=Path, default=None, help="JSON run-times file (optional; uses built-in if omitted)")
    parser.add_argument("--model-dir", type=Path, default=_ml.DEFAULT_MODEL_DIR)
    parser.add_argument("--save-reports", action="store_true", help="Write short reports per combo to compare_<filter>_<detector>.txt")
    args = parser.parse_args()

    # --- Load events ---
    if args.run_times:
        events = _ml.load_events_from_json(args.run_times)
    else:
        events = _ml.parse_event_times()

    # Truth per run: use built-in TRUTH table when run names match (has correct counts);
    # fall back to events[i][4]/[5] which may be 0 if loaded from a json without truth counts.
    truth_by_name = {name: (tL, tR) for name, tL, tR in TRUTH}
    truth_list = []
    for i, ev in enumerate(events):
        name = ev[0]
        if name in truth_by_name:
            tL, tR = truth_by_name[name]
        else:
            tL, tR = ev[4], ev[5]
        truth_list.append((name, tL, tR))

    print("Loading IMU data (run segments)...")
    times_utc, accel_raw, gyro_raw, quat_raw, run_ids, events = _ml.load_run_segments(args.data, events)
    n = len(run_ids)
    if n == 0:
        print("No data loaded. Check CSV path and run-times.")
        sys.exit(1)
    print(f"  Rows: {n}  ({n / _ml.SAMPLE_RATE_HZ:.1f} s)")

    # --- Load classifier (for ml detector combos) ---
    clf = None
    if args.model_dir.exists():
        try:
            clf, n_models = _ml._load_ensemble(args.model_dir)
            print(f"  Classifier loaded ({n_models} model(s)).")
        except FileNotFoundError:
            print("  No trained classifier found — 'ml' detector combos will train on-the-fly (may differ from saved model).")
    else:
        print("  model-dir not found — 'ml' detector combos will train on-the-fly.")

    # --- Run all combos ---
    rows = []
    all_reports = {}
    print("\nRunning all filter x detector combinations...")
    for filter_name, detector in COMBOS:
        label = f"{filter_name:10s} {detector:9s}"
        try:
            preds, report_lines = _run_combo(
                filter_name, detector,
                times_utc, accel_raw, gyro_raw, quat_raw,
                run_ids, events, clf,
            )
        except Exception as exc:
            preds = [(None, None)] * len(truth_list)
            report_lines = [f"ERROR: {exc}"]
            print(f"  {label} — ERROR: {exc}")

        total_err = 0
        run_cols = []
        for i, (name, tL, tR) in enumerate(truth_list):
            if i < len(preds):
                pL, pR = preds[i]
            else:
                pL, pR = None, None
            if pL is None:
                run_cols.append(f"ERROR")
                continue
            err = abs(pL - tL) + abs(pR - tR)
            total_err += err
            match = "OK" if err == 0 else "!!"
            run_cols.append(f"{pL}L/{pR}R {match}")
        rows.append((filter_name, detector, run_cols, total_err))
        all_reports[(filter_name, detector)] = report_lines

    # --- Print comparison table ---
    print()
    print("=" * 90)
    print("FILTER x DETECTOR COMPARISON vs GROUND TRUTH")
    print("=" * 90)

    # Header
    hdr_runs = "  ".join(f"{name} (truth {tL}L/{tR}R)".ljust(22) for name, tL, tR in truth_list)
    print(f"{'Filter':<12}{'Detector':<11}{hdr_runs}  {'Total Err':>9}")
    print("-" * 90)
    for filter_name, detector, run_cols, total_err in rows:
        cols_str = "  ".join(c.ljust(22) for c in run_cols)
        print(f"{filter_name:<12}{detector:<11}{cols_str}  {total_err:>9}")

    print()
    print("Legend: OK = matches truth exactly  |  !! = mismatch  |  Total Err = sum of |predicted - truth| across all runs")

    # --- SNR table ---
    print()
    print("=" * 60)
    print("SIGNAL-TO-NOISE RATIO (peak |gyro_z| in turn / std of straight run)")
    print("(Higher = cleaner signal relative to noise baseline)")
    print("=" * 60)
    print(f"{'Filter':<12}", end="")
    for name, *_ in truth_list:
        print(f"  {name:<10}", end="")
    print()
    print("-" * 60)
    for filter_name in ["none", "kalman", "butterworth", "madgwick", "combined"]:
        snr = _snr(gyro_raw[:, 2], run_ids, events, filter_name, accel_raw, quat_raw)
        print(f"{filter_name:<12}", end="")
        for name, *_ in truth_list:
            v = snr.get(name, float("nan"))
            print(f"  {v:<10}", end="")
        print()

    # --- Save reports if requested ---
    if args.save_reports:
        print()
        for (filter_name, detector), lines in all_reports.items():
            fname = f"compare_{filter_name}_{detector}.txt"
            with open(fname, "w", encoding="utf-8") as f:
                f.write(f"# Filter: {filter_name}  Detector: {detector}\n")
                f.write("\n".join(lines))
            print(f"  Saved: {fname}")

    print("\nDone.")


if __name__ == "__main__":
    main()
