#!/usr/bin/env python3
"""
Run the ski boot IMU report on new data using a previously trained model.
Runs are auto-detected (movement between stops) unless you pass --run-times or --whole.

Example:
  python run_report.py --data new_session.csv --model-dir saved_model --output report.txt
  python run_report.py --data new_session.csv --report short --output short.txt
  python run_report.py --data new_session.csv --model-dir saved_model --run-times runs.json
  python run_report.py --data new_session.csv --model-dir saved_model --whole
  python run_report.py --data new_session.csv --run-times runs.json --filter kalman --detector threshold
  python run_report.py --data new_session.csv --run-times runs.json --filter madgwick --detector threshold
"""

import argparse
from pathlib import Path

# Invoke ml_pipeline in predict mode
from ml_pipeline import _predict_mode

DEFAULT_MODEL_DIR = Path(__file__).parent / "saved_model"


def main():
    p = argparse.ArgumentParser(description="Run IMU report on new data using saved model.")
    p.add_argument("--data", type=Path, required=True, help="Path to IMU CSV file")
    p.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="Directory with config.json and classifier.joblib (or .pkl)")
    p.add_argument("--run-times", type=Path, default=None, help="Optional JSON with run time ranges. If omitted, runs are auto-detected (skiing vs stopped). Use --whole to treat the entire file as one segment.")
    p.add_argument("--output", type=Path, default=None, help="Write report to this file")
    p.add_argument("--whole", action="store_true", help="Treat entire CSV as one segment (ignore --run-times)")
    p.add_argument("--report", choices=["short", "long"], default="long", help="short: guide-style summary only; long: full per-run details (default)")
    p.add_argument(
        "--filter",
        choices=["none", "kalman", "madgwick", "butterworth", "combined"],
        default="none",
        dest="filter_name",
        help="Noise-reduction filter: none (default), kalman, madgwick, butterworth, or combined.",
    )
    p.add_argument(
        "--detector",
        choices=["ml", "threshold"],
        default="threshold",
        help="Turn-detection method: threshold (scipy peak detection, default) or ml (Random Forest).",
    )
    args = p.parse_args()
    _predict_mode(args.data, args.model_dir, args.run_times, args.output, args.whole, args.report,
                  filter_name=args.filter_name, detector=args.detector)


if __name__ == "__main__":
    main()
