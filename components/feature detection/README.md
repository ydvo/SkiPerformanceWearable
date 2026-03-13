# Ski Boot IMU Analysis

This project processes inertial measurement unit (IMU) sensor data from a ski boot to automatically detect skiing runs, count left/right turns, classify turn width, estimate speed, and report forward/backward lean angles. It uses a combination of signal processing (Butterworth filtering, peak detection) and machine learning (Random Forest ensemble) to analyze accelerometer, gyroscope, and quaternion data.

To get started, run `python run_report.py --data data/<your_csv>.csv` for automatic analysis, or `python ml_pipeline.py --mode train --ground-truth ground_truth.json` to train on labeled observations. See `USAGE.md` for full command reference and ground truth file format.
