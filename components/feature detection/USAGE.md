# Ski Boot IMU — Quick Usage Guide

## run_report.py — Analyze new data

Auto-detects runs (skiing vs stopped) and reports turns, lean angles, speed.

```bash
# Default: auto-detect runs, threshold detector, long report
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv

# Short summary only
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv --report short

# Save report to file
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv --output report.txt

# Use ML (Random Forest) detector instead of threshold
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv --detector ml

# Apply a noise filter before analysis
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv --filter butterworth

# Treat entire file as one segment (skip run detection)
python run_report.py --data data/2026_02_10_skiway_sensor_data.csv --whole
```

### Options

| Flag | Values | Default | Description |
|------|--------|---------|-------------|
| `--data` | path | *(required)* | IMU CSV file |
| `--report` | `short`, `long` | `long` | Summary only vs full per-run details |
| `--detector` | `threshold`, `ml` | `threshold` | Peak detection or Random Forest |
| `--filter` | `none`, `kalman`, `butterworth`, `madgwick`, `combined` | `none` | Noise filter on gyro/accel |
| `--output` | path | *(stdout)* | Write report to file |
| `--model-dir` | path | `saved_model/` | Directory with config and trained models |
| `--whole` | flag | off | Treat entire CSV as one segment |
| `--run-times` | path | *(auto-detect)* | JSON with known run time ranges |

## ml_pipeline.py — Train on labeled data

Trains a model on labeled run segments and saves it for later use by `run_report.py`.

```bash
# Train using the default ground truth file (ground_truth.json)
python ml_pipeline.py --mode train --data data/2026_02_10_skiway_sensor_data.csv

# Train with a custom ground truth file (e.g. new observations)
python ml_pipeline.py --mode train --data new_session.csv --ground-truth new_ground_truth.json

# Train and save to a custom directory
python ml_pipeline.py --mode train --data data/2026_02_10_skiway_sensor_data.csv --model-dir my_model

# Predict mode (same as run_report.py)
python ml_pipeline.py --mode predict --data data/2026_02_10_skiway_sensor_data.csv
```

Training accumulates models — each `--mode train` run adds a new classifier to the ensemble. Prediction uses majority vote across all saved models.

## ground_truth.json — Labeled observations

Ground truth is stored in a JSON file (default: `ground_truth.json`). Add new observations here to train on additional data without changing code.

```json
{
  "date": "2026-02-10",
  "offset_hours": 5,
  "runs": [
    {
      "name": "run1",
      "start": "10:28:32",
      "end": "10:29:10",
      "class": "straight",
      "left": 0,
      "right": 0,
      "description": "no turns, leaning back a lot"
    },
    {
      "name": "run2",
      "start": "10:32:37",
      "end": "10:33:08",
      "class": "turning",
      "left": 7,
      "right": 6,
      "description": "7 left turns, 6 right turns"
    }
  ]
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `date` | yes | Date of the session (`YYYY-MM-DD`) |
| `offset_hours` | yes | Hours to add to convert local time to UTC (e.g. 5 for EST) |
| `runs[].name` | yes | Label for the run |
| `runs[].start` | yes | Start time (`HH:MM:SS`, local) |
| `runs[].end` | yes | End time (`HH:MM:SS`, local) |
| `runs[].class` | no | `"straight"` or `"turning"` (default `"unknown"`) |
| `runs[].left` | no | Ground truth left turn count (default 0) |
| `runs[].right` | no | Ground truth right turn count (default 0) |
| `runs[].description` | no | Free-text notes |

## Requirements

```
pip install -r requirements.txt
```

Needs: numpy, scikit-learn, scipy, joblib.
