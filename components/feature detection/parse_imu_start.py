#!/usr/bin/env python3
"""
Parse the beginning of the ski boot IMU CSV only (file is large).
- Skip Metadata column and metadata header rows.
- Convert Time from Unix epoch (microseconds) to UTC to match known events.
- Summarize structure and align with run 2, 3, 4 events for ML design.
"""

import csv
from datetime import datetime, timezone
from pathlib import Path

CSV_PATH = Path(__file__).parent / "data" / "2026_02_10_skiway_sensor_data.csv"

# Known events (UTC), from guide.md
EVENTS_UTC = [
    ("Run 2", "10:28:32", "10:29:10", "no turns, straight down, leaning back a lot"),
    ("Run 3", "10:32:37", "10:33:08", "7 left, 6 right turns; 7th left is stop"),
    ("Run 4", "10:39:38", "10:40:31", "5 left turns, wide, occasional lean backs"),
]

# Parse only first N data rows to keep runtime small
MAX_DATA_ROWS = 20000


def epoch_us_to_utc(us: float) -> datetime:
    """Convert epoch microseconds to UTC datetime."""
    return datetime.fromtimestamp(float(us) / 1e6, tz=timezone.utc)


def main():
    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)

    # Skip "Metadata" for analysis; use first sensor block only for time/simple stats
    # Columns: 0=Metadata, 1=SyncCount, 2=Time, 3=Button, 4-6=Accel XYZ, 7-9=Gyro XYZ, ...
    idx_time = 2
    idx_accel_x, idx_accel_y, idx_accel_z = 4, 5, 6
    idx_gyro_x, idx_gyro_y, idx_gyro_z = 7, 8, 9

    times_utc = []
    accel_z = []
    gyro_z = []
    data_rows = 0

    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            if len(row) <= idx_gyro_z:
                continue
            first = (row[0] or "").strip()
            # Skip metadata rows (e.g. "File Format Version=6")
            if first and "=" in first:
                continue
            try:
                t_us = float(row[idx_time])
                times_utc.append(epoch_us_to_utc(t_us))
                accel_z.append(float(row[idx_accel_z]))
                gyro_z.append(float(row[idx_gyro_z]))
            except (ValueError, IndexError):
                continue
            data_rows += 1
            if data_rows >= MAX_DATA_ROWS:
                break

    if not times_utc:
        print("No data rows found.")
        return

    t0, t1 = times_utc[0], times_utc[-1]
    date_str = t0.strftime("%Y-%m-%d")
    print("=== IMU data (beginning only) ===\n")
    print(f"Data rows parsed: {len(times_utc)} (max {MAX_DATA_ROWS})")
    print(f"Time range (UTC): {t0.strftime('%H:%M:%S.%f')[:-3]} -> {t1.strftime('%H:%M:%S.%f')[:-3]} on {date_str}")
    print(f"Duration: {(t1 - t0).total_seconds():.1f} s")
    print()

    # Compare to known events (same calendar day assumed)
    # If event times in guide are local (e.g. EST), 10:28 local = 15:28 UTC → runs appear later in file
    print("=== Known events (guide: UTC; if actually local, runs align later in CSV) ===")
    for name, start, end, desc in EVENTS_UTC:
        print(f"  {name}: {start} - {end}  ({desc})")
    print()

    # Check if parsed window overlaps any event
    print("=== Overlap with parsed window ===")
    for name, start, end, _ in EVENTS_UTC:
        start_t = datetime.strptime(f"{date_str} {start}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
        end_t = datetime.strptime(f"{date_str} {end}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
        if t1 >= start_t and t0 <= end_t:
            print(f"  Parsed window OVERLAPS {name}")
        else:
            print(f"  Parsed window does not overlap {name}")
    print()

    # Simple stats on first sensor (sensor 1), Accel Z and Gyro Z
    print("=== Sensor 1 (first block) — Accel Z & Gyro Z (beginning) ===")
    print(f"  Accel Z (m/s^2): min={min(accel_z):.2f}, max={max(accel_z):.2f}, mean={sum(accel_z)/len(accel_z):.2f}")
    print(f"  Gyro Z (rad/s):  min={min(gyro_z):.3f}, max={max(gyro_z):.3f}, mean={sum(gyro_z)/len(gyro_z):.3f}")
    print("  (Accel Z ~ gravity when upright; turns show in gyro and lateral accel.)")
    print()

    # Column layout reminder
    print("=== CSV layout (first sensor block only; Metadata column skipped for ML) ===")
    print("  Time: epoch microseconds (column 2). Convert to UTC for event alignment.")
    print("  Accel X/Y/Z, Gyro X/Y/Z are primary turn/lean signals.")
    print("  Quaternion = orientation; can derive lean and turn direction.")


if __name__ == "__main__":
    main()
