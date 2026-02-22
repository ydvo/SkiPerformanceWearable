#!/usr/bin/env python3
"""
H5toCsv.py — Convert APDM Opal IMU .h5 files to the CSV format expected by ml_pipeline.py.

Actual APDM Opal HDF5 structure (observed in sample files):
  Top-level groups: <sensor_id>  (e.g. "BE-002391"), plus "Annotations"
  /<sensor_id>/Time                       — Unix timestamps, microseconds (uint64)
  /<sensor_id>/SyncValue                  — Sync counter (uint64)
  /<sensor_id>/ButtonStatus               — Button state (uint8)
  /<sensor_id>/Calibrated/Accelerometers  — (N, 3) m/s²     [X, Y, Z]
  /<sensor_id>/Calibrated/Gyroscopes      — (N, 3) rad/s     [X, Y, Z]
  /<sensor_id>/Calibrated/Magnetometers   — (N, 3) µT        [X, Y, Z]
  /<sensor_id>/Calibrated/Orientation     — (N, 4) quaternion [scalar, X, Y, Z]
  /<sensor_id>/Calibrated/Temperature     — (N,)  °C
  /<sensor_id> attrs: LocalTimeOffset (µs from UTC), Timezone, SampleRate, ...

  Note: Some files contain a SINGLE sensor group; others contain ALL sensors
  together in one file. The converter handles both cases.

Output CSV matches the column order that ml_pipeline.py expects:
  Metadata, Sync Count, Time, Button Status,
  Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2),
  Angular Velocity X (rad/s), Angular Velocity Y (rad/s), Angular Velocity Z (rad/s),
  Magnetic Field X (uT), Magnetic Field Y (uT), Magnetic Field Z (uT),
  Orientation Quaternion Scalar, Orientation Quaternion X, Orientation Quaternion Y, Orientation Quaternion Z,
  Temperature (deg C) Z, Pressure (Pa)
  [repeated for each additional sensor if --all-sensors is used]

Usage
-----
  # Inspect what sensors are inside an h5 file
  python H5toCsv.py recording.h5 --list-sensors

  # Convert (auto-name output, uses first sensor only)
  python H5toCsv.py recording.h5

  # Specify output filename
  python H5toCsv.py recording.h5 --output my_session.csv

  # Include all sensors (extra column blocks, matching original multi-sensor CSV)
  python H5toCsv.py recording.h5 --all-sensors --output session.csv

  # Merge multiple single-sensor h5 files into one CSV (time-aligned)
  python H5toCsv.py file1.h5 file2.h5 file3.h5 --merge --output merged.csv

  # After conversion, run the existing pipeline:
  python run_report.py --data session.csv --run-times run_times.json --report short
  python run_report.py --data session.csv --filter kalman --detector threshold --report short

Dependencies
------------
  pip install h5py numpy   (h5py is the only new dependency)
"""

import argparse
import csv
import sys
from datetime import datetime, timezone, timedelta
from pathlib import Path

import numpy as np

try:
    import h5py
except ImportError:
    print(
        "ERROR: h5py is required.\n"
        "Install with:  pip install h5py\n"
        "or:            conda install -c anaconda h5py"
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _decode(value) -> str:
    """Decode bytes/np.bytes_ to str; return str unchanged."""
    if isinstance(value, (bytes, np.bytes_)):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _sensor_ids(f) -> list:
    """Return sensor group IDs (top-level keys that are not 'Annotations')."""
    return [k for k in f.keys() if k != "Annotations"]


def _read_ds(grp, key, n_rows, n_cols):
    """Read a dataset; return zero-filled array if absent."""
    if key in grp:
        data = grp[key][()].astype(np.float64)
        if data.ndim == 1:
            return data.reshape(-1, 1)
        return data
    return np.zeros((n_rows, n_cols if n_cols > 1 else 1))


# ---------------------------------------------------------------------------
# Single-file sensor reader
# ---------------------------------------------------------------------------

def list_sensors(h5_path: Path):
    """Print sensor IDs found in the file along with basic info."""
    with h5py.File(h5_path, "r") as f:
        sids = _sensor_ids(f)
        if not sids:
            print(f"No sensor groups found in {h5_path.name}.")
            print(f"  Top-level keys: {list(f.keys())}")
            return
        print(f"Found {len(sids)} sensor(s) in {h5_path.name}:")
        for sid in sids:
            g = f[sid]
            rows = g["Time"].shape[0] if "Time" in g else "?"
            sr   = g.attrs.get("SampleRate", "?")
            tz   = _decode(g.attrs.get("Timezone", ""))
            cal_keys = list(g.get("Calibrated", {}).keys()) if "Calibrated" in g else []
            print(f"  {sid!r:20s}  rows={rows}  sample_rate={sr}  timezone={tz!r}  calibrated={cal_keys}")


def read_sensor_from_file(f, sid: str) -> dict:
    """Read one sensor group from an open h5py.File object."""
    g = f[sid]

    if "Time" not in g:
        raise KeyError(f"Sensor {sid!r}: no 'Time' dataset found.")

    cal = g.get("Calibrated", None)
    if cal is None:
        raise KeyError(f"Sensor {sid!r}: no 'Calibrated' group found.")
    if "Accelerometers" not in cal:
        raise KeyError(f"Sensor {sid!r}: no 'Calibrated/Accelerometers' dataset found.")
    if "Gyroscopes" not in cal:
        raise KeyError(f"Sensor {sid!r}: no 'Calibrated/Gyroscopes' dataset found.")

    time_us  = g["Time"][()].astype(np.int64)
    sync     = g["SyncValue"][()].astype(np.int64) if "SyncValue" in g else np.arange(len(time_us), dtype=np.int64)
    button   = g["ButtonStatus"][()].astype(np.int32) if "ButtonStatus" in g else np.zeros(len(time_us), dtype=np.int32)
    n        = len(time_us)

    accel = cal["Accelerometers"][()].astype(np.float64)   # (N,3) m/s²
    gyro  = cal["Gyroscopes"][()].astype(np.float64)        # (N,3) rad/s
    mag   = _read_ds(cal, "Magnetometers", n, 3)            # (N,3) µT
    quat  = _read_ds(cal, "Orientation",   n, 4)            # (N,4)
    temp  = _read_ds(cal, "Temperature",   n, 1).flatten()  # (N,)
    pres  = np.zeros(n)  # Barometer not present in these files

    # LocalTimeOffset is stored in MICROSECONDS (confirmed from attrs)
    local_offset_us = int(g.attrs.get("LocalTimeOffset", 0))
    tz_name = _decode(g.attrs.get("Timezone", ""))
    sr = int(g.attrs.get("SampleRate", 128))

    return {
        "sid":              sid,
        "label":            sid,          # sensor ID is the label in this format
        "tz_name":          tz_name,
        "local_offset_us":  local_offset_us,
        "sample_rate":      sr,
        "time_us":          time_us,
        "sync":             sync,
        "button":           button,
        "accel":            accel,
        "gyro":             gyro,
        "mag":              mag,
        "quat":             quat,
        "temp":             temp,
        "pres":             pres,
    }


# ---------------------------------------------------------------------------
# Multi-file merge helper
# ---------------------------------------------------------------------------

def merge_single_sensor_files(h5_paths: list[Path]) -> list[dict]:
    """Load one sensor from each file and return a list of sensor dicts.

    Each file in h5_paths should contain exactly one sensor group.
    If a file contains multiple sensors, the first one is used.
    """
    sensors = []
    for p in h5_paths:
        with h5py.File(p, "r") as f:
            sids = _sensor_ids(f)
            if not sids:
                print(f"WARNING: no sensor groups in {p.name} — skipping.")
                continue
            sid = sids[0]
            if len(sids) > 1:
                print(f"  {p.name}: multiple sensors {sids}; using first ({sid!r}).")
            sensors.append(read_sensor_from_file(f, sid))
            print(f"  {p.name}: loaded sensor {sid!r}  rows={len(sensors[-1]['time_us'])}")
    return sensors


# ---------------------------------------------------------------------------
# CSV header builder
# ---------------------------------------------------------------------------

def _sensor_block_header(sensor_num: int = 1) -> list:
    """Return column names for one sensor block.
    Block 1 includes the leading Metadata column; subsequent blocks do not.
    """
    block = [
        "Sync Count",
        "Time",
        "Button Status",
        "Acceleration X (m/s^2)",
        "Acceleration Y (m/s^2)",
        "Acceleration Z (m/s^2)",
        "Angular Velocity X (rad/s)",
        "Angular Velocity Y (rad/s)",
        "Angular Velocity Z (rad/s)",
        "Magnetic Field X (uT)",
        "Magnetic Field Y (uT)",
        "Magnetic Field Z (uT)",
        "Orientation Quaternion Scalar",
        "Orientation Quaternion X",
        "Orientation Quaternion Y",
        "Orientation Quaternion Z",
        "Temperature (deg C) Z",
        "Pressure (Pa)",
    ]
    if sensor_num == 1:
        return ["Metadata"] + block
    return block


# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------

def write_csv(sensor_data: list, output_path: Path, source_name: str = ""):
    """Write sensor_data (list of sensor dicts) to a ml_pipeline-compatible CSV.

    Parameters
    ----------
    sensor_data  : list of dicts returned by read_sensor_from_file().
                   All sensors are time-aligned at index level (truncated to shortest).
    output_path  : destination CSV path (created / overwritten).
    source_name  : label for the metadata rows (filename etc.).
    """
    lengths = [len(s["time_us"]) for s in sensor_data]
    n_rows  = min(lengths)
    if len(set(lengths)) > 1:
        print(f"  WARNING: sensor row counts differ {lengths}; truncating to {n_rows}.")

    primary = sensor_data[0]

    # Build metadata header rows (skipped by ml_pipeline.py — they contain "=")
    sensor_ids_str    = ":".join(s["sid"]   for s in sensor_data)
    sensor_labels_str = ":".join(s["label"] for s in sensor_data)
    offset_h = primary["local_offset_us"] / 3_600_000_000.0
    metadata_rows = [
        f"File Format Version=H5toCsv",
        f"Monitor Case IDs= :{sensor_ids_str}",
        f"Monitor Labels= :{sensor_labels_str}",
        f"H5 Source={source_name}",
        f"Timezone={primary['tz_name']}  LocalTimeOffset_hours={offset_h:.4f}",
    ]

    # CSV column header
    header = _sensor_block_header(sensor_num=1)
    for i in range(1, len(sensor_data)):
        header += _sensor_block_header(sensor_num=i + 1)

    print(f"Writing {n_rows} rows to {output_path} ...")
    with open(output_path, "w", newline="", encoding="utf-8") as fout:
        writer = csv.writer(fout)
        writer.writerow(header)

        for meta_line in metadata_rows:
            writer.writerow([meta_line] + [""] * (len(header) - 1))

        for i in range(n_rows):
            row = []
            for block_idx, s in enumerate(sensor_data):
                t_us   = int(s["time_us"][i])
                sc     = int(s["sync"][i])
                btn    = int(s["button"][i])
                ax, ay, az = s["accel"][i]
                gx, gy, gz = s["gyro"][i]
                mx = s["mag"][i, 0] if s["mag"].shape[1] >= 3 else 0.0
                my = s["mag"][i, 1] if s["mag"].shape[1] >= 3 else 0.0
                mz = s["mag"][i, 2] if s["mag"].shape[1] >= 3 else 0.0
                qs = s["quat"][i, 0] if s["quat"].shape[1] >= 4 else 1.0
                qx = s["quat"][i, 1] if s["quat"].shape[1] >= 4 else 0.0
                qy = s["quat"][i, 2] if s["quat"].shape[1] >= 4 else 0.0
                qz = s["quat"][i, 3] if s["quat"].shape[1] >= 4 else 0.0
                tc = float(s["temp"][i]) if i < len(s["temp"]) else 0.0
                pr = float(s["pres"][i]) if i < len(s["pres"]) else 0.0

                if block_idx == 0:
                    row += [
                        "",               # Metadata (empty for data rows)
                        sc,               # Sync Count
                        t_us,             # Time (Unix µs)
                        btn,              # Button Status
                        f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}",
                        f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                        f"{mx:.6f}", f"{my:.6f}", f"{mz:.6f}",
                        f"{qs:.6f}", f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}",
                        f"{tc:.6f}", f"{pr:.6f}",
                    ]
                else:
                    row += [
                        sc,
                        t_us,
                        btn,
                        f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}",
                        f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                        f"{mx:.6f}", f"{my:.6f}", f"{mz:.6f}",
                        f"{qs:.6f}", f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}",
                        f"{tc:.6f}", f"{pr:.6f}",
                    ]
            writer.writerow(row)

    print(f"Done.  {output_path}")


# ---------------------------------------------------------------------------
# Post-conversion summary
# ---------------------------------------------------------------------------

def print_summary(sensor_data: list, output_path: Path, h5_label: str):
    """Print time range and a run_times.json template."""
    primary = sensor_data[0]
    n_rows  = min(len(s["time_us"]) for s in sensor_data)
    t0_us   = int(primary["time_us"][0])
    t1_us   = int(primary["time_us"][n_rows - 1])

    t0_utc = datetime.fromtimestamp(t0_us / 1e6, tz=timezone.utc)
    t1_utc = datetime.fromtimestamp(t1_us / 1e6, tz=timezone.utc)
    duration = (t1_us - t0_us) / 1e6

    # Try to derive the real UTC offset from the IANA timezone name stored in the file.
    # The LocalTimeOffset attribute in these APDM files is a firmware counter artifact
    # and does NOT reliably encode the true UTC offset — do not use it for run_times.json.
    tz_name = primary.get("tz_name", "")
    offset_h = 0.0
    t0_local = t0_utc
    t1_local = t1_utc
    tz_note = "UTC (could not resolve timezone)"
    try:
        import zoneinfo
        zi = zoneinfo.ZoneInfo(tz_name)
        t0_local = t0_utc.astimezone(zi)
        t1_local = t1_utc.astimezone(zi)
        offset_h = t0_local.utcoffset().total_seconds() / 3600.0
        tz_note = f"{tz_name}  (UTC{offset_h:+.1f}h)"
    except Exception:
        try:
            # Fallback: try pytz
            import pytz
            zi = pytz.timezone(tz_name)
            t0_local = t0_utc.astimezone(zi)
            t1_local = t1_utc.astimezone(zi)
            offset_h = t0_local.utcoffset().total_seconds() / 3600.0
            tz_note = f"{tz_name}  (UTC{offset_h:+.1f}h)"
        except Exception:
            pass

    # run_times.json offset_hours: pipeline adds offset_hours to local time to get UTC.
    # If local = UTC - 5h, then offset_hours = 5 (positive).
    offset_hours_json = int(round(-offset_h)) if offset_h != 0.0 else 0

    print()
    print("=== Conversion summary ===")
    print(f"  Source    : {h5_label}")
    print(f"  Output CSV: {output_path}")
    print(f"  Sensors   : {[s['sid'] for s in sensor_data]}")
    print(f"  Rows      : {n_rows}  ({duration:.1f} s / {duration/60:.1f} min)")
    print(f"  UTC  start: {t0_utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"  UTC  end  : {t1_utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"  Local start: {t0_local.strftime('%Y-%m-%d %H:%M:%S')}  ({tz_note})")
    print(f"  Local end  : {t1_local.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    print("  To use with ml_pipeline / run_report, create a run_times.json:")
    print(f"""
  {{
    "date": "{t0_utc.strftime('%Y-%m-%d')}",
    "offset_hours": {offset_hours_json},
    "runs": [
      {{ "name": "run1", "start": "HH:MM:SS", "end": "HH:MM:SS" }},
      {{ "name": "run2", "start": "HH:MM:SS", "end": "HH:MM:SS" }}
    ]
  }}

  Fill in HH:MM:SS using LOCAL times ({tz_note}).
  Recording window (local): {t0_local.strftime('%H:%M:%S')} — {t1_local.strftime('%H:%M:%S')}
  Recording window (UTC) :  {t0_utc.strftime('%H:%M:%S')} — {t1_utc.strftime('%H:%M:%S')}
""")
    print("  Then run:")
    print(f"    python run_report.py --data {output_path.name} --run-times run_times.json --report short")
    print()


# ---------------------------------------------------------------------------
# Main conversion entry points
# ---------------------------------------------------------------------------

def convert_single(h5_path: Path, output_path: Path, all_sensors: bool = False) -> list:
    """Convert one h5 file (single- or multi-sensor) to CSV. Returns sensor_data list."""
    with h5py.File(h5_path, "r") as f:
        sids = _sensor_ids(f)
        if not sids:
            raise ValueError(
                f"No sensor groups found in {h5_path}.\n"
                f"Top-level keys: {list(f.keys())}"
            )
        sids_to_use = sids if all_sensors else sids[:1]
        print(f"Loading {len(sids_to_use)} sensor(s) from {h5_path.name}: {sids_to_use}")
        sensor_data = [read_sensor_from_file(f, sid) for sid in sids_to_use]

    write_csv(sensor_data, output_path, source_name=h5_path.name)
    return sensor_data


def convert_merge(h5_paths: list, output_path: Path) -> list:
    """Load one sensor per file and merge all into a single CSV."""
    print(f"Merging {len(h5_paths)} file(s) ...")
    sensor_data = merge_single_sensor_files(h5_paths)
    if not sensor_data:
        raise ValueError("No sensors could be loaded from the provided files.")
    write_csv(sensor_data, output_path, source_name=" + ".join(p.name for p in h5_paths))
    return sensor_data


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert APDM Opal IMU .h5 file(s) to the CSV format expected by ml_pipeline.py."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python H5toCsv.py recording.h5                            # first sensor only
  python H5toCsv.py recording.h5 --output session.csv
  python H5toCsv.py recording.h5 --all-sensors              # all sensors as extra blocks
  python H5toCsv.py recording.h5 --list-sensors             # inspect without converting
  python H5toCsv.py f1.h5 f2.h5 f3.h5 --merge --output merged.csv

After conversion:
  python run_report.py --data session.csv --run-times runs.json --report short
  python run_report.py --data session.csv --filter kalman --detector threshold --report short
        """,
    )
    parser.add_argument(
        "h5_files", type=Path, nargs="+",
        help="One or more APDM Opal .h5 files.",
    )
    parser.add_argument(
        "--output", "-o", type=Path, default=None,
        help="Output CSV path.  Defaults to <first_h5_stem>.csv in the current directory.",
    )
    parser.add_argument(
        "--all-sensors", action="store_true",
        help="Include all sensors in the file as extra column blocks (matches original multi-sensor CSV).",
    )
    parser.add_argument(
        "--merge", action="store_true",
        help="When multiple h5 files are given, load one sensor per file and merge into one CSV.",
    )
    parser.add_argument(
        "--list-sensors", action="store_true",
        help="Print sensor IDs and info for each file, then exit without converting.",
    )
    args = parser.parse_args()

    # Validate input files
    for p in args.h5_files:
        if not p.exists():
            print(f"ERROR: File not found: {p}")
            sys.exit(1)

    if args.list_sensors:
        for p in args.h5_files:
            list_sensors(p)
        return

    output_path = args.output or args.h5_files[0].with_suffix(".csv")

    if args.merge or len(args.h5_files) > 1:
        if not args.merge and len(args.h5_files) > 1:
            print("Multiple files provided — enabling --merge automatically.")
        sensor_data = convert_merge(args.h5_files, output_path)
        h5_label = " + ".join(p.name for p in args.h5_files)
    else:
        sensor_data = convert_single(args.h5_files[0], output_path, all_sensors=args.all_sensors)
        h5_label = args.h5_files[0].name

    print_summary(sensor_data, output_path, h5_label)


if __name__ == "__main__":
    main()
