# analyze.py
# - compares encoder angle to IMU euler angles for sensor fusion accuracy analysis
# - supports live serial capture and offline CSV analysis
#
# Usage:
#   Offline:  python analyze.py data.csv
#   Live:     python analyze.py --live --port /dev/ttyACM0 --baud 115200
#   Live+Save: python analyze.py --live --port /dev/ttyACM0 --save capture.csv

import argparse
import sys
import signal
from io import StringIO

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import signal as sp_signal


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare encoder angle to IMU euler angles."
    )
    parser.add_argument(
        "file", nargs="?", default=None, help="Path to CSV file for offline analysis."
    )
    parser.add_argument(
        "--live", action="store_true", help="Enable live serial capture mode."
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)."
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)."
    )
    parser.add_argument(
        "--save",
        default=None,
        help="In live mode, save captured data to this CSV path.",
    )
    parser.add_argument(
        "--axis",
        choices=["pitch", "roll", "yaw"],
        default=None,
        help="IMU axis to use for error analysis. Auto-detected if omitted.",
    )
    args = parser.parse_args()

    if not args.live and args.file is None:
        parser.error("Provide a CSV file path, or use --live for serial capture.")

    return args


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------

COLUMNS = ["tag", "ts", "enc", "pitch", "roll", "yaw"]


def parse_csv(path):
    """Read a CSV file, filtering for DATA lines only."""
    df = pd.read_csv(path, names=COLUMNS, on_bad_lines="skip")
    df = df[df["tag"] == "DATA"].copy()
    for col in ["ts", "enc", "pitch", "roll", "yaw"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df.dropna(inplace=True)
    df.reset_index(drop=True, inplace=True)
    # convert timestamp to seconds relative to first sample
    df["t"] = (df["ts"] - df["ts"].iloc[0]) / 1000.0
    return df


def parse_line(line):
    """Parse a single DATA line. Returns dict or None."""
    line = line.strip()
    if not line.startswith("DATA,"):
        return None
    parts = line.split(",")
    if len(parts) != 6:
        return None
    try:
        return {
            "tag": parts[0],
            "ts": float(parts[1]),
            "enc": float(parts[2]),
            "pitch": float(parts[3]),
            "roll": float(parts[4]),
            "yaw": float(parts[5]),
        }
    except ValueError:
        return None


def select_best_axis(df, forced_axis=None):
    """Pick the euler axis most correlated with the encoder signal.

    Returns (axis_name, series_of_that_axis).
    """
    if forced_axis is not None:
        return forced_axis, df[forced_axis]

    best_axis = None
    best_corr = -1.0
    for axis in ["pitch", "roll", "yaw"]:
        corr = abs(df["enc"].corr(df[axis]))
        if corr > best_corr:
            best_corr = corr
            best_axis = axis
    return best_axis, df[best_axis]


def normalize_angle(a):
    """Normalize angle array to [0, 360)."""
    return np.mod(a, 360.0)


def wrapped_error(imu, enc):
    """Compute angular error in [-180, 180) handling wrap-around."""
    diff = normalize_angle(imu) - normalize_angle(enc)
    # wrap to [-180, 180)
    diff = (diff + 180.0) % 360.0 - 180.0
    return diff


def estimate_phase_lag(enc, imu, sample_period_s):
    """Estimate phase lag in ms via cross-correlation."""
    # remove means
    a = enc - np.mean(enc)
    b = imu - np.mean(imu)
    corr = sp_signal.correlate(b, a, mode="full")
    lags = sp_signal.correlation_lags(len(b), len(a), mode="full")
    peak_idx = np.argmax(np.abs(corr))
    lag_samples = lags[peak_idx]
    lag_ms = lag_samples * sample_period_s * 1000.0
    return lag_ms


# ---------------------------------------------------------------------------
# Offline analysis
# ---------------------------------------------------------------------------


def run_offline(df, forced_axis):
    """Generate the multi-panel figure and print stats."""
    # Convert encoder from [0, 360) to [-180, 180) to match IMU convention
    df["enc"] = np.where(df["enc"] > 180.0, df["enc"] - 360.0, df["enc"])

    axis_name, imu_axis = select_best_axis(df, forced_axis)

    # If the IMU axis is negatively correlated with the encoder the two have
    # opposite sign conventions for the same physical rotation.  Flip the IMU
    # axis so that both increase in the same direction.
    if imu_axis.corr(df["enc"]) < 0:
        imu_axis = -imu_axis

    # Align IMU axis to encoder reference frame by removing the initial offset.
    # The magnetometer yaw is relative to magnetic north while the encoder is
    # manually zeroed, so they differ by a constant heading offset.
    offset = imu_axis.iloc[0] - df["enc"].iloc[0]
    imu_axis = imu_axis - offset

    error = wrapped_error(imu_axis.values, df["enc"].values)

    # stats
    mean_err = np.mean(error)
    std_err = np.std(error)
    rms_err = np.sqrt(np.mean(error**2))
    max_err = np.max(np.abs(error))
    corr_r = df["enc"].corr(df[axis_name])

    # phase lag
    if len(df) > 10:
        sample_period = np.median(np.diff(df["t"].values))
        lag_ms = estimate_phase_lag(df["enc"].values, imu_axis.values, sample_period)
    else:
        lag_ms = float("nan")

    # print report
    print()
    print("=== Accuracy Report ===")
    print(f"  Best matching axis:  {axis_name}")
    print(f"  Correlation (R):     {corr_r:.4f}")
    print(f"  Mean error:          {mean_err:.2f} deg")
    print(f"  Std deviation:       {std_err:.2f} deg")
    print(f"  RMS error:           {rms_err:.2f} deg")
    print(f"  Max absolute error:  {max_err:.2f} deg")
    print(f"  Estimated phase lag: {lag_ms:.1f} ms")
    print()

    # --- figure ---
    fig, axes = plt.subplots(
        3, 1, figsize=(12, 9), sharex=False, gridspec_kw={"height_ratios": [3, 2, 2]}
    )
    fig.suptitle("Encoder vs IMU Angle Comparison", fontsize=14)

    t = df["t"].values

    # panel 1: angle vs time (all axes, selected axis shown aligned)
    ax1 = axes[0]
    ax1.plot(t, df["enc"].values, "-", color="tab:blue", linewidth=1.5, label="Encoder")
    for axis, color in [
        ("pitch", "tab:red"),
        ("roll", "tab:green"),
        ("yaw", "tab:orange"),
    ]:
        if axis == axis_name:
            ax1.plot(
                t,
                imu_axis.values,
                "--",
                color=color,
                linewidth=1.0,
                label=f"{axis.title()} (aligned)",
            )
        else:
            ax1.plot(
                t, df[axis].values, "--", color=color, linewidth=1.0, label=axis.title()
            )
    ax1.set_ylabel("Angle (deg)")
    ax1.set_xlabel("Time (s)")
    ax1.set_title("Angle vs Time")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # panel 2: error vs time
    ax2 = axes[1]
    ax2.plot(t, error, "-", color="tab:purple", linewidth=1.0)
    ax2.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax2.axhline(
        mean_err + std_err, color="tab:purple", linestyle=":", linewidth=0.7, alpha=0.5
    )
    ax2.axhline(
        mean_err - std_err, color="tab:purple", linestyle=":", linewidth=0.7, alpha=0.5
    )
    ax2.fill_between(
        t, mean_err - std_err, mean_err + std_err, color="tab:purple", alpha=0.1
    )
    ax2.set_ylabel("Error (deg)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title(f"Error vs Time  ({axis_name} - encoder)")
    ax2.grid(True, alpha=0.3)

    # panel 3: error histogram
    ax3 = axes[2]
    ax3.hist(
        error, bins=50, color="tab:purple", alpha=0.7, edgecolor="black", linewidth=0.5
    )
    ax3.axvline(
        mean_err,
        color="red",
        linestyle="-",
        linewidth=1.2,
        label=f"Mean={mean_err:.2f}",
    )
    ax3.axvline(mean_err + std_err, color="red", linestyle=":", linewidth=0.8)
    ax3.axvline(mean_err - std_err, color="red", linestyle=":", linewidth=0.8)
    stats_text = (
        f"Mean: {mean_err:.2f}\n"
        f"Std:  {std_err:.2f}\n"
        f"RMS:  {rms_err:.2f}\n"
        f"Max:  {max_err:.2f}"
    )
    ax3.text(
        0.98,
        0.95,
        stats_text,
        transform=ax3.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="right",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )
    ax3.set_xlabel("Error (deg)")
    ax3.set_ylabel("Count")
    ax3.set_title("Error Distribution")
    ax3.legend(loc="upper left")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Live serial capture
# ---------------------------------------------------------------------------


def run_live(port, baud, save_path, forced_axis):
    """Stream from serial, plot in real-time, then do full analysis on stop."""
    import serial as pyserial

    ser = pyserial.Serial(port, baud, timeout=0.1)
    print(f"Connected to {port} at {baud} baud. Press Ctrl+C to stop capture.")

    # storage
    rows = []

    # setup live figure (angle vs time only)
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_title("Live: Angle vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.grid(True, alpha=0.3)

    (line_enc,) = ax.plot([], [], "-", color="tab:blue", linewidth=1.5, label="Encoder")
    (line_pitch,) = ax.plot([], [], "--", color="tab:red", linewidth=1.0, label="Pitch")
    (line_roll,) = ax.plot([], [], "--", color="tab:green", linewidth=1.0, label="Roll")
    (line_yaw,) = ax.plot([], [], "--", color="tab:orange", linewidth=1.0, label="Yaw")
    ax.legend(loc="upper right")

    t0 = None
    sign_flip = None  # determined once after enough samples accumulate

    def update_plot():
        """Redraw the live plot with current data."""
        nonlocal sign_flip

        if len(rows) < 2:
            return
        t_arr = np.array([r["t"] for r in rows])
        ax.set_xlim(t_arr[0], max(t_arr[-1], 1.0))

        enc_arr = np.array([r["enc"] for r in rows])
        # Convert encoder from [0, 360) to [-180, 180) to match IMU convention
        enc_arr = np.where(enc_arr > 180.0, enc_arr - 360.0, enc_arr)
        pitch_arr = np.array([r["pitch"] for r in rows])
        roll_arr = np.array([r["roll"] for r in rows])
        yaw_arr = np.array([r["yaw"] for r in rows])

        # Determine sign flip once after 20 samples to detect opposite
        # rotation conventions between encoder and yaw.
        if sign_flip is None and len(rows) >= 20:
            corr = np.corrcoef(enc_arr, yaw_arr)[0, 1]
            sign_flip = -1.0 if corr < 0 else 1.0

        # Apply sign flip and offset correction so yaw overlays encoder
        if sign_flip is not None:
            yaw_arr = sign_flip * yaw_arr
            yaw_arr = yaw_arr - (yaw_arr[0] - enc_arr[0])

        line_enc.set_data(t_arr, enc_arr)
        line_pitch.set_data(t_arr, pitch_arr)
        line_roll.set_data(t_arr, roll_arr)
        line_yaw.set_data(t_arr, yaw_arr)

        # auto-scale y axis
        all_vals = np.concatenate([enc_arr, pitch_arr, roll_arr, yaw_arr])
        ymin, ymax = np.min(all_vals), np.max(all_vals)
        margin = max((ymax - ymin) * 0.1, 5.0)
        ax.set_ylim(ymin - margin, ymax + margin)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    try:
        last_draw = 0
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                parsed = parse_line(line)
                if parsed is None:
                    continue

                # compute relative time
                if t0 is None:
                    t0 = parsed["ts"]
                parsed["t"] = (parsed["ts"] - t0) / 1000.0
                rows.append(parsed)

                # throttle redraws to ~5 Hz
                now = parsed["t"]
                if now - last_draw >= 0.2:
                    update_plot()
                    last_draw = now

            except (pyserial.SerialException, UnicodeDecodeError):
                continue

    except KeyboardInterrupt:
        print(f"\nCapture stopped. {len(rows)} samples collected.")

    finally:
        ser.close()
        plt.close(fig)

    if len(rows) == 0:
        print("No data captured.")
        return

    # build dataframe
    df = pd.DataFrame(rows)

    # save if requested
    if save_path is not None:
        save_df = df[["tag", "ts", "enc", "pitch", "roll", "yaw"]].copy()
        save_df.to_csv(save_path, index=False, header=False)
        print(f"Data saved to {save_path}")

    # run full offline analysis on captured data
    print("\nRunning analysis on captured data...")
    run_offline(df, forced_axis)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    args = parse_args()

    if args.live:
        run_live(args.port, args.baud, args.save, args.axis)
    else:
        df = parse_csv(args.file)
        if len(df) == 0:
            print("No DATA lines found in file.")
            sys.exit(1)
        print(f"Loaded {len(df)} samples from {args.file}")
        run_offline(df, args.axis)


if __name__ == "__main__":
    main()
