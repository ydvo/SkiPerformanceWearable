# /// script
# dependencies = [
#   "matplotlib",
# ]
# ///
import csv
import re
import matplotlib.pyplot as plt

# -------- CONFIG --------
INPUT_TXT = "imu_dump.log"
OUTPUT_CSV = "imu_timeseries.csv"
SAMPLES_PER_FRAME = 14
# ------------------------


def is_data_line(line: str) -> bool:
    return bool(re.match(r"^\d+", line))


def parse_frame(line: str):
    parts = line.split(",")

    expected_len = 2 + 4 * SAMPLES_PER_FRAME
    if len(parts) != expected_len:
        raise ValueError(
            f"Expected {expected_len} fields, got {len(parts)}"
        )

    start_us = int(parts[0])
    end_us = int(parts[-1])

    floats = list(map(float, parts[1:-1]))

    samples = []
    for i in range(0, len(floats), 4):
        samples.append(tuple(floats[i:i + 4]))

    return start_us, samples, end_us


def main():
    collecting = False
    rows_written = 0

    timestamps = []
    ws, xs, ys, zs = [], [], [], []

    with open(INPUT_TXT, "r") as fin, open(OUTPUT_CSV, "w", newline="") as fout:
        writer = csv.writer(fout)
        writer.writerow(["timestamp_us", "w", "x", "y", "z"])

        for lineno, raw in enumerate(fin, start=1):
            line = raw.strip()

            if not line:
                continue

            # ---- FSM ----
            if line == "START":
                collecting = True
                continue

            if line in ("END", "ERROR"):
                break

            if not collecting:
                continue
            # -------------

            if not is_data_line(line):
                continue

            try:
                start_us, samples, end_us = parse_frame(line)
            except Exception as e:
                print(f"[WARN] Line {lineno}: {e}")
                continue

            n = len(samples)
            if n < 2:
                continue

            dt = (end_us - start_us) / (n - 1)

            for i, (w, x, y, z) in enumerate(samples):
                t = start_us + i * dt

                writer.writerow([int(t), w, x, y, z])

                timestamps.append(t * 1e-6)  # seconds for plotting
                ws.append(w)
                xs.append(x)
                ys.append(y)
                zs.append(z)

                rows_written += 1

    print(f"[DONE] Wrote {rows_written} samples → {OUTPUT_CSV}")

    # -------- PLOTTING --------
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, ws, label="w")
    plt.plot(timestamps, xs, label="x")
    plt.plot(timestamps, ys, label="y")
    plt.plot(timestamps, zs, label="z")

    plt.xlabel("Time (s)")
    plt.ylabel("Quaternion value")
    plt.title("IMU Quaternion vs Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
