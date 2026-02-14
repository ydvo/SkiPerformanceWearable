import csv
import re

# ---- CONFIG ----
INPUT_TXT = "imu_dump.log"
OUTPUT_CSV = "imu_dump.csv"
SAMPLES_PER_FRAME = 14
# ----------------


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

    with open(INPUT_TXT, "r") as fin, open(OUTPUT_CSV, "w", newline="") as fout:
        writer = csv.writer(fout)

        # CSV header
        header = ["frame_start_us"]
        for i in range(SAMPLES_PER_FRAME):
            header += [f"w{i}", f"x{i}", f"y{i}", f"z{i}"]
        header.append("frame_end_us")
        writer.writerow(header)

        for lineno, raw in enumerate(fin, start=1):
            line = raw.strip()

            if not line:
                continue

            # ---- STATE TRANSITIONS ----
            if line == "START":
                collecting = True
                print(f"[INFO] START detected at line {lineno}")
                continue

            if line == "END" or line == "ERROR":
                print(f"[INFO] {line} detected at line {lineno}")
                break

            if not collecting:
                continue
            # ---------------------------

            if not is_data_line(line):
                print(f"[WARN] Ignored non-data line {lineno}: {line}")
                continue

            try:
                start_us, samples, end_us = parse_frame(line)
            except Exception as e:
                print(f"[WARN] Parse error on line {lineno}: {e}")
                continue

            row = [start_us]
            for (w, x, y, z) in samples:
                row.extend([w, x, y, z])
            row.append(end_us)

            writer.writerow(row)
            rows_written += 1

    print(f"[DONE] Collected {rows_written} frames → {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
