# /// script
# dependencies = [
#   "pyserial",
# ]
# ///

import serial
import sys

PORT = "/dev/tty.usbmodem2101"   # change if needed (COMx on Windows)
BAUD = 921600
OUT_FILE = "imu_dump.bin"
TIMEOUT = 2.0

def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

    print("Waiting for START...")
    while True:
        line = ser.readline()
        if not line:
            continue

        line = line.strip()
        if line == b"START":
            break

    print("START received")

    total = 0

    with open(OUT_FILE, "wb") as f:
        while True:
            line = ser.readline()
            if not line:
                continue

            line = line.strip()

            if line == b"END":
                print("END received")
                break

            if not line.startswith(b"CHUNK"):
                print("Skipping unexpected line:", line)
                continue

            try:
                size = int(line.split()[1])
            except (IndexError, ValueError):
                print("Invalid CHUNK header:", line)
                break

            remaining = size
            while remaining > 0:
                data = ser.read(remaining)
                if not data:
                    raise RuntimeError("Timeout while reading chunk")
                f.write(data)
                remaining -= len(data)
                total += len(data)

            print(f"Received chunk: {size} bytes (total {total})")

    ser.close()
    print(f"Dump complete: {total} bytes written to {OUT_FILE}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user")


