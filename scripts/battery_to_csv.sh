#!/usr/bin/env bash

INPUT="${1:-battery_testing_inside.txt}"
OUTPUT="${2:-battery_testing_inside.csv}"

{
    echo "timestamp_us,level"
    sed -nE 's/.*timestamp_us: ([0-9]+), level: ([0-9.]+).*/\1,\2/p' "$INPUT"
} > "$OUTPUT"

echo "Written to $OUTPUT"
