# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "pandas",
#   "matplotlib",
# ]
# ///

import pandas as pd
import matplotlib.pyplot as plt

cold = pd.read_csv("battery_testing_cold.csv")
inside = pd.read_csv("battery_testing_inside.csv")

min_level = min(cold["level"].iloc[0], inside["level"].iloc[0])

cold = cold[cold["level"] <= min_level]
inside = inside[inside["level"] <= min_level]

# Convert microseconds to seconds, starting from 0
cold["timestamp_s"] = (cold["timestamp_us"] - cold["timestamp_us"].iloc[0]) / 1e6
cold["timestamp_s"] = cold["timestamp_s"].round()
inside["timestamp_s"] = (inside["timestamp_us"] - inside["timestamp_us"].iloc[0]) / 1e6
inside["timestamp_s"] = inside["timestamp_s"].round()

renamed_cold = cold.rename(columns={"level": "cold_level"})
renamed_inside = inside.rename(columns={"level": "idle_level"})

print(renamed_cold)

diff = pd.merge(renamed_cold, renamed_inside, on="timestamp_s")
print(diff.shape)
diff["diff"] = diff["idle_level"] - diff["cold_level"]

plt.figure(figsize=(12, 6))
plt.plot(cold["timestamp_s"] / 3600, cold["level"], label="Cold Room (t = -15C)", color="steelblue")
plt.plot(inside["timestamp_s"] / 3600, inside["level"], label="Ambient Environment (t = 25C)", color="tomato")
plt.plot(diff["timestamp_s"] / 3600, diff["diff"], linestyle="dashed", label="Difference", color="gray")

plt.xlabel("Time (hours)")
plt.ylabel("Battery Level (%)")
plt.title("Battery Level vs Time")
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("battery_comparison.png", dpi=150)
plt.show()
print("Saved to battery_comparison.png")
