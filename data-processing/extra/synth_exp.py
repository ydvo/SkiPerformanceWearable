import numpy as np
from synth import synth_imu_9dof

"""
ICM20948 Parameters

Gyroscope: 
    Noise BW: 10Hz
    Noise density: 0.015/sqrt(Hz)
    Noise RMS: 0.015 * sqrt(10) = 0.047 dps = 8.2e-4 rad/s
    Initial ZRO Tolerance: +- 5dps and temp: +- 0.05dps/C
    Optionally: Random walk for bias drift 

Accelerometer: 
    Initial tolerance: +- 50mg and temp: +-0.8 mg/s
    Noise density: 230 ug/sqrt(Hz)
    Noise RMS: 230ug * sqrt(10) = 0.727 mg

Magnetometer: 
    Range: +- 4900 uT
    Hard iron and soft iron must be calibrated for environment. 
"""

cfg = {
    "Fs": 200,
    "T": 30,
    "a_nav_fun": lambda t: np.column_stack([
        0.0*t,
        0.0*t,
        0.0*t
    ]),
    "gyro": {
        "noise_std": np.deg2rad(0.047), 
        "bias0": np.deg2rad([0.8, -0.6, 0.5]),
        "bias_rw": np.deg2rad(0.01), 
        "scale_mis": np.eye(3) + np.diag([0.002, -0.003, 0.001]),
    },
    "acc":  {
        "noise_std": 0.727 * 9.81 * 1e-3,
        "bias": np.array([10*9.81*1e-3, -20*9.81*1e-3, 30*9.81*1e-3]), 
        "scale_mis": np.eye(3) + np.diag([0.003, 0.002, -0.002]),
    },
    "mag":  {
        "noise_std": 0.6, 
        "hard_iron": np.array([+12.0, -6.0, +8.0]), 
        "soft_iron": np.array([[1.03, 0.02, 0.00],
                           [0.00, 0.98, 0.02],
                           [0.00, 0.00, 1.01]]),
    },
}


sim = synth_imu_9dof(cfg)
t = sim["time"]

import matplotlib.pyplot as plt
# --- True vs noisy Euler angles ---
plt.figure(figsize=(10,6))
plt.plot(t, np.rad2deg(sim["euler_true"]), lw=2)
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.title("True Euler angles (Yaw, Pitch, Roll)")
plt.legend(["Yaw", "Pitch", "Roll"])
plt.grid(True)
plt.tight_layout()

# --- Measured (noisy) signals ---
fig, axs = plt.subplots(3, 1, figsize=(10,8), sharex=True)
axs[0].plot(t, np.rad2deg(sim["gyro_true"]), lw=1.5, label="True")
axs[0].plot(t, np.rad2deg(sim["gyro"]), lw=0.8, alpha=0.7, label="Noisy")
axs[0].set_ylabel("Gyro [deg/s]")
axs[0].legend()
axs[0].grid(True)

axs[1].plot(t, sim["acc_true"], lw=1.5, label="True")
axs[1].plot(t, sim["accel"], lw=0.8, alpha=0.7, label="Noisy")
axs[1].set_ylabel("Accel [m/s²]")
axs[1].legend()
axs[1].grid(True)

axs[2].plot(t, sim["mag_true"], lw=1.5, label="True")
axs[2].plot(t, sim["mag"], lw=0.8, alpha=0.7, label="Noisy")
axs[2].set_ylabel("Magnetic field")
axs[2].set_xlabel("Time [s]")
axs[2].legend()
axs[2].grid(True)

plt.suptitle("True vs. Measured (Noisy) IMU Signals")
plt.tight_layout()
plt.show()



