import imufusion
import matplotlib.pyplot as plt
import numpy as np
from synth_imu_9dof import synth_imu_9dof

cfg = {
    "Fs": 200,
    "T": 120,
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
        "hard_iron": np.array([0.0, 0.0, 0.0]), #np.array([+12.0, -6.0, +8.0]), 
        "soft_iron": np.array([[1.00, 0.00, 0.00],
                            [0.00, 1.00, 0.00],
                            [0.00, 0.00, 1.00]]),
    },
}

sim = synth_imu_9dof(cfg)
t = sim["time"]

# --- Plot 2: Measured (noisy) signals ---
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

ahrs = imufusion.Ahrs()
euler = np.empty((len(t), 3))

gyro = np.rad2deg(sim["gyro"])
acc = sim["accel"] / 9.81
mag = sim["mag"]

for index in range(len(t)):
    # ahrs.update(gyro[index], acc[index], mag[index], 1 / sim["Fs"])
    ahrs.update_no_magnetometer(gyro[index], acc[index], 1 / sim["Fs"])
    euler[index] = ahrs.quaternion.to_euler()

# Plot Euler angles
fig, axs = plt.subplots(3, 1, figsize=(10,8), sharex=True)
euler_true = np.rad2deg(sim["euler_true"])

axs[0].plot(t, euler_true[:, 2], "tab:orange", label="Roll true")
axs[0].plot(t, euler[:, 0], "tab:red", label="Roll")
axs[0].set_title("Roll")
axs[0].set_xlabel("Seconds")
axs[0].set_ylabel("Degrees")
axs[0].grid()
axs[0].legend()

axs[1].plot(t, euler_true[:, 1], "tab:olive", label="Pitch true")
axs[1].plot(t, euler[:, 1], "tab:green", label="Pitch")
axs[1].set_title("Pitch")
axs[1].set_xlabel("Seconds")
axs[1].set_ylabel("Degrees")
axs[1].grid()
axs[1].legend()

axs[2].plot(t, euler_true[:, 0], "tab:cyan", label="Yaw true")
axs[2].plot(t, euler[:, 2], "tab:blue", label="Yaw")
axs[2].set_title("Yaw")
axs[2].set_xlabel("Seconds")
axs[2].set_ylabel("Degrees")
axs[2].grid()
axs[2].legend()

plt.suptitle("True vs. Measured (Noisy) Euler Angles")
plt.tight_layout()
plt.show()
