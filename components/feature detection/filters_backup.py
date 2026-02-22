"""
Noise-reduction filters for ski boot IMU data.

Two independent filters:
  - KalmanFilter1D: smooths a single signal (e.g. gyro_z) using a 2-state
    constant-velocity Kalman filter.  Pure numpy, no extra dependencies.
  - MadgwickFilter: quaternion-based orientation filter that fuses accel +
    gyro to produce a drift-corrected orientation.  The filtered quaternion
    can be used directly in place of the sensor's raw quaternion, and the
    yaw-rate derived from successive quaternion differences gives a cleaner
    turn signal than raw gyro_z.  Pure numpy, no extra dependencies.

Usage
-----
from filters import KalmanFilter1D, MadgwickFilter

# --- Kalman (gyro_z only) ---
kf = KalmanFilter1D(process_noise=KALMAN_Q, measurement_noise=KALMAN_R)
gyro_z_smooth = kf.filter(gyro_z_raw)      # shape (N,)

# --- Madgwick (full orientation) ---
mf = MadgwickFilter(beta=MADGWICK_BETA, sample_rate=128.0)
quat_smooth = mf.filter(gyro_rad_s, accel_m_s2)  # shape (N, 4) [w, x, y, z]
# Then replace raw quat array with quat_smooth before feature extraction.
# For a turn-rate signal use:
gyro_z_smooth = mf.yaw_rate(quat_smooth)          # shape (N,)
"""

import numpy as np


# ---------------------------------------------------------------------------
# Kalman 1-D (constant velocity model applied to a scalar signal)
# ---------------------------------------------------------------------------

class KalmanFilter1D:
    """Scalar Kalman filter: state = [value, rate]; measurement = value.

    Parameters
    ----------
    process_noise : float
        Diagonal process noise covariance scaling (Q).  Larger values let
        the filter track fast changes; smaller values smooth more aggressively.
        Typical starting point for gyro_z: 1e-3.
    measurement_noise : float
        Measurement noise variance (R).  Larger values trust the model more
        and smooth more.  Typical starting point for gyro_z: 1e-1.
    dt : float
        Sample interval in seconds (default 1/128 = ~7.8 ms at 128 Hz).
    """

    def __init__(
        self,
        process_noise: float = 1e-3,
        measurement_noise: float = 1e-1,
        dt: float = 1.0 / 128.0,
    ):
        self.dt = dt
        self.q = float(process_noise)
        self.r = float(measurement_noise)

        # State transition: [value, rate] -> [value + rate*dt, rate]
        self.F = np.array([[1.0, dt], [0.0, 1.0]])

        # Observation: we observe value only
        self.H = np.array([[1.0, 0.0]])

        # Process noise covariance (scaled identity)
        self.Q = self.q * np.eye(2)

        # Measurement noise
        self.R = np.array([[self.r]])

    def filter(self, signal: np.ndarray) -> np.ndarray:
        """Apply the Kalman filter to a 1-D signal array.

        Parameters
        ----------
        signal : array of shape (N,)

        Returns
        -------
        smoothed : array of shape (N,) — filtered signal values.
        """
        signal = np.asarray(signal, dtype=np.float64)
        n = len(signal)
        out = np.empty(n)

        # Initialise state and covariance
        x = np.array([signal[0], 0.0])  # [value, rate]
        P = np.eye(2) * 1.0

        F, H, Q, R = self.F, self.H, self.Q, self.R
        HT = H.T

        for k in range(n):
            # Predict
            x = F @ x
            P = F @ P @ F.T + Q

            # Update
            y = signal[k] - (H @ x)[0]             # innovation
            S = (H @ P @ HT + R)[0, 0]             # innovation covariance
            K = (P @ HT) / S                        # Kalman gain (2,1)
            x = x + K[:, 0] * y
            P = (np.eye(2) - K @ H) @ P

            out[k] = x[0]

        return out


# ---------------------------------------------------------------------------
# Madgwick orientation filter
# ---------------------------------------------------------------------------

class MadgwickFilter:
    """Quaternion-based sensor fusion filter (Madgwick 2010).

    Fuses 3-axis gyroscope and 3-axis accelerometer to produce a
    drift-corrected orientation quaternion.  Magnetometer is optional.

    Parameters
    ----------
    beta : float
        Filter gain.  Higher = faster correction of gyro drift but more
        sensitivity to accel noise.  Typical range: 0.01 – 0.1.
        Default 0.033 (Madgwick's recommendation for 50-100 Hz IMU).
    sample_rate : float
        Sensor sample rate in Hz (default 128.0).
    """

    def __init__(self, beta: float = 0.033, sample_rate: float = 128.0):
        self.beta = float(beta)
        self.dt = 1.0 / float(sample_rate)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def filter(
        self,
        gyro: np.ndarray,
        accel: np.ndarray,
        mag: np.ndarray | None = None,
        q0: np.ndarray | None = None,
    ) -> np.ndarray:
        """Run the Madgwick filter over an array of IMU samples.

        Parameters
        ----------
        gyro  : (N, 3) gyroscope readings in rad/s  [gx, gy, gz]
        accel : (N, 3) accelerometer readings in m/s²  [ax, ay, az]
        mag   : (N, 3) optional magnetometer readings (µT or arbitrary units).
                If None the accel-only (MARG-less) version is used.
        q0    : (4,) initial quaternion [w, x, y, z].  If None defaults to
                identity [1, 0, 0, 0].

        Returns
        -------
        quats : (N, 4) filtered quaternions  [w, x, y, z]
        """
        gyro = np.asarray(gyro, dtype=np.float64)
        accel = np.asarray(accel, dtype=np.float64)
        n = len(gyro)
        quats = np.empty((n, 4))

        q = np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else np.array(q0, dtype=np.float64)
        q = q / np.linalg.norm(q)

        if mag is not None:
            mag = np.asarray(mag, dtype=np.float64)
            for k in range(n):
                q = self._update_marg(q, gyro[k], accel[k], mag[k])
                quats[k] = q
        else:
            for k in range(n):
                q = self._update_imu(q, gyro[k], accel[k])
                quats[k] = q

        return quats

    def yaw_rate(self, quats: np.ndarray) -> np.ndarray:
        """Estimate yaw-rate (rad/s) from successive quaternion differences.

        This is the z-component of the angular velocity in the world frame
        derived from the filtered orientation sequence.  Suitable as a
        turn-rate signal in place of raw gyro_z.

        Parameters
        ----------
        quats : (N, 4) quaternion array [w, x, y, z]

        Returns
        -------
        yaw_rate : (N,) array in rad/s.  First sample equals second sample.
        """
        quats = np.asarray(quats, dtype=np.float64)
        n = len(quats)
        if n < 2:
            return np.zeros(n)

        # Yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
        yaw = np.arctan2(
            2.0 * (quats[:, 0] * quats[:, 3] + quats[:, 1] * quats[:, 2]),
            1.0 - 2.0 * (quats[:, 2] ** 2 + quats[:, 3] ** 2),
        )
        # Unwrap to avoid 2π jumps
        yaw = np.unwrap(yaw)

        # Differentiate
        dyaw = np.diff(yaw) / self.dt
        # Pad first sample
        dyaw = np.concatenate([[dyaw[0]], dyaw])
        return dyaw

    def pitch_rad(self, quats: np.ndarray) -> np.ndarray:
        """Pitch (rad) from filtered quaternions — same convention as ml_pipeline.quat_to_pitch_rad."""
        quats = np.asarray(quats, dtype=np.float64)
        q0, qx, qy, qz = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
        sin_pitch = 2.0 * (q0 * qy - qz * qx)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
        return np.arcsin(sin_pitch)

    # ------------------------------------------------------------------
    # internal step functions
    # ------------------------------------------------------------------

    def _update_imu(self, q: np.ndarray, g: np.ndarray, a: np.ndarray) -> np.ndarray:
        """Single-step Madgwick update (accel + gyro only)."""
        q0, q1, q2, q3 = q
        dt = self.dt
        beta = self.beta

        # Normalise accel; skip update if near zero (sensor flat or free-fall)
        a_norm = np.linalg.norm(a)
        if a_norm < 1e-6:
            return self._integrate_gyro(q, g, dt)
        ax, ay, az = a / a_norm

        # Gradient descent step: objective function F = q* ⊗ g_hat ⊗ q - a_hat
        # where g_hat = [0,0,0,1] (gravity in world frame)
        F1 = 2.0 * (q1 * q3 - q0 * q2) - ax
        F2 = 2.0 * (q0 * q1 + q2 * q3) - ay
        F3 = 2.0 * (0.5 - q1 ** 2 - q2 ** 2) - az

        J11 = -2.0 * q2;  J12 =  2.0 * q3;  J13 = -2.0 * q0;  J14 =  2.0 * q1
        J21 =  2.0 * q1;  J22 =  2.0 * q0;  J23 =  2.0 * q3;  J24 =  2.0 * q2
        J31 =  0.0;        J32 = -4.0 * q1;  J33 = -4.0 * q2;  J34 =  0.0

        grad0 = J11 * F1 + J21 * F2 + J31 * F3
        grad1 = J12 * F1 + J22 * F2 + J32 * F3
        grad2 = J13 * F1 + J23 * F2 + J33 * F3
        grad3 = J14 * F1 + J24 * F2 + J34 * F3
        grad = np.array([grad0, grad1, grad2, grad3])
        g_norm = np.linalg.norm(grad)
        if g_norm > 1e-10:
            grad /= g_norm

        # Gyro quaternion derivative
        q_dot = 0.5 * self._quat_mult(q, np.array([0.0, g[0], g[1], g[2]]))

        # Fuse
        q_new = q + (q_dot - beta * grad) * dt
        return q_new / np.linalg.norm(q_new)

    def _update_marg(
        self, q: np.ndarray, g: np.ndarray, a: np.ndarray, m: np.ndarray
    ) -> np.ndarray:
        """Single-step MARG update (accel + gyro + magnetometer)."""
        q0, q1, q2, q3 = q
        dt = self.dt
        beta = self.beta

        a_norm = np.linalg.norm(a)
        if a_norm < 1e-6:
            return self._integrate_gyro(q, g, dt)
        ax, ay, az = a / a_norm

        m_norm = np.linalg.norm(m)
        if m_norm < 1e-6:
            return self._update_imu(q, g, a)
        mx, my, mz = m / m_norm

        # Reference direction of Earth's magnetic field in world frame
        hx = 2.0 * mx * (0.5 - q2**2 - q3**2) + 2.0 * my * (q1*q2 - q0*q3) + 2.0 * mz * (q1*q3 + q0*q2)
        hy = 2.0 * mx * (q1*q2 + q0*q3) + 2.0 * my * (0.5 - q1**2 - q3**2) + 2.0 * mz * (q2*q3 - q0*q1)
        hz = 2.0 * mx * (q1*q3 - q0*q2) + 2.0 * my * (q2*q3 + q0*q1) + 2.0 * mz * (0.5 - q1**2 - q2**2)
        bx = float(np.sqrt(hx**2 + hy**2))
        bz = float(hz)

        # Objective function (accel + mag)
        F1 = 2.0 * (q1*q3 - q0*q2) - ax
        F2 = 2.0 * (q0*q1 + q2*q3) - ay
        F3 = 2.0 * (0.5 - q1**2 - q2**2) - az
        F4 = 2.0*bx*(0.5 - q2**2 - q3**2) + 2.0*bz*(q1*q3 - q0*q2) - mx
        F5 = 2.0*bx*(q1*q2 - q0*q3)       + 2.0*bz*(q0*q1 + q2*q3) - my
        F6 = 2.0*bx*(q0*q2 + q1*q3)       + 2.0*bz*(0.5 - q1**2 - q2**2) - mz

        # Jacobian (6x4) rows = F, cols = q0..q3
        J = np.array([
            [-2*q2,    2*q3,   -2*q0,    2*q1],
            [ 2*q1,    2*q0,    2*q3,    2*q2],
            [ 0,      -4*q1,   -4*q2,    0   ],
            [-2*bz*q2, 2*bz*q3, -4*bx*q2-2*bz*q0, -4*bx*q3+2*bz*q1],
            [-2*bx*q3, 2*bx*q2-2*bz*q1,  2*bx*q1+2*bz*q3, -2*bx*q0-2*bz*q2],  # noqa: E501 (intentionally long)
            [ 2*bx*q2, 2*bx*q3-2*bz*q0,  2*bx*q0+2*bz*q3,  2*bx*q1-4*bz*q2],  # noqa: E501
        ])
        F_vec = np.array([F1, F2, F3, F4, F5, F6])
        grad = J.T @ F_vec
        g_norm = np.linalg.norm(grad)
        if g_norm > 1e-10:
            grad /= g_norm

        q_dot = 0.5 * self._quat_mult(q, np.array([0.0, g[0], g[1], g[2]]))
        q_new = q + (q_dot - beta * grad) * dt
        return q_new / np.linalg.norm(q_new)

    @staticmethod
    def _integrate_gyro(q: np.ndarray, g: np.ndarray, dt: float) -> np.ndarray:
        """Pure gyro integration (when accel is unreliable)."""
        q_dot = 0.5 * MadgwickFilter._quat_mult(q, np.array([0.0, g[0], g[1], g[2]]))
        q_new = q + q_dot * dt
        n = np.linalg.norm(q_new)
        return q_new / n if n > 1e-10 else q

    @staticmethod
    def _quat_mult(p: np.ndarray, r: np.ndarray) -> np.ndarray:
        """Hamilton product of two quaternions [w, x, y, z]."""
        p0, p1, p2, p3 = p
        r0, r1, r2, r3 = r
        return np.array([
            p0*r0 - p1*r1 - p2*r2 - p3*r3,
            p0*r1 + p1*r0 + p2*r3 - p3*r2,
            p0*r2 - p1*r3 + p2*r0 + p3*r1,
            p0*r3 + p1*r2 - p2*r1 + p3*r0,
        ])


# ---------------------------------------------------------------------------
# Convenience wrapper used by ml_pipeline.py
# ---------------------------------------------------------------------------

def apply_filter(
    gyro: np.ndarray,
    accel: np.ndarray,
    quat: np.ndarray,
    filter_name: str,
    sample_rate: float = 128.0,
    kalman_q: float = 1e-3,
    kalman_r: float = 1e-1,
    madgwick_beta: float = 0.033,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply the requested filter and return (gyro_out, accel_out, quat_out).

    Parameters
    ----------
    gyro, accel, quat : raw arrays from the CSV loader (N, 3) and (N, 4).
    filter_name : one of {'none', 'kalman', 'madgwick'}.
    sample_rate : sensor sample rate in Hz.
    kalman_q, kalman_r : Kalman process / measurement noise.
    madgwick_beta : Madgwick gain.

    Returns
    -------
    gyro_out  : (N, 3) — gyro_z may be replaced by smoothed signal.
    accel_out : (N, 3) — unchanged for kalman; smoothed axes for madgwick.
    quat_out  : (N, 4) — unchanged for kalman; drift-corrected for madgwick.
    """
    if filter_name == "none" or filter_name is None:
        return gyro, accel, quat

    if filter_name == "kalman":
        kf = KalmanFilter1D(process_noise=kalman_q, measurement_noise=kalman_r, dt=1.0 / sample_rate)
        gyro_out = gyro.copy()
        # Smooth all three gyro axes independently
        gyro_out[:, 0] = kf.filter(gyro[:, 0])
        gyro_out[:, 1] = kf.filter(gyro[:, 1])
        gyro_out[:, 2] = kf.filter(gyro[:, 2])
        return gyro_out, accel, quat

    if filter_name == "madgwick":
        mf = MadgwickFilter(beta=madgwick_beta, sample_rate=sample_rate)
        quat_out = mf.filter(gyro, accel)
        # Derive a clean yaw-rate from filtered orientation → replace gyro_z
        yr = mf.yaw_rate(quat_out)
        gyro_out = gyro.copy()
        gyro_out[:, 2] = yr
        return gyro_out, accel, quat_out

    raise ValueError(f"Unknown filter '{filter_name}'. Choose from: none, kalman, madgwick")
