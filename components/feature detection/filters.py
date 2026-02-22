"""
Noise-reduction filters for ski boot IMU data.

Three independent filters:
  - KalmanFilter1D: zero-phase (forward-backward) scalar Kalman filter.
    Smooths gyroscope axes without introducing phase lag.
  - MadgwickFilter: quaternion-based sensor fusion (accel + gyro).
    Produces drift-corrected orientation; yaw-rate derived from quaternion
    differences.  Best used for orientation/lean, not as a gyro_z replacement.
  - butterworth_lowpass: simple 2nd-order Butterworth low-pass filter
    (zero-phase via filtfilt).  The most predictable smoother for turn
    detection — removes high-frequency noise while preserving turn peaks.

Usage
-----
from filters import apply_filter

# In ml_pipeline.py:
gyro_out, accel_out, quat_out = apply_filter(gyro, accel, quat, "kalman")
"""

import numpy as np


# ---------------------------------------------------------------------------
# Butterworth low-pass (zero-phase, scipy-free fallback included)
# ---------------------------------------------------------------------------

def _sosfiltfilt(sos, x):
    """Zero-phase SOS filter: forward then backward pass."""
    from scipy.signal import sosfilt
    y = sosfilt(sos, x)
    y = sosfilt(sos, y[::-1])[::-1]
    return y


def butterworth_lowpass(
    signal: np.ndarray,
    sample_rate: float = 128.0,
    cutoff_hz: float = 5.0,
    order: int = 2,
) -> np.ndarray:
    """Zero-phase Butterworth low-pass filter on a 1-D signal.

    Parameters
    ----------
    signal      : (N,) array
    sample_rate : Hz
    cutoff_hz   : -3 dB cutoff frequency.  5 Hz keeps turn dynamics
                  (turns last ~0.5-2 s → 0.5-2 Hz) while removing
                  high-frequency vibration and sensor noise.
    order       : filter order (2 is gentle; 4 is sharper).

    Returns
    -------
    (N,) smoothed array (zero phase lag).
    """
    from scipy.signal import butter
    nyq = sample_rate / 2.0
    if cutoff_hz >= nyq:
        return signal.copy()
    sos = butter(order, cutoff_hz / nyq, btype="low", output="sos")
    return _sosfiltfilt(sos, np.asarray(signal, dtype=np.float64))


# ---------------------------------------------------------------------------
# Kalman 1-D — zero-phase (forward + backward pass)
# ---------------------------------------------------------------------------

class KalmanFilter1D:
    """Zero-phase scalar Kalman filter.

    Runs a standard predict/update pass forward, then a second pass backward,
    and averages the two estimates.  This eliminates the phase lag inherent in
    a single-pass Kalman filter while retaining its adaptive smoothing.

    The state model is position-only (random walk) which is a better match for
    oscillatory gyroscope data than the constant-velocity model.

    Parameters
    ----------
    process_noise : float
        Process noise variance (Q).  Controls how fast the filter can track
        changes.  Larger → less smoothing, tracks fast changes.
    measurement_noise : float
        Measurement noise variance (R).  Larger → more smoothing.
    dt : float
        Sample interval (seconds).  Used to scale Q properly.
    """

    def __init__(
        self,
        process_noise: float = 5e-2,
        measurement_noise: float = 5e-1,
        dt: float = 1.0 / 128.0,
    ):
        self.dt = dt
        self.q = float(process_noise)
        self.r = float(measurement_noise)

    def _forward(self, signal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Single forward Kalman pass.  Returns (estimates, variances)."""
        n = len(signal)
        est = np.empty(n)
        var = np.empty(n)

        x = signal[0]
        P = 1.0

        q, r = self.q, self.r

        for k in range(n):
            # Predict (random-walk: x_pred = x, P_pred = P + Q)
            P_pred = P + q

            # Update
            S = P_pred + r
            K = P_pred / S
            x = x + K * (signal[k] - x)
            P = (1.0 - K) * P_pred

            est[k] = x
            var[k] = P

        return est, var

    def filter(self, signal: np.ndarray) -> np.ndarray:
        """Apply zero-phase Kalman filter (forward + backward, variance-weighted merge).

        Parameters
        ----------
        signal : (N,) array

        Returns
        -------
        (N,) smoothed array with zero phase lag.
        """
        signal = np.asarray(signal, dtype=np.float64)
        if len(signal) < 2:
            return signal.copy()

        fwd_est, fwd_var = self._forward(signal)
        bwd_est, bwd_var = self._forward(signal[::-1])
        bwd_est = bwd_est[::-1]
        bwd_var = bwd_var[::-1]

        # Variance-weighted combination (optimal under Gaussian assumption)
        w_fwd = 1.0 / (fwd_var + 1e-15)
        w_bwd = 1.0 / (bwd_var + 1e-15)
        out = (w_fwd * fwd_est + w_bwd * bwd_est) / (w_fwd + w_bwd)
        return out


# ---------------------------------------------------------------------------
# Madgwick orientation filter
# ---------------------------------------------------------------------------

class MadgwickFilter:
    """Quaternion-based sensor fusion filter (Madgwick 2010).

    Fuses 3-axis gyroscope and 3-axis accelerometer to produce a
    drift-corrected orientation quaternion.

    Parameters
    ----------
    beta : float
        Filter gain.  Higher = faster correction of gyro drift but more
        sensitivity to accel noise.  For dynamic motion (skiing) use a low
        value (0.005-0.02) so centripetal acceleration doesn't corrupt the
        orientation.  Default 0.01.
    sample_rate : float
        Sensor sample rate in Hz (default 128.0).
    """

    def __init__(self, beta: float = 0.01, sample_rate: float = 128.0):
        self.beta = float(beta)
        self.dt = 1.0 / float(sample_rate)

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
        accel : (N, 3) accelerometer readings in m/s^2  [ax, ay, az]
        mag   : (N, 3) optional magnetometer readings.
        q0    : (4,) initial quaternion [w, x, y, z].

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

        Uses a low-pass filter on the raw yaw derivative to suppress the
        amplification of quantization noise from numerical differentiation.

        Parameters
        ----------
        quats : (N, 4) quaternion array [w, x, y, z]

        Returns
        -------
        yaw_rate : (N,) array in rad/s.
        """
        quats = np.asarray(quats, dtype=np.float64)
        n = len(quats)
        if n < 2:
            return np.zeros(n)

        yaw = np.arctan2(
            2.0 * (quats[:, 0] * quats[:, 3] + quats[:, 1] * quats[:, 2]),
            1.0 - 2.0 * (quats[:, 2] ** 2 + quats[:, 3] ** 2),
        )
        yaw = np.unwrap(yaw)

        dyaw = np.diff(yaw) / self.dt
        dyaw = np.concatenate([[dyaw[0]], dyaw])

        # Smooth the differentiated signal to suppress noise amplification
        try:
            dyaw = butterworth_lowpass(dyaw, 1.0 / self.dt, cutoff_hz=3.0, order=2)
        except ImportError:
            # scipy not available — use simple moving average as fallback
            win = max(1, int(0.1 * (1.0 / self.dt)))  # ~100 ms window
            kernel = np.ones(win) / win
            dyaw = np.convolve(dyaw, kernel, mode="same")

        return dyaw

    def pitch_rad(self, quats: np.ndarray) -> np.ndarray:
        """Pitch (rad) from filtered quaternions."""
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

        a_norm = np.linalg.norm(a)
        if a_norm < 1e-6:
            return self._integrate_gyro(q, g, dt)

        # Reject accelerometer correction when accel magnitude deviates
        # significantly from gravity — indicates dynamic motion (turns, bumps)
        gravity = 9.806
        accel_deviation = abs(a_norm - gravity) / gravity
        effective_beta = beta * max(0.0, 1.0 - accel_deviation * 3.0)

        ax, ay, az = a / a_norm

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

        q_dot = 0.5 * self._quat_mult(q, np.array([0.0, g[0], g[1], g[2]]))

        q_new = q + (q_dot - effective_beta * grad) * dt
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

        gravity = 9.806
        accel_deviation = abs(a_norm - gravity) / gravity
        effective_beta = beta * max(0.0, 1.0 - accel_deviation * 3.0)

        hx = 2.0 * mx * (0.5 - q2**2 - q3**2) + 2.0 * my * (q1*q2 - q0*q3) + 2.0 * mz * (q1*q3 + q0*q2)
        hy = 2.0 * mx * (q1*q2 + q0*q3) + 2.0 * my * (0.5 - q1**2 - q3**2) + 2.0 * mz * (q2*q3 - q0*q1)
        hz = 2.0 * mx * (q1*q3 - q0*q2) + 2.0 * my * (q2*q3 + q0*q1) + 2.0 * mz * (0.5 - q1**2 - q2**2)
        bx = float(np.sqrt(hx**2 + hy**2))
        bz = float(hz)

        F1 = 2.0 * (q1*q3 - q0*q2) - ax
        F2 = 2.0 * (q0*q1 + q2*q3) - ay
        F3 = 2.0 * (0.5 - q1**2 - q2**2) - az
        F4 = 2.0*bx*(0.5 - q2**2 - q3**2) + 2.0*bz*(q1*q3 - q0*q2) - mx
        F5 = 2.0*bx*(q1*q2 - q0*q3)       + 2.0*bz*(q0*q1 + q2*q3) - my
        F6 = 2.0*bx*(q0*q2 + q1*q3)       + 2.0*bz*(0.5 - q1**2 - q2**2) - mz

        J = np.array([
            [-2*q2,    2*q3,   -2*q0,    2*q1],
            [ 2*q1,    2*q0,    2*q3,    2*q2],
            [ 0,      -4*q1,   -4*q2,    0   ],
            [-2*bz*q2, 2*bz*q3, -4*bx*q2-2*bz*q0, -4*bx*q3+2*bz*q1],
            [-2*bx*q3, 2*bx*q2-2*bz*q1,  2*bx*q1+2*bz*q3, -2*bx*q0-2*bz*q2],
            [ 2*bx*q2, 2*bx*q3-2*bz*q0,  2*bx*q0+2*bz*q3,  2*bx*q1-4*bz*q2],
        ])
        F_vec = np.array([F1, F2, F3, F4, F5, F6])
        grad = J.T @ F_vec
        g_norm = np.linalg.norm(grad)
        if g_norm > 1e-10:
            grad /= g_norm

        q_dot = 0.5 * self._quat_mult(q, np.array([0.0, g[0], g[1], g[2]]))
        q_new = q + (q_dot - effective_beta * grad) * dt
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
# Pre-smoothing helper for the threshold detector
# ---------------------------------------------------------------------------

def smooth_for_peak_detection(
    gyro_z: np.ndarray,
    sample_rate: float = 128.0,
    cutoff_hz: float = 2.0,
) -> np.ndarray:
    """Low-pass filter gyro_z before peak detection.

    A 2 Hz cutoff preserves turn dynamics (turns are 0.5-2 Hz events) while
    aggressively removing vibration, foot-strike, and sensor noise that cause
    false peaks.
    """
    try:
        return butterworth_lowpass(gyro_z, sample_rate, cutoff_hz=cutoff_hz, order=2)
    except ImportError:
        # Fallback: moving average (~0.25 s window at 128 Hz)
        win = max(1, int(0.25 * sample_rate))
        kernel = np.ones(win) / win
        return np.convolve(np.asarray(gyro_z, dtype=np.float64), kernel, mode="same")


# ---------------------------------------------------------------------------
# Convenience wrapper used by ml_pipeline.py
# ---------------------------------------------------------------------------

def apply_filter(
    gyro: np.ndarray,
    accel: np.ndarray,
    quat: np.ndarray,
    filter_name: str,
    sample_rate: float = 128.0,
    kalman_q: float = 5e-2,
    kalman_r: float = 5e-1,
    madgwick_beta: float = 0.01,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply the requested filter and return (gyro_out, accel_out, quat_out).

    Parameters
    ----------
    gyro, accel, quat : raw arrays from the CSV loader (N, 3) and (N, 4).
    filter_name : one of {'none', 'kalman', 'madgwick', 'butterworth'}.
    sample_rate : sensor sample rate in Hz.
    kalman_q, kalman_r : Kalman process / measurement noise.
    madgwick_beta : Madgwick gain.

    Returns
    -------
    gyro_out  : (N, 3) — smoothed gyro axes.
    accel_out : (N, 3) — unchanged for kalman/butterworth; unchanged for madgwick.
    quat_out  : (N, 4) — unchanged for kalman/butterworth; drift-corrected for madgwick.
    """
    if filter_name == "none" or filter_name is None:
        return gyro, accel, quat

    if filter_name == "kalman":
        kf = KalmanFilter1D(process_noise=kalman_q, measurement_noise=kalman_r, dt=1.0 / sample_rate)
        gyro_out = gyro.copy()
        gyro_out[:, 0] = kf.filter(gyro[:, 0])
        gyro_out[:, 1] = kf.filter(gyro[:, 1])
        gyro_out[:, 2] = kf.filter(gyro[:, 2])
        return gyro_out, accel, quat

    if filter_name == "butterworth":
        gyro_out = gyro.copy()
        gyro_out[:, 0] = butterworth_lowpass(gyro[:, 0], sample_rate, cutoff_hz=5.0)
        gyro_out[:, 1] = butterworth_lowpass(gyro[:, 1], sample_rate, cutoff_hz=5.0)
        gyro_out[:, 2] = butterworth_lowpass(gyro[:, 2], sample_rate, cutoff_hz=5.0)
        return gyro_out, accel, quat

    if filter_name == "madgwick":
        mf = MadgwickFilter(beta=madgwick_beta, sample_rate=sample_rate)
        quat_out = mf.filter(gyro, accel)
        gyro_out = gyro.copy()
        gyro_out[:, 0] = butterworth_lowpass(gyro[:, 0], sample_rate, cutoff_hz=5.0)
        gyro_out[:, 1] = butterworth_lowpass(gyro[:, 1], sample_rate, cutoff_hz=5.0)
        gyro_out[:, 2] = butterworth_lowpass(gyro[:, 2], sample_rate, cutoff_hz=5.0)
        return gyro_out, accel, quat_out

    if filter_name == "combined":
        return _apply_combined(
            gyro, accel, quat, sample_rate,
            kalman_q, kalman_r, madgwick_beta,
        )

    raise ValueError(
        f"Unknown filter '{filter_name}'. "
        "Choose from: none, kalman, madgwick, butterworth, combined"
    )


# ---------------------------------------------------------------------------
# Combined adaptive filter
# ---------------------------------------------------------------------------

# Windows with NSR above this are "high-noise transition" windows where
# smoothing can introduce classification errors by clipping asymmetric tails.
_COMBINED_NSR_THRESHOLD = 4.0

# Minimum samples in a window for the adaptive blend (must match WINDOW_SAMPLES
# in ml_pipeline; imported at call time to avoid circular dependency).
_COMBINED_WINDOW_SAMPLES = 128


def _apply_combined(
    gyro: np.ndarray,
    accel: np.ndarray,
    quat: np.ndarray,
    sample_rate: float,
    kalman_q: float,
    kalman_r: float,
    madgwick_beta: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Adaptive combination of Kalman + Butterworth, falling back to raw
    in high-noise transition windows where smoothing degrades accuracy.

    Strategy
    --------
    1. Compute two filtered versions of each gyro axis:
       - Kalman (zero-phase, adaptive to rate changes)
       - Butterworth (fixed-bandwidth, zero-phase)
    2. For each analysis window (128 samples at 128 Hz = 1 s):
       - Compute the noise-to-signal ratio (NSR = std / |mean|) of the
         *raw* signal in that window.
       - If NSR > threshold (high-noise transition window): keep the raw
         signal, because smoothing would clip asymmetric tails and shift
         the window mean across a classification boundary.
       - If NSR <= threshold (clean signal): use the average of Kalman
         and Butterworth, which reduces noise without shifting the mean
         significantly.
    3. The Madgwick quaternion is also computed and returned for orientation/
       lean analysis (it doesn't affect the gyro output).

    This exploits the finding that Kalman and Butterworth produce correlated
    errors (both shift means in the same direction on transition windows),
    so averaging them doesn't cancel the bias — but detecting the vulnerable
    windows and falling back to raw does.
    """
    gyro = np.asarray(gyro, dtype=np.float64)
    accel = np.asarray(accel, dtype=np.float64)
    quat = np.asarray(quat, dtype=np.float64)
    n = len(gyro)
    ws = _COMBINED_WINDOW_SAMPLES

    # --- Filtered versions ---
    kf = KalmanFilter1D(process_noise=kalman_q, measurement_noise=kalman_r,
                        dt=1.0 / sample_rate)
    gyro_kalman = gyro.copy()
    for ax in range(3):
        gyro_kalman[:, ax] = kf.filter(gyro[:, ax])

    gyro_bw = gyro.copy()
    for ax in range(3):
        gyro_bw[:, ax] = butterworth_lowpass(gyro[:, ax], sample_rate, cutoff_hz=5.0)

    # --- Madgwick for quaternion (orientation/lean) ---
    mf = MadgwickFilter(beta=madgwick_beta, sample_rate=sample_rate)
    quat_out = mf.filter(gyro, accel)

    # --- Adaptive per-window blend ---
    gyro_out = gyro.copy()
    n_windows = n // ws
    nsr_threshold = _COMBINED_NSR_THRESHOLD

    for wi in range(n_windows):
        s = wi * ws
        e = s + ws
        for ax in range(3):
            raw_win = gyro[s:e, ax]
            raw_mean = raw_win.mean()
            raw_std = raw_win.std()
            abs_mean = abs(raw_mean)

            if abs_mean < 1e-6:
                nsr = raw_std / 1e-6
            else:
                nsr = raw_std / abs_mean

            if nsr > nsr_threshold:
                # High-noise transition window: raw signal is more reliable
                gyro_out[s:e, ax] = raw_win
            else:
                # Clean window: average of Kalman and Butterworth
                gyro_out[s:e, ax] = 0.5 * (gyro_kalman[s:e, ax] + gyro_bw[s:e, ax])

    # Handle trailing samples (< 1 full window) with simple average
    tail_start = n_windows * ws
    if tail_start < n:
        for ax in range(3):
            gyro_out[tail_start:, ax] = 0.5 * (
                gyro_kalman[tail_start:, ax] + gyro_bw[tail_start:, ax]
            )

    return gyro_out, accel, quat_out
