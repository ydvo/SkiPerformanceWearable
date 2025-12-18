import numpy as np
from typing import Dict, Any

def synth_imu_9dof(cfg: Dict[str, Any] | None = None) -> Dict[str, np.ndarray]:
    """
    Generate synthetic 9-DoF IMU data (gyro, accel, mag) from a known trajectory
    with configurable noise, bias, drift, and calibration errors.

    Returns a dict with:
      time [N], Fs, q_true [N,4] (w,x,y,z), euler_true [N,3] (yaw,pitch,roll, ZYX),
      gyro_true [N,3] (rad/s), acc_true [N,3] (m/s^2), mag_true [N,3],
      gyro/accel/mag measured [N,3], params (effective config).
    """
    # -------------------- Defaults --------------------
    def_cfg: Dict[str, Any] = {
        "Fs": 200.0,
        "T": 60.0,
        "g_n": np.array([0.0, 0.0, 9.81]),
        "m_n": np.array([0.25, 0.0, 0.45]),
        # Trajectory: Euler ZYX (yaw, pitch, roll) in radians
        "yaw_fun":   lambda t: np.deg2rad(60*np.sin(2*np.pi*0.03*t) + 30*np.sin(2*np.pi*0.01*t)),
        "pitch_fun": lambda t: np.deg2rad(8*np.sin(2*np.pi*0.07*t)),
        "roll_fun":  lambda t: np.deg2rad(6*np.sin(2*np.pi*0.11*t) - 3),
        # Optional linear acceleration in nav frame (m/s^2); return [N,3]
        "a_nav_fun": lambda t: np.column_stack([
            0.4*np.sin(2*np.pi*0.20*t),
            0.3*np.sin(2*np.pi*0.15*t),
            0.0*t
        ]),
        # Gyro error model
        "gyro": {
            "bias0":     np.deg2rad(np.array([0.5, -0.4, 0.3])),
            "bias_rw":   np.deg2rad(0.02),    # rad/s / sqrt(s)
            "noise_std": np.deg2rad(0.02),    # rad/s
            "scale_mis": np.eye(3) + np.diag([0.002, -0.003, 0.001]),
        },
        # Accel error model
        "acc": {
            "bias":      np.array([0.02, -0.03, 0.01]),  # m/s^2
            "noise_std": 0.05,                           # m/s^2
            "scale_mis": np.eye(3) + np.diag([0.003, 0.002, -0.002]),
        },
        # Mag error model
        "mag": {
            "hard_iron": np.array([0.02, -0.01, 0.015]),
            "soft_iron": np.array([[1.05, 0.01, 0.00],
                                   [0.00, 0.98, 0.02],
                                   [0.00, 0.00, 1.02]]),
            "noise_std": 0.01,
        },
        # RNG seed (set to None to leave RNG state unchanged)
        "rng_seed": 7,
    }

    cfg = _deep_merge(def_cfg, cfg or {})

    # -------------------- Timebase & truth attitude --------------------
    Fs = float(cfg["Fs"])
    dt = 1.0 / Fs
    T  = float(cfg["T"])
    t  = np.arange(0.0, T + dt, dt)
    N  = t.size

    if cfg["rng_seed"] is not None:
        rng = np.random.default_rng(cfg["rng_seed"])
    else:
        rng = np.random.default_rng()

    yaw   = cfg["yaw_fun"](t)
    pitch = cfg["pitch_fun"](t)
    roll  = cfg["roll_fun"](t)

    q_true = eul2quat_zyx(yaw, pitch, roll)              # [N,4] (w,x,y,z)
    R_nb   = quat2rotm_batch(q_true)                     # body <- nav (N,3,3)
    R_bn   = np.transpose(R_nb, (0,2,1))                 # nav <- body

    # -------------------- True body-frame signals --------------------
    omega_true = np.zeros((N,3))
    for k in range(1, N):
        dq = quat_multiply(q_true[k], quat_conj(q_true[k-1]))
        c = np.clip(dq[0], -1.0, 1.0)
        ang = 2.0 * np.arccos(c)
        if ang < 1e-9:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            s2 = np.sin(ang/2.0)
            axis = dq[1:4] / s2
            nrm = np.linalg.norm(axis)
            axis = axis / (nrm if nrm > 0 else 1.0)
        omega_true[k] = (ang/dt) * axis

    a_nav = cfg["a_nav_fun"](t)  # [N,3]

    g_n = cfg["g_n"].reshape(3)
    m_n = cfg["m_n"].reshape(3)

    acc_true = (R_bn @ (g_n + a_nav).reshape(N,3,1)).squeeze(-1)  # [N,3]
    mag_true = (R_bn @ m_n.reshape(3,1)).squeeze(-1)              # [N,3]

    # -------------------- Sensor error models --------------------
    # Gyro: bias random walk + white noise + scale/misalignment
    gyro_cfg = cfg["gyro"]
    bg = np.zeros((N,3))
    bg[0] = gyro_cfg["bias0"].reshape(3)
    for k in range(1, N):
        bg[k] = bg[k-1] + np.sqrt(dt) * gyro_cfg["bias_rw"] * rng.standard_normal(3)

    gyro_meas = (omega_true + bg) @ gyro_cfg["scale_mis"].T \
                + gyro_cfg["noise_std"] * rng.standard_normal((N,3))

    # Accel: bias + scale/misalignment + white noise
    acc_cfg = cfg["acc"]
    accel_meas = (acc_true + acc_cfg["bias"].reshape(1,3)) @ acc_cfg["scale_mis"].T \
                 + acc_cfg["noise_std"] * rng.standard_normal((N,3))

    # Mag: soft-iron (3x3), hard-iron (3,), + white noise
    mag_cfg = cfg["mag"]
    mag_meas = (mag_true @ mag_cfg["soft_iron"].T) + mag_cfg["hard_iron"].reshape(1,3) \
               + mag_cfg["noise_std"] * rng.standard_normal((N,3))

    # -------------------- Pack & return --------------------
    out = {
        "time":        t,
        "Fs":          Fs,
        "q_true":      q_true,
        "euler_true":  np.stack([yaw, pitch, roll], axis=1),
        "gyro_true":   omega_true,
        "acc_true":    acc_true,
        "mag_true":    mag_true,
        "gyro":        gyro_meas,
        "accel":       accel_meas,
        "mag":         mag_meas,
        "params":      cfg,
    }
    return out

# -------------------- Small helpers --------------------

def _deep_merge(a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(a)
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out

def eul2quat_zyx(yaw: np.ndarray, pitch: np.ndarray, roll: np.ndarray) -> np.ndarray:
    """
    Convert ZYX Euler (yaw, pitch, roll) to quaternion [w,x,y,z]; vectorized.
    """
    cy = np.cos(yaw * 0.5);  sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5);  sr = np.sin(roll * 0.5)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    q = np.stack([w, x, y, z], axis=-1)
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return q

def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton product, both [w,x,y,z].
    """
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat2rotm_batch(q: np.ndarray) -> np.ndarray:
    """
    Batch quaternion -> rotation matrix (body <- nav), [N,3,3].
    """
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w,x,y,z = q[:,0], q[:,1], q[:,2], q[:,3]
    R = np.empty((q.shape[0], 3, 3))
    R[:,0,0] = 1 - 2*(y*y + z*z)
    R[:,0,1] = 2*(x*y - z*w)
    R[:,0,2] = 2*(x*z + y*w)
    R[:,1,0] = 2*(x*y + z*w)
    R[:,1,1] = 1 - 2*(x*x + z*z)
    R[:,1,2] = 2*(y*z - x*w)
    R[:,2,0] = 2*(x*z - y*w)
    R[:,2,1] = 2*(y*z + x*w)
    R[:,2,2] = 1 - 2*(x*x + y*y)
    return R
