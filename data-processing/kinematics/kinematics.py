"""
  Imu Data: 
    - acc (x, y, z) m/s^2
    - gyro (x, y, z) rad/s
    - mag (x, y, z) uT

  GPS Data (NMEA 0183): 
    - $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
      - format
      - UTC (172814.0)
      - Latitude (3723.46587704)
      - Direction (N)
      - Longitude (12202.26957864)
      - Direction (W)
      - GPS Quality 
        0: Fix not valid
        1: GPS fix
        2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
        3: Not applicable
        4: RTK Fixed, xFill
        5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
        6: INS Dead reckoning
      ...
      - Orthometric height (m)
    - GPRMC,203522.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,130522,0.0,E,D*2B

    -- Latitude and Longitude

  Output frame @ t: 
    - orientation
      roll, pitch, yaw in radians
    - angular velocity 
      roll_rate, pitch_rate, yaw_rate in rad/s
    - linear acceleration (gravity-free, in body frame)
      a_forward, a_lateral, a_vertical in m/s2
    - position
      lat, lon, altitude
    - velocity
      speed and vertical speed in m/s 
"""

from dataclasses import dataclass
from ahrs.filters import Madgwick
import numpy as np

@dataclass
class ImuSample:
  acc: np.ndarray     # [ax, ay, az] in m/s2
  gyro: np.ndarray    # [gx, gy, gz] in rad/s
  mag: np.ndarray     # [mx, my, mz] in uT

@dataclass
class GpsSample:
  lat: float          # decimal deg
  lon: float          # decimal deg
  altitude: float     # meters
  speed: float        # m/s
  fix_quality: int

@dataclass
class MergedDataSample: 
  t: float            # timestamp (s)
  imu: ImuSample
  gps: GpsSample

@dataclass
class KinematicFrame: 
  t: float
  # orientation (rad)
  roll: float
  pitch: float
  yaw: float
  # angular velocity (rad/s)
  roll_rate: float
  pitch_rate: float
  yaw_rate: float
  # linear accelaration (m/s2, body frame, gravity removed)
  a_forward: float
  a_lateral: float
  a_vertical: float
  # position
  lat: float
  lon: float  
  alt: float # (m)
  # velocity (m/s)
  speed: float
  vertical_speed: float

GRAVITY = np.array([0.0, 0.0, 9.8])

def quat_to_euler(q) -> tuple[float, float, float]: 
  w, x, y, z = q
  roll = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
  pitch = np.arcsin(np.clip(2 * (w*y - x*z), -1.0, 1.0))
  yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

  return roll, pitch, yaw

def quat_to_rotmat(q) -> np.ndarray: 
  w, x, y, z = q
  return np.array([
    [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)], 
    [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)], 
    [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]
  ])

class Kinematics: 
  def __init__(self, update_freq_hz=100):
    self.imu_filter = Madgwick(frequency=update_freq_hz)
    self.dt = 1 / update_freq_hz
    self.q = np.array([1.0, 0.0, 0.0, 0.0])
    self.prev_t = None

  def process(self, sample: MergedDataSample) -> KinematicFrame: 
    #TODO: Convert stats from imu frame to body frame based on the way it will clip to the boot or calibration
    # This way, will be easier to process data later on
    # body frame is x forward, y lateral and z vertical for example

    if self.prev_t is None: 
      dt = self.dt
    else: 
      dt = sample.t - self.prev_t

    self.prev_t = sample.t

    self.q = self.imu_filter.updateMARG(
      q=self.q, 
      acc=sample.imu.acc, 
      gyr=sample.imu.gyro, 
      mag=sample.imu.mag*1_000
    )

    roll, pitch, yaw = quat_to_euler(self.q)

    # TODO: add filter (LPF?)
    roll_rate, pitch_rate, yaw_rate = sample.imu.gyro

    R = quat_to_rotmat(self.q)
    gravity_body = R.T @ GRAVITY
    lin_acc = sample.imu.acc - gravity_body

    a_forward, a_lateral, a_vertical = lin_acc

    return KinematicFrame(
      t=sample.t, 

      roll=roll, 
      pitch=pitch, 
      yaw=yaw,

      roll_rate=roll_rate,
      pitch_rate=pitch_rate,
      yaw_rate=yaw_rate,

      a_forward=a_forward,
      a_lateral=a_lateral,
      a_vertical=a_vertical,

      lat=sample.gps.lat,
      lon=sample.gps.lon,
      altitude=sample.gps.altitude,

      speed=sample.gps.speed, 
      # TODO: add vertical speed calculation using altitude data trends
      vertical_speed=None
    )



