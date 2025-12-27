from kinematics import KinematicFrame
from dataclasses import dataclass

"""
  Features to be extracted: 
  - turns
  - edges
  - stability
  - air time
  - falls

  Need to define sane thresholds and logic to interpret filtered signals represented in KinematicFrame
"""

@dataclass
class Turn: 
  start_t: float
  end_t: float
  duration: float
  direction: str # left or right
  # yaw rate stats (peak, mean, min)
  peak_yaw_rate: float
  # edge angle during turn (peak, mean, min)
  peak_edge_angle: float
  avg_speed: float

@dataclass
class SessionFeatures: 
  turns: list[Turn]
  turn_count: int
  turn_frequency: float
  avg_turn_duration: float
  turn_symmetry: float

  max_edge_angle: float
  avg_edge_angle: float
  carving_ratio: float

  max_speed: float
  avg_speed: float
  # std dev
  speed_variability: float

  # TODO: airtime? 

  # the distance travelled uphill 
  elevation_gain: float
  # vertical distance travelled from the top of mountain down to the base
  vertical_drop: float
  # the ratio of the vertical drop to the horizontal distance (steepness of the mountain)
  avg_gradient: float

  duration: float
  distance: float





