import math
import multiprocessing
import numpy as np

from abc import ABC, abstractmethod
from collections import namedtuple

W, H = 1928, 1208


vec3 = namedtuple("vec3", ["x", "y", "z"])

class GPSState:
  def __init__(self):
    self.latitude = 0
    self.longitude = 0
    self.altitude = 0

  def from_xy(self, xy):
    """Simulates a lat/lon from an xy coordinate on a plane, for simple simulation. TODO: proper global projection?"""
    BASE_LAT = 32.75308505188913
    BASE_LON = -117.2095393365393
    DEG_TO_METERS = 100000

    self.latitude = float(BASE_LAT + xy[0] / DEG_TO_METERS)
    self.longitude = float(BASE_LON + xy[1] / DEG_TO_METERS)
    self.altitude = 0


class IMUState:
  def __init__(self):
    self.accelerometer: vec3 = vec3(0,0,0)
    self.gyroscope: vec3 = vec3(0,0,0)
    self.bearing: float = 0


class SimulatorState:
  def __init__(self):
    self.valid = False
    self.is_engaged = False
    self.ignition = True

    self.velocity: vec3 = None
    self.position_xy: tuple[float, float] | None = None
    self.bearing: float = 0
    self.gps = GPSState()
    self.imu = IMUState()
    self.debug_has_lane: bool = False
    self.debug_on_lane: bool = False
    self.debug_lane_s: float | None = None
    self.debug_lane_lateral: float | None = None
    self.debug_lane_heading_error_deg: float | None = None
    self.debug_on_yellow_line: bool = False
    self.debug_on_white_line: bool = False
    self.debug_crash_sidewalk: bool = False
    self.debug_out_of_route: bool = False

    self.steering_angle: float = 0

    self.user_gas: float = 0
    self.user_brake: float = 0
    self.user_torque: float = 0

    self.cruise_button = 0

    self.left_blinker = False
    self.right_blinker = False

    self.carstate_a_ego: float = 0.0
    self.planner_a_target: float = 0.0
    self.hccc_accel: float = 0.0
    self.manual_accel: float = 0.0
    self.final_accel: float = 0.0
    self.hccc_active: bool = False

    # Optional virtual lead state for simulator-generated liveTracks.
    self.lead_status: bool = False
    self.lead_d_rel: float = 0.0
    self.lead_y_rel: float = 0.0
    self.lead_v_rel: float = 0.0
    self.lead_a_rel: float = 0.0

  @property
  def speed(self):
    return math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2 + self.velocity.z ** 2)


class World(ABC):
  def __init__(self, dual_camera):
    self.dual_camera = dual_camera

    self.image_lock = multiprocessing.Semaphore(value=0)
    self.road_image = np.zeros((H, W, 3), dtype=np.uint8)
    self.wide_road_image = np.zeros((H, W, 3), dtype=np.uint8)

    self.exit_event = multiprocessing.Event()

  @abstractmethod
  def apply_controls(self, steer_sim, throttle_out, brake_out, bridge_telemetry=None):
    pass

  @abstractmethod
  def tick(self):
    pass

  @abstractmethod
  def read_state(self):
    pass

  @abstractmethod
  def read_sensors(self, simulator_state: SimulatorState):
    pass

  @abstractmethod
  def read_cameras(self):
    pass

  @abstractmethod
  def close(self, reason: str):
    pass

  @abstractmethod
  def reset(self):
    pass
