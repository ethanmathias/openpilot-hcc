import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3

try:
  from beamngpy import BeamNGpy, Scenario, Vehicle
  from beamngpy.sensors import Camera, Electrics
except ImportError:  # pragma: no cover
  BeamNGpy = None
  Scenario = None
  Vehicle = None
  Camera = None
  Electrics = None


def _load_speed_profile(csv_path: str, scenario_index: int) -> tuple[np.ndarray, np.ndarray]:
  data = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
  if data.ndim == 1:
    data = np.expand_dims(data, axis=0)
  if scenario_index >= data.shape[1]:
    raise ValueError(f"Scenario index {scenario_index} is out of bounds for {csv_path}")
  speeds = data[:, scenario_index]
  speeds = speeds[np.isfinite(speeds)]
  if len(speeds) < 2:
    raise ValueError("Need at least two valid speed samples in scenario CSV")
  times = np.arange(len(speeds), dtype=np.float64) * 0.1
  return times, speeds.astype(np.float64)


@dataclass
class BeamNGConfig:
  host: str = "localhost"
  port: int = 64256
  home: str | None = None
  map_name: str = "west_coast_usa"
  scenario_name: str = "OpenpilotBeamNG"
  ego_vehicle_model: str = "etk800"
  lead_vehicle_model: str = "etk800"
  step_s: float = 0.01
  scenario_index: int | None = None
  scenario_csv: str | None = None


class BeamNGWorld(World):
  def __init__(self, config: BeamNGConfig, dual_camera: bool = False):
    super().__init__(dual_camera)
    if BeamNGpy is None or Scenario is None or Vehicle is None or Camera is None or Electrics is None:
      raise RuntimeError("beamngpy with Camera and Electrics support is required for the BeamNG bridge")

    self.config = config
    self._pending_reset = False
    self._prev_v_rel = 0.0
    self._target_speed_pre = 0.0
    self._sim_time = 0.0
    self._profile_times = None
    self._profile_speeds = None
    if config.scenario_index is not None and config.scenario_csv is not None:
      self._profile_times, self._profile_speeds = _load_speed_profile(config.scenario_csv, int(config.scenario_index))

    self.beamng = BeamNGpy(config.host, config.port, home=config.home)
    self._open_beamng()
    self._spawn_scenario()

  @property
  def target_speed_pre(self) -> float:
    return float(self._target_speed_pre)

  def _open_beamng(self):
    open_attempts = [
      lambda: self.beamng.open(None, "-gfx", "vk"),
      lambda: self.beamng.open(),
    ]
    last_error = None
    for attempt in open_attempts:
      try:
        attempt()
        break
      except TypeError as exc:
        last_error = exc
    else:
      if last_error is not None:
        raise last_error

    for method_name, args in (
      ("set_deterministic", ()),
      ("set_steps_per_second", (int(round(1.0 / self.config.step_s)),)),
    ):
      method = getattr(self.beamng, method_name, None)
      if callable(method):
        try:
          method(*args)
        except TypeError:
          pass

  def _spawn_scenario(self):
    scenario = Scenario(self.config.map_name, self.config.scenario_name)
    self.ego_vehicle = Vehicle("ego_vehicle", model=self.config.ego_vehicle_model, licence="EGO", colour="Blue")
    self.preceding_vehicle = Vehicle("preceding_vehicle", model=self.config.lead_vehicle_model, licence="PRE", colour="Red")

    ego_start_pos = (-800.3449955, -464.0569456, 106.624861)
    ego_start_rot = (0.0017, 0.0053, 0.9169, -0.3991)
    pre_start_pos = (-793.2977155, -457.3357649, 106.6151421)
    pre_start_rot = (0.0014, 0.0043, 0.9235, -0.3836)

    scenario.add_vehicle(self.ego_vehicle, pos=ego_start_pos, rot_quat=ego_start_rot)
    scenario.add_vehicle(self.preceding_vehicle, pos=pre_start_pos, rot_quat=pre_start_rot)
    scenario.make(self.beamng)
    self.beamng.load_scenario(scenario)

    self._attach_sensors()
    self.beamng.start_scenario()

    if hasattr(self.preceding_vehicle, "ai_set_mode"):
      self.preceding_vehicle.ai_set_mode("span")
    if hasattr(self.preceding_vehicle, "ai_drive_in_lane"):
      self.preceding_vehicle.ai_drive_in_lane(True)
    if hasattr(self.preceding_vehicle, "ai_set_aggression"):
      self.preceding_vehicle.ai_set_aggression(0.0)
    if hasattr(self.ego_vehicle, "set_shift_mode"):
      self.ego_vehicle.set_shift_mode("arcade")

  def _make_camera(self, field_of_view_y: int):
    attempts = [
      lambda: Camera(
        pos=(0.0, 0.0, 1.22),
        dir=(1.0, 0.0, 0.0),
        up=(0.0, 0.0, 1.0),
        resolution=(W, H),
        field_of_view_y=field_of_view_y,
        is_render_colours=True,
        is_render_annotations=False,
        is_render_depth=False,
        is_render_instance=False,
      ),
      lambda: Camera(
        (0.0, 0.0, 1.22),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
        resolution=(W, H),
        field_of_view_y=field_of_view_y,
        colour=True,
        annotation=False,
        depth=False,
      ),
    ]
    last_error = None
    for attempt in attempts:
      try:
        return attempt()
      except TypeError as exc:
        last_error = exc
    if last_error is not None:
      raise last_error
    raise RuntimeError("Failed to construct BeamNG camera sensor")

  def _attach_sensors(self):
    self.ego_vehicle.attach_sensor("electrics", Electrics())
    self.preceding_vehicle.attach_sensor("electrics", Electrics())
    self.ego_vehicle.attach_sensor("road_camera", self._make_camera(60))
    if self.dual_camera:
      self.ego_vehicle.attach_sensor("wide_road_camera", self._make_camera(120))

  def _heading_from_state(self, vehicle_state) -> float:
    direction = vehicle_state.get("dir")
    if direction is not None:
      return float(math.atan2(direction[1], direction[0]))
    velocity = vehicle_state.get("vel", (0.0, 0.0, 0.0))
    if np.linalg.norm(velocity[:2]) > 1e-6:
      return float(math.atan2(velocity[1], velocity[0]))
    return 0.0

  def _camera_to_rgb(self, camera_data) -> np.ndarray:
    colour = camera_data
    if isinstance(camera_data, dict):
      colour = camera_data.get("colour", camera_data)
    if hasattr(colour, "convert"):
      colour = colour.convert("RGB")
    array = np.array(colour, dtype=np.uint8)
    if array.ndim == 2:
      array = np.repeat(array[:, :, None], 3, axis=2)
    if array.shape[-1] == 4:
      array = array[:, :, :3]
    return np.ascontiguousarray(array[:, :, :3])

  def apply_controls(self, steer_sim, throttle_out, brake_out, bridge_telemetry=None):
    self.ego_vehicle.control(throttle=float(throttle_out), brake=float(brake_out), steering=float(steer_sim))

  def tick(self):
    if self._pending_reset:
      restart = getattr(self.beamng, "restart_scenario", None)
      if callable(restart):
        restart()
      self._pending_reset = False
      self._sim_time = 0.0

    self.beamng.step(1)
    self._sim_time += self.config.step_s
    if self._profile_times is not None and self._profile_speeds is not None:
      self._target_speed_pre = float(np.interp(self._sim_time, self._profile_times, self._profile_speeds))
      if hasattr(self.preceding_vehicle, "ai_set_speed"):
        self.preceding_vehicle.ai_set_speed(self._target_speed_pre, mode="set")

  def read_state(self):
    pass

  def read_sensors(self, simulator_state: SimulatorState):
    self.ego_vehicle.poll_sensors()
    self.preceding_vehicle.poll_sensors()

    ego_state = self.ego_vehicle.state
    lead_state = self.preceding_vehicle.state
    ego_velocity = ego_state.get("vel", (0.0, 0.0, 0.0))
    lead_velocity = lead_state.get("vel", (0.0, 0.0, 0.0))
    ego_position = ego_state.get("pos", (0.0, 0.0, 0.0))
    lead_position = lead_state.get("pos", (0.0, 0.0, 0.0))
    ego_heading = self._heading_from_state(ego_state)

    simulator_state.velocity = vec3(float(ego_velocity[0]), float(ego_velocity[1]), float(ego_velocity[2]))
    simulator_state.position_xy = (float(ego_position[0]), float(ego_position[1]))
    simulator_state.bearing = float(math.degrees(ego_heading))
    simulator_state.imu.bearing = simulator_state.bearing
    simulator_state.gps.from_xy(simulator_state.position_xy)

    electrics_sensor = self.ego_vehicle.sensors.get("electrics") if hasattr(self.ego_vehicle, "sensors") else None
    electrics_data = getattr(electrics_sensor, "data", {}) if electrics_sensor is not None else {}
    simulator_state.steering_angle = float(electrics_data.get("steering", 0.0))

    lead_delta_xy = np.array(lead_position[:2], dtype=np.float64) - np.array(ego_position[:2], dtype=np.float64)
    ego_forward = np.array([math.cos(ego_heading), math.sin(ego_heading)], dtype=np.float64)
    ego_left = np.array([-math.sin(ego_heading), math.cos(ego_heading)], dtype=np.float64)
    d_rel = float(np.dot(lead_delta_xy, ego_forward))
    y_rel = float(np.dot(lead_delta_xy, ego_left))
    ego_speed = float(np.linalg.norm(ego_velocity[:2]))
    lead_speed = float(np.linalg.norm(lead_velocity[:2]))
    v_rel = float(lead_speed - ego_speed)
    a_rel = float((v_rel - self._prev_v_rel) / self.config.step_s)
    self._prev_v_rel = v_rel

    simulator_state.lead_status = d_rel > 0.0
    simulator_state.lead_d_rel = float(d_rel)
    simulator_state.lead_y_rel = float(y_rel)
    simulator_state.lead_v_rel = float(v_rel)
    simulator_state.lead_a_rel = float(a_rel)
    simulator_state.debug_has_lane = False
    simulator_state.debug_on_lane = False
    simulator_state.debug_lane_s = float("nan")
    simulator_state.debug_lane_lateral = float("nan")
    simulator_state.debug_lane_heading_error_deg = float("nan")
    simulator_state.debug_on_yellow_line = False
    simulator_state.debug_on_white_line = False
    simulator_state.debug_crash_sidewalk = False
    simulator_state.debug_out_of_route = False
    simulator_state.valid = True

  def read_cameras(self):
    try:
      self.ego_vehicle.poll_sensors()
      road_sensor = self.ego_vehicle.sensors.get("road_camera") if hasattr(self.ego_vehicle, "sensors") else None
      self.road_image[:] = self._camera_to_rgb(getattr(road_sensor, "data", road_sensor))
      if self.dual_camera:
        wide_sensor = self.ego_vehicle.sensors.get("wide_road_camera")
        self.wide_road_image[:] = self._camera_to_rgb(getattr(wide_sensor, "data", wide_sensor))
    except Exception:
      self.road_image[:] = 0
      if self.dual_camera:
        self.wide_road_image[:] = 0
    finally:
      self.image_lock.release()

  def close(self, reason: str):
    self.exit_event.set()
    self.image_lock.release()
    self.beamng.close()

  def reset(self):
    self._pending_reset = True
