import csv
import math
import re
import time
from collections import namedtuple
from dataclasses import dataclass, field
from multiprocessing.connection import Connection
from pathlib import Path

import numpy as np
from panda3d.core import Vec3

from metadrive.component.vehicle.vehicle_type import vehicle_type
from metadrive.engine.core.engine_core import EngineCore
from metadrive.engine.core.image_buffer import ImageBuffer
from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.obs.image_obs import ImageObservation
from metadrive.policy.idm_policy import IDMPolicy

from openpilot.common.realtime import Ratekeeper
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import vec3

# Camera mounting used by both road and wide streams in MetaDrive.
C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0, 0)

LEAD_STEER_BLEND_IDM = 0.2
LEAD_STEER_BLEND_LANE = 0.8
EGO_LANE_LOOKAHEAD_M = 6.0
HCCC_PARITY_DT_S = 0.1
HCCC_PARITY_BETA = 0.65

# IPC message payloads shared with MetaDriveWorld.
metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple(
  "metadrive_vehicle_state",
  [
    "velocity",
    "position",
    "bearing",
    "steering_angle",
    "lead_status",
    "lead_d_rel",
    "lead_y_rel",
    "lead_v_rel",
    "lead_a_rel",
    "lead_vehicle_valid",
    "lead_vehicle_velocity",
    "lead_vehicle_bearing",
    "lead_vehicle_steering_angle",
    "debug_has_lane",
    "debug_on_lane",
    "debug_lane_s",
    "debug_lane_lateral",
    "debug_lane_heading_error_deg",
    "debug_on_yellow_line",
    "debug_on_white_line",
    "debug_crash_sidewalk",
    "debug_out_of_route",
  ],
)


# Lead vehicle config/state containers.
@dataclass
class LeadConfig:
  """Configuration knobs for spawning and driving the synthetic lead vehicle."""
  enabled: bool
  distance_m: float
  start_delay_s: float
  lateral_offset_m: float
  model: str
  render: bool
  profile_scn: int | None
  profile_csv: str | None
  output_csv: str | None
  output_graph: str | None
  output_control_method: str | None
  output_vehicle_name: str | None


@dataclass
class LeadState:
  """Mutable runtime state for the synthetic lead vehicle."""
  vehicle: object | None = None
  policy: IDMPolicy | None = None
  start_time: float | None = None
  measurement: dict[str, float | bool] = field(default_factory=lambda: _lead_measurement(False))
  prev_v_rel: float = 0.0
  profile_t: np.ndarray | None = None
  profile_s: np.ndarray | None = None
  profile_v: np.ndarray | None = None
  profile_lane: object | None = None
  profile_s_base: float | None = None
  pose_replay_failed: bool = False
  # These fields are logged to the replay CSV so we can distinguish a clean
  # pose-replay step from controller fallback during analysis.
  pose_replay_applied: bool = False
  pose_replay_fallback_active: bool = False
  target_speed_mps: float | None = None


# Geometry and profile utility helpers.
def _lead_measurement(enabled=False, d_rel=0.0, y_rel=0.0, v_rel=0.0, a_rel=0.0):
  """Create a normalized lead measurement dict used by bridge IPC."""
  return {
    "status": bool(enabled),
    "d_rel": float(d_rel),
    "y_rel": float(y_rel),
    "v_rel": float(v_rel),
    "a_rel": float(a_rel),
  }


def _compute_rel(ego_position, ego_heading, lead_position):
  """Project lead position into ego forward/left relative coordinates."""
  cos_heading = math.cos(float(ego_heading))
  sin_heading = math.sin(float(ego_heading))
  ego_forward_unit = np.array([cos_heading, sin_heading], dtype=np.float64)
  ego_left_unit = np.array([-sin_heading, cos_heading], dtype=np.float64)
  lead_delta_xy = np.array(lead_position, dtype=np.float64) - np.array(ego_position, dtype=np.float64)
  return float(np.dot(lead_delta_xy, ego_forward_unit)), float(np.dot(lead_delta_xy, ego_left_unit))


def _wrap_to_pi(angle: float) -> float:
  """Wrap any angle in radians to [-pi, pi)."""
  return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _planar_speed(velocity_vector_xy) -> float:
  """Return planar speed magnitude from XY velocity components."""
  return float(np.linalg.norm([velocity_vector_xy[0], velocity_vector_xy[1]]))


def _lane_identifier(lane) -> str:
  """Build a readable lane identifier string for debug logging."""
  if lane is None:
    return "<none>"

  parts = [lane.__class__.__name__]
  for attr in ("index", "lane_index", "name"):
    value = getattr(lane, attr, None)
    if value is not None:
      parts.append(f"{attr}={value}")
  return " ".join(parts)


def _ego_debug_state(vehicle):
  """Collect lane and route diagnostics for the ego vehicle."""
  debug_state = {
    "has_lane": False,
    "on_lane": False,
    "lane_s": float("nan"),
    "lane_lateral": float("nan"),
    "lane_heading_error_deg": float("nan"),
    "lane_id": _lane_identifier(getattr(vehicle, "lane", None)),
    "on_yellow_line": bool(getattr(vehicle, "on_yellow_continuous_line", False)),
    "on_white_line": bool(getattr(vehicle, "on_white_continuous_line", False)),
    "crash_sidewalk": bool(getattr(vehicle, "crash_sidewalk", False)),
    "out_of_route": bool(getattr(vehicle, "out_of_route", False)),
  }

  lane = getattr(vehicle, "lane", None)
  if lane is None:
    return debug_state

  try:
    lane_s, lane_lateral = lane.local_coordinates(vehicle.position)
    p0 = np.array(lane.position(float(lane_s), 0.0), dtype=np.float64)[:2]
    p1 = np.array(lane.position(float(lane_s) + 1.0, 0.0), dtype=np.float64)[:2]
    tangent = p1 - p0
    lane_heading_error_deg = float("nan")
    if np.linalg.norm(tangent) > 1e-6:
      lane_heading = float(math.atan2(tangent[1], tangent[0]))
      lane_heading_error_deg = math.degrees(_wrap_to_pi(lane_heading - float(vehicle.heading_theta)))

    lane_width = getattr(lane, "width", None)
    on_lane = False
    if lane_width is not None:
      on_lane = abs(float(lane_lateral)) <= (float(lane_width) * 0.5)

    debug_state.update({
      "has_lane": True,
      "on_lane": on_lane,
      "lane_s": float(lane_s),
      "lane_lateral": float(lane_lateral),
      "lane_heading_error_deg": float(lane_heading_error_deg),
    })
  except Exception:
    pass

  return debug_state


def _lead_debug_state(vehicle):
  return _ego_debug_state(vehicle)


def _format_debug_state(prefix: str, debug: dict, position_xy, heading_theta: float) -> str:
  """Format a single-line, human-readable debug state summary."""
  position_xy_arr = np.array(position_xy, dtype=np.float64)[:2]
  return (
    f"{prefix}: pos=({position_xy_arr[0]:.2f}, {position_xy_arr[1]:.2f}) "
    f"bearing={math.degrees(float(heading_theta)):.2f} deg "
    f"laneId={debug['lane_id']} hasLane={debug['has_lane']} onLane={debug['on_lane']} "
    f"s={debug['lane_s']:.2f} latOff={debug['lane_lateral']:.2f} "
    f"hdgErr={debug['lane_heading_error_deg']:.2f} deg "
    f"yellow={debug['on_yellow_line']} white={debug['on_white_line']} "
    f"sidewalk={debug['crash_sidewalk']} outOfRoute={debug['out_of_route']}"
  )


def _load_lead_speed_profile(csv_path: str, scenario_index: int):
  """Load a scenario speed profile from CSV and integrate it to distance."""
  if scenario_index < 1:
    raise ValueError(f"Scenario index must be >= 1, got {scenario_index}")

  time_samples_s: list[float] = []
  speed_samples_mps: list[float] = []
  with open(csv_path, newline="") as csv_file:
    csv_reader = csv.reader(csv_file)
    for csv_row in csv_reader:
      if not csv_row or not csv_row[0].strip():
        continue

      try:
        timestamp_s = float(csv_row[0].strip())
      except ValueError:
        continue

      if scenario_index >= len(csv_row):
        if speed_samples_mps:
          break
        continue

      speed_text = csv_row[scenario_index].strip()
      if not speed_text:
        if speed_samples_mps:
          break
        continue

      try:
        speed_mps = max(float(speed_text), 0.0)
      except ValueError:
        if speed_samples_mps:
          break
        continue

      time_samples_s.append(timestamp_s)
      speed_samples_mps.append(speed_mps)

  if len(time_samples_s) < 2:
    raise RuntimeError(f"Scenario {scenario_index} has insufficient samples in {csv_path}")

  time_arr = np.asarray(time_samples_s, dtype=np.float64)
  speed_arr = np.asarray(speed_samples_mps, dtype=np.float64)
  sort_index = np.argsort(time_arr)
  time_arr = time_arr[sort_index]
  speed_arr = speed_arr[sort_index]

  unique_time_s, first_unique_index = np.unique(time_arr, return_index=True)
  unique_speed_mps = speed_arr[first_unique_index]
  if unique_time_s.size < 2:
    raise RuntimeError(f"Scenario {scenario_index} needs at least two unique time samples in {csv_path}")

  # Integrate speed profile into cumulative longitudinal distance (meters).
  cumulative_distance_m = np.zeros_like(unique_speed_mps)
  for sample_idx in range(1, unique_time_s.size):
    delta_t_s = max(float(unique_time_s[sample_idx] - unique_time_s[sample_idx - 1]), 0.0)
    cumulative_distance_m[sample_idx] = (
      cumulative_distance_m[sample_idx - 1] +
      0.5 * (unique_speed_mps[sample_idx - 1] + unique_speed_mps[sample_idx]) * delta_t_s
    )

  return unique_time_s, cumulative_distance_m, unique_speed_mps


def _profile_interp_position(lead_state: LeadState, elapsed_s: float) -> float:
  """Interpolate cumulative longitudinal profile distance at elapsed time."""
  assert lead_state.profile_t is not None and lead_state.profile_s is not None
  return float(
    np.interp(
      max(elapsed_s, 0.0),
      lead_state.profile_t,
      lead_state.profile_s,
      left=lead_state.profile_s[0],
      right=lead_state.profile_s[-1],
    )
  )


def _profile_interp_speed(lead_state: LeadState, elapsed_s: float) -> float:
  """Interpolate target lead speed at elapsed time."""
  assert lead_state.profile_t is not None and lead_state.profile_v is not None
  return float(
    np.interp(
      max(elapsed_s, 0.0),
      lead_state.profile_t,
      lead_state.profile_v,
      left=lead_state.profile_v[0],
      right=lead_state.profile_v[-1],
    )
  )


def _set_vehicle_pose_2d(vehicle, position_xy, heading_theta: float, speed_mps: float):
  """Best-effort direct pose/speed injection for a MetaDrive vehicle."""
  target_x = float(position_xy[0])
  target_y = float(position_xy[1])
  position_set = False

  if hasattr(vehicle, "set_position"):
    try:
      vehicle.set_position([target_x, target_y])
      position_set = True
    except Exception:
      pass
  if not position_set and hasattr(vehicle, "set_pos"):
    try:
      vehicle.set_pos([target_x, target_y])
      position_set = True
    except Exception:
      pass
  if not position_set:
    try:
      vehicle.position = [target_x, target_y]
      position_set = True
    except Exception:
      pass

  heading_set = False
  for setter in ("set_heading_theta", "set_heading"):
    if hasattr(vehicle, setter):
      try:
        getattr(vehicle, setter)(float(heading_theta))
        heading_set = True
        break
      except Exception:
        pass
  if not heading_set:
    try:
      vehicle.heading_theta = float(heading_theta)
      heading_set = True
    except Exception:
      pass

  velocity_x = float(math.cos(heading_theta) * speed_mps)
  velocity_y = float(math.sin(heading_theta) * speed_mps)
  for setter in ("set_velocity",):
    if hasattr(vehicle, setter):
      try:
        getattr(vehicle, setter)([velocity_x, velocity_y])
        break
      except Exception:
        pass

  return position_set and heading_set


# MetaDrive runtime patch points for bridge behavior.
def _patch_metadrive(arrive_dest_done: bool):
  """Monkey-patch MetaDrive internals to match bridge runtime expectations."""
  # Inject sensor and termination behavior patches to keep bridge semantics stable.
  def add_image_sensor_patched(self, name: str, cls, args):
    """Create image sensors with optional CUDA support."""
    use_cuda = bool(self.global_config.get("image_on_cuda", False))
    sensor = cls(*args, self, cuda=use_cuda)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  def observe_patched(self, *args, **kwargs):
    """Return cached image observation state without extra rendering work."""
    return self.state

  ImageObservation.observe = observe_patched

  if not arrive_dest_done:
    def arrive_destination_patch(self, *args, **kwargs):
      """Disable destination-complete termination."""
      return False
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch

  # Keep the simulator running even if MetaDrive thinks the vehicle is out of road.
  def out_of_road_patch(self, *args, **kwargs):
    """Disable out-of-road termination to keep episodes running."""
    return False
  MetaDriveEnv._is_out_of_road = out_of_road_patch


def _vehicle_cls_for_model(model_name, fallback_cls):
  """Resolve a MetaDrive vehicle class for a model name with fallback."""
  vehicle_cls = vehicle_type.get(model_name)
  return vehicle_cls if vehicle_cls is not None else fallback_cls


def _candidate_models(primary_model, ego_model):
  """Generate de-duplicated lead model candidates in priority order."""
  candidate_order = [primary_model, ego_model, "s", "m", "l", "xl", "default"]
  return [model_name for i, model_name in enumerate(candidate_order) if model_name and model_name not in candidate_order[:i]]


def _lane_center_steer(vehicle, lane) -> float | None:
  """Compute steering correction to center a vehicle within its current lane."""
  if lane is None:
    return None

  try:
    lane_s_coord, lane_lateral_offset = lane.local_coordinates(vehicle.position)
    lane_point_start = np.array(lane.position(float(lane_s_coord), 0.0), dtype=np.float64)[:2]
    lane_point_end = np.array(lane.position(float(lane_s_coord) + 1.0, 0.0), dtype=np.float64)[:2]
    lane_tangent = lane_point_end - lane_point_start
    if np.linalg.norm(lane_tangent) <= 1e-6:
      return None

    lane_heading = float(math.atan2(lane_tangent[1], lane_tangent[0]))
    heading_error = _wrap_to_pi(lane_heading - float(vehicle.heading_theta))
    return float(np.clip(1.2 * heading_error - 0.18 * float(lane_lateral_offset), -1.0, 1.0))
  except Exception:
    return None


def _ego_lane_fallback_steer(lead_vehicle, ego_vehicle, lead_distance_m: float) -> float | None:
  """Fallback steering target that keeps lead ahead on ego's lane centerline."""
  try:
    ego_lane = getattr(ego_vehicle, "lane", None)
    if ego_lane is None:
      return None

    ego_lane_s, _ = ego_lane.local_coordinates(ego_vehicle.position)
    lead_target_s = float(ego_lane_s + lead_distance_m + EGO_LANE_LOOKAHEAD_M)
    lead_target_xy = np.array(ego_lane.position(lead_target_s, 0.0), dtype=np.float64)[:2]
    lead_current_xy = np.array(lead_vehicle.position, dtype=np.float64)[:2]
    lead_to_target_vec = lead_target_xy - lead_current_xy
    if np.linalg.norm(lead_to_target_vec) <= 1e-6:
      return None

    desired_heading = float(math.atan2(lead_to_target_vec[1], lead_to_target_vec[0]))
    heading_error = _wrap_to_pi(desired_heading - float(lead_vehicle.heading_theta))
    return float(np.clip(1.8 * heading_error, -1.0, 1.0))
  except Exception:
    return None


def _lead_spawn_pose(ego_vehicle, lead_cfg: LeadConfig):
  """Compute initial XY/heading for spawning the synthetic lead vehicle."""
  fallback_heading_rad = float(ego_vehicle.heading_theta)
  heading_unit_vec = np.array([math.cos(fallback_heading_rad), math.sin(fallback_heading_rad)], dtype=np.float64)
  fallback_position_xy = np.array(ego_vehicle.position, dtype=np.float64) + heading_unit_vec * lead_cfg.distance_m
  if abs(lead_cfg.lateral_offset_m) > 1e-3:
    lane_left_unit_vec = np.array([-heading_unit_vec[1], heading_unit_vec[0]], dtype=np.float64)
    fallback_position_xy += lane_left_unit_vec * lead_cfg.lateral_offset_m

  lane = getattr(ego_vehicle, "lane", None)
  if lane is None:
    return fallback_position_xy, fallback_heading_rad

  try:
    ego_s, _ = lane.local_coordinates(ego_vehicle.position)
    target_s = float(ego_s + lead_cfg.distance_m)
    lane_target = np.array(lane.position(target_s, lead_cfg.lateral_offset_m), dtype=np.float64)
    target_position = lane_target[:2]

    p0 = np.array(lane.position(target_s, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
    p1 = np.array(lane.position(target_s + 0.5, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
    tangent = p1 - p0
    if np.linalg.norm(tangent) > 1e-6:
      target_heading = float(math.atan2(tangent[1], tangent[0]))
    else:
      target_heading = fallback_heading_rad

    return target_position, target_heading
  except Exception:
    return fallback_position_xy, fallback_heading_rad


# Lead vehicle lifecycle and update loop.
def _spawn_lead_vehicle(env: MetaDriveEnv, lead_cfg: LeadConfig, lead_state: LeadState):
  """Spawn and initialize the lead vehicle plus optional policy/profile state."""
  lead_state.vehicle = None
  lead_state.policy = None
  lead_state.start_time = None
  lead_state.profile_lane = None
  lead_state.profile_s_base = None
  lead_state.pose_replay_failed = False

  if not lead_cfg.enabled:
    return

  ego_vehicle = env.vehicle
  target_position, target_heading = _lead_spawn_pose(ego_vehicle, lead_cfg)
  ego_model = ego_vehicle.config.get("vehicle_model", None)

  lead_vehicle_config_template = dict(env.config.get("vehicle_config", {}))
  lead_vehicle_config_template["render_vehicle"] = lead_cfg.render
  for key in ("show_navi_mark", "show_dest_mark", "show_line_to_dest", "show_line_to_navi_mark"):
    if key in lead_vehicle_config_template:
      lead_vehicle_config_template[key] = False

  fallback_vehicle_cls = ego_vehicle.__class__
  spawn_error = None
  for model_name in _candidate_models(lead_cfg.model, ego_model):
    candidate_vehicle_config = dict(lead_vehicle_config_template)
    candidate_vehicle_config["vehicle_model"] = model_name
    candidate_vehicle_cls = _vehicle_cls_for_model(model_name, fallback_vehicle_cls)

    try:
      lead_state.vehicle = env.engine.spawn_object(
        candidate_vehicle_cls,
        vehicle_config=candidate_vehicle_config,
        position=target_position.tolist(),
        heading=target_heading,
      )
      if lead_state.profile_t is None:
        policy_seed = int((time.time() * 1000) % (2**31 - 1))
        lead_state.policy = IDMPolicy(lead_state.vehicle, policy_seed)
      else:
        lead_state.policy = None
      lead_state.start_time = time.monotonic()

      if lead_state.profile_t is not None and lead_state.profile_s is not None:
        lane_for_profile = getattr(lead_state.vehicle, "lane", None) or getattr(env.vehicle, "lane", None)
        if lane_for_profile is not None:
          lead_s, _ = lane_for_profile.local_coordinates(lead_state.vehicle.position)
          lead_state.profile_lane = lane_for_profile
          lead_state.profile_s_base = float(lead_s - lead_state.profile_s[0])

      print(f"[INFO] Spawned lead vehicle model '{model_name}'")
      return
    except (OSError, FileNotFoundError) as e:
      spawn_error = e
      print(f"[WARNING] Failed lead model '{model_name}': {e}")
    except Exception as e:
      spawn_error = e
      print(f"[WARNING] Lead spawn error for '{model_name}': {e}")

  print(f"[WARNING] Lead vehicle disabled after model spawn failures: {spawn_error}")


def _clear_lead_vehicle(env: MetaDriveEnv, lead_state: LeadState):
  """Remove lead vehicle from simulation and reset lead runtime state."""
  lead_vehicle = lead_state.vehicle
  if lead_vehicle is not None:
    cleared = False
    clear_error_messages = []

    candidate_object_keys = []
    for attr in ("name", "id", "index"):
      value = getattr(lead_vehicle, attr, None)
      if value is not None:
        candidate_object_keys.append(value)

    for object_key in candidate_object_keys:
      try:
        env.engine.clear_objects([object_key])
        cleared = True
        break
      except Exception as e:
        clear_error_messages.append(str(e))

    if not cleared:
      try:
        env.engine.clear_objects([lead_vehicle])
        cleared = True
      except Exception as e:
        clear_error_messages.append(str(e))

    if not cleared and hasattr(lead_vehicle, "destroy"):
      try:
        lead_vehicle.destroy()
      except Exception as e:
        clear_error_messages.append(str(e))

    if not cleared and clear_error_messages:
      print(f"[WARNING] Failed to clear lead vehicle before reset: {' | '.join(clear_error_messages)}")

  lead_state.vehicle = None
  lead_state.policy = None
  lead_state.start_time = None
  lead_state.profile_lane = None
  lead_state.profile_s_base = None
  lead_state.pose_replay_failed = False
  lead_state.pose_replay_applied = False
  lead_state.pose_replay_fallback_active = False
  lead_state.target_speed_mps = None


def _update_lead_vehicle(env: MetaDriveEnv, lead_cfg: LeadConfig, lead_state: LeadState):
  """Advance lead vehicle behavior for one bridge step."""
  if lead_state.vehicle is None:
    return

  try:
    elapsed_s = 0.0 if lead_state.start_time is None else (time.monotonic() - lead_state.start_time)
    lead_state.pose_replay_applied = False
    lead_state.pose_replay_fallback_active = False
    lead_state.target_speed_mps = None
    if lead_cfg.start_delay_s > 0.0 and elapsed_s < lead_cfg.start_delay_s:
      lead_action = np.array([0.0, 0.0], dtype=np.float64)
    elif lead_state.profile_t is not None and lead_state.profile_s is not None:
      if lead_state.profile_lane is None:
        lane_for_profile = getattr(lead_state.vehicle, "lane", None) or getattr(env.vehicle, "lane", None)
        if lane_for_profile is not None:
          lead_s, _ = lane_for_profile.local_coordinates(lead_state.vehicle.position)
          lead_state.profile_lane = lane_for_profile
          lead_state.profile_s_base = float(lead_s - lead_state.profile_s[0])

      if lead_state.profile_lane is not None and lead_state.profile_s_base is not None:
        profile_distance_m = _profile_interp_position(lead_state, elapsed_s - lead_cfg.start_delay_s)
        target_speed_mps = _profile_interp_speed(lead_state, elapsed_s - lead_cfg.start_delay_s)
        lead_state.target_speed_mps = float(target_speed_mps)
        target_s = float(lead_state.profile_s_base + profile_distance_m)

        p0 = np.array(lead_state.profile_lane.position(target_s, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
        p1 = np.array(lead_state.profile_lane.position(target_s + 0.5, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
        tangent = p1 - p0
        if np.linalg.norm(tangent) > 1e-6:
          heading = float(math.atan2(tangent[1], tangent[0]))
        else:
          heading = float(lead_state.vehicle.heading_theta)

        if not _set_vehicle_pose_2d(lead_state.vehicle, p0, heading, max(target_speed_mps, 0.0)):
          if not lead_state.pose_replay_failed:
            print("[WARNING] Lead pose replay failed; falling back to controller tracking.")
            lead_state.pose_replay_failed = True
          lead_state.pose_replay_fallback_active = True
        else:
          lead_state.pose_replay_failed = False
          lead_state.pose_replay_applied = True
          lead_action = np.array([0.0, 0.0], dtype=np.float64)
          lead_state.vehicle.before_step(lead_action.tolist())
          return

      target_speed_mps = _profile_interp_speed(lead_state, elapsed_s - lead_cfg.start_delay_s)
      lead_state.target_speed_mps = float(target_speed_mps)
      current_speed_mps = _planar_speed(lead_state.vehicle.velocity)
      longitudinal_command = float(np.clip((target_speed_mps - current_speed_mps) * 0.35, -1.0, 1.0))
      lane_steer = _lane_center_steer(lead_state.vehicle, getattr(lead_state.vehicle, "lane", None))
      if lane_steer is None:
        lane_steer = _ego_lane_fallback_steer(lead_state.vehicle, env.vehicle, lead_cfg.distance_m)
      steer_cmd = 0.0 if lane_steer is None else float(np.clip(lane_steer, -1.0, 1.0))
      lead_action = np.array([steer_cmd, longitudinal_command], dtype=np.float64)
    else:
      if lead_state.policy is None:
        return
      lead_action = np.array(lead_state.policy.act(), dtype=np.float64)

      lane_steer = _lane_center_steer(lead_state.vehicle, getattr(lead_state.vehicle, "lane", None))
      if lane_steer is None:
        lane_steer = _ego_lane_fallback_steer(lead_state.vehicle, env.vehicle, lead_cfg.distance_m)

      if lane_steer is not None:
        lead_action[0] = LEAD_STEER_BLEND_IDM * float(lead_action[0]) + LEAD_STEER_BLEND_LANE * lane_steer

      lead_action[0] = float(np.clip(lead_action[0], -1.0, 1.0))

    lead_state.vehicle.before_step(lead_action.tolist())
  except Exception as e:
    print(f"[WARNING] Lead update failed, clearing lead vehicle: {e}")
    _clear_lead_vehicle(env, lead_state)


def _update_lead_measurement(env: MetaDriveEnv, lead_state: LeadState, step_dt: float):
  """Update relative lead state (dRel/yRel/vRel/aRel) from current sim state."""
  if lead_state.vehicle is None:
    lead_state.prev_v_rel = 0.0
    lead_state.measurement = _lead_measurement(False)
    return

  d_rel_m, y_rel_m = _compute_rel(env.vehicle.position, env.vehicle.heading_theta, lead_state.vehicle.position)
  if d_rel_m <= 0.5:
    lead_state.prev_v_rel = 0.0
    lead_state.measurement = _lead_measurement(False)
    return

  v_rel_mps = _planar_speed(lead_state.vehicle.velocity) - _planar_speed(env.vehicle.velocity)
  delta_t_s = max(step_dt, 1e-3)
  a_rel_mps2 = (v_rel_mps - lead_state.prev_v_rel) / delta_t_s
  lead_state.prev_v_rel = v_rel_mps
  lead_state.measurement = _lead_measurement(True, d_rel_m, y_rel_m, v_rel_mps, a_rel_mps2)


def _capture_rgb_image(env: MetaDriveEnv, sensor_name: str):
  """Capture an RGB frame from a named MetaDrive camera sensor."""
  camera_sensor = env.engine.sensors[sensor_name]
  camera_sensor.get_cam().reparentTo(env.vehicle.origin)
  camera_sensor.get_cam().setPos(C3_POSITION)
  camera_sensor.get_cam().setHpr(C3_HPR)
  captured_frame = camera_sensor.perceive(to_float=False)
  return captured_frame if isinstance(captured_frame, np.ndarray) else captured_frame.get()


def _send_running_state(simulation_state_send: Connection):
  """Publish a 'running' simulation lifecycle message to the bridge."""
  simulation_state_send.send(metadrive_simulation_state(running=True, done=False, done_info=None))


def _send_done_state(simulation_state_send: Connection, done_result):
  """Publish a terminal simulation lifecycle message to the bridge."""
  simulation_state_send.send(metadrive_simulation_state(running=False, done=done_result[0], done_info=done_result[1]))


def _profile_is_complete(lead_cfg: LeadConfig, lead_state: LeadState, now_monotonic_s: float) -> bool:
  """Return True when replay time exceeds the final sample timestamp in profile CSV."""
  if lead_state.profile_t is None or lead_state.start_time is None:
    return False

  replay_elapsed_s = now_monotonic_s - lead_state.start_time - lead_cfg.start_delay_s
  return replay_elapsed_s >= float(lead_state.profile_t[-1])


@dataclass
class HCCCParityTrace:
  lead_speed_mps: float = 0.0
  lead_accel_mps2: float = 0.0
  feedforward_mps2: float = 0.0
  raw_command_mps2: float = 0.0
  command_error_mps2: float = 0.0


class HCCCParityLogger:
  """Reconstruct a BeamNG-style HC3 trace from the replay inputs for comparison."""
  def __init__(self, dt: float = HCCC_PARITY_DT_S, beta: float = HCCC_PARITY_BETA):
    self.dt = dt
    self.beta = beta
    self.reset()

  def reset(self):
    self._prev_lead_speed = None
    self._feedforward = 0.0
    self._held_command = 0.0
    self._steps_until_update = 0
    self.trace = HCCCParityTrace()

  def update(self, ego_speed_mps: float, lead_speed_mps: float, active: bool, actual_hccc_cmd: float) -> HCCCParityTrace:
    if not active:
      self.reset()
      return self.trace

    if self._steps_until_update <= 0:
      if self._prev_lead_speed is not None:
        lead_accel_mps2 = (lead_speed_mps - self._prev_lead_speed) / self.dt
      else:
        lead_accel_mps2 = 0.0
      self._prev_lead_speed = lead_speed_mps

      self._feedforward = self._feedforward + self.dt * ((1.0 - self.beta) * lead_accel_mps2 - self._feedforward)
      self._held_command = self.beta * (lead_speed_mps - ego_speed_mps) + self._feedforward
      self._steps_until_update = max(1, int(round(self.dt / 0.02))) - 1
    else:
      self._steps_until_update -= 1
      lead_accel_mps2 = self.trace.lead_accel_mps2

    self.trace = HCCCParityTrace(
      lead_speed_mps=float(lead_speed_mps),
      lead_accel_mps2=float(lead_accel_mps2),
      feedforward_mps2=float(self._feedforward),
      raw_command_mps2=float(self._held_command),
      command_error_mps2=float(actual_hccc_cmd - self._held_command),
    )
    return self.trace


OUTPUT_CSV_COLUMNS = [
  "time[s]",
  "position_ego[m]",
  "position_pre[m]",
  "speed_ego[m/s]",
  "speed_pre[m/s]",
  "target_speed_pre[m/s]",
  "acceleration_ego_sim[m/s2]",
  "acceleration_ego_carstate[m/s2]",
  "lead_d_rel[m]",
  "lead_v_rel[m/s]",
  "lead_a_rel[m/s2]",
  "headway[m]",
  "deltav[m/s]",
  "hccc_input_v_ego[m/s]",
  "hccc_input_radar_v_rel[m/s]",
  "hccc_input_radar_d_rel[m]",
  "hccc_input_lead_is_radar",
  "hccc_input_lead_track_id",
  "hccc_input_lead_speed_est[m/s]",
  "hccc_input_sim_track_status",
  "hccc_input_sim_track_d_rel[m]",
  "hccc_input_sim_track_v_rel[m/s]",
  "hccc_input_live_tracks_seq",
  "hccc_input_live_tracks_point_count",
  "hccc_input_live_tracks_d_rel[m]",
  "hccc_input_live_tracks_v_rel[m/s]",
  "hccc_reference_lead_speed[m/s]",
  "hccc_reference_lead_accel[m/s2]",
  "hccc_reference_feedforward[m/s2]",
  "hccc_reference_cmd[m/s2]",
  "hccc_reference_error[m/s2]",
  "ego_lane_id",
  "ego_on_lane",
  "ego_lane_s[m]",
  "ego_lane_lateral[m]",
  "ego_lane_heading_error_deg",
  "lead_lane_id",
  "lead_on_lane",
  "lead_lane_lateral[m]",
  "lead_pose_replay_applied",
  "lead_pose_fallback_active",
  "planner_a_target[m/s2]",
  "hccc_accel[m/s2]",
  "manual_accel[m/s2]",
  "final_accel_cmd[m/s2]",
  "hccc_active",
  "driver_gas",
  "driver_brake",
  "engine_throttle",
  "engine_brake",
  "controller_throttle",
  "controller_brake",
]


def _get_next_test_number(directory: Path) -> int:
  """Find the next TestN index for BeamNG-style output naming."""
  test_numbers: list[int] = []
  pattern = re.compile(r"Test(\d+)\.vehicle\..*\.csv$")
  if directory.exists():
    for filename in directory.iterdir():
      match = pattern.match(filename.name)
      if match:
        test_numbers.append(int(match.group(1)))
  return (max(test_numbers) + 1) if test_numbers else 1


def _default_output_graph_path(output_csv_path: str) -> str:
  """Create a default PNG path beside the CSV output."""
  csv_path = Path(output_csv_path).expanduser()
  return str(csv_path.with_suffix(".png"))


def _default_output_paths(lead_cfg: LeadConfig) -> tuple[str, str]:
  """Build BeamNG-style default CSV and graph output paths."""
  control_method = (lead_cfg.output_control_method or "default").lower()
  vehicle_name = lead_cfg.output_vehicle_name or "vehicle"

  # Keep replay artifacts inside the repo by default so each run is easy to
  # inspect, diff, and commit alongside the code that produced it.
  sim_root = Path(__file__).resolve().parents[2]
  data_dir = sim_root / "data" / control_method
  graph_dir = sim_root / "graphs" / control_method
  data_dir.mkdir(parents=True, exist_ok=True)
  graph_dir.mkdir(parents=True, exist_ok=True)

  test_number = _get_next_test_number(data_dir)
  scenario_tag = f".scn{lead_cfg.profile_scn}" if lead_cfg.profile_scn is not None else ""
  stem = f"Test{test_number}.vehicle.{vehicle_name}_ICE{scenario_tag}.{control_method}"
  return str(data_dir / f"{stem}.csv"), str(graph_dir / f"{stem}.png")


def _open_output_csv(path: str):
  """Create output CSV writer and write the header matching requested schema."""
  output_path = Path(path).expanduser()
  output_path.parent.mkdir(parents=True, exist_ok=True)
  output_file = output_path.open("w", newline="")
  output_writer = csv.writer(output_file)
  output_writer.writerow(OUTPUT_CSV_COLUMNS)
  return output_file, output_writer, str(output_path)


def _write_output_graph(csv_path: str, graph_path: str):
  """Render a speed-vs-time graph from the bridge telemetry CSV."""
  try:
    import matplotlib.pyplot as plt
  except ImportError as exc:
    raise RuntimeError("matplotlib is required to generate graph output") from exc

  times_s: list[float] = []
  speed_ego_mps: list[float] = []
  speed_pre_mps: list[float] = []

  with Path(csv_path).expanduser().open("r", newline="") as csv_file:
    reader = csv.DictReader(csv_file)
    for row in reader:
      try:
        times_s.append(float(row["time[s]"]))
        speed_ego_mps.append(float(row["speed_ego[m/s]"]))
        speed_pre_mps.append(float(row["speed_pre[m/s]"]))
      except (KeyError, TypeError, ValueError):
        continue

  if not times_s:
    raise RuntimeError(f"No plottable telemetry rows found in {csv_path}")

  graph_output_path = Path(graph_path).expanduser()
  graph_output_path.parent.mkdir(parents=True, exist_ok=True)

  plt.figure(figsize=(12, 6))
  plt.plot(times_s, speed_pre_mps, label="Preceding Vehicle")
  plt.plot(times_s, speed_ego_mps, label="Ego Vehicle")
  plt.xlabel("Time [s]")
  plt.ylabel("Speed [m/s]")
  plt.title("Speed vs Time")
  plt.legend()
  plt.grid(True)
  plt.tight_layout()
  plt.savefig(graph_output_path)
  plt.show()
  plt.close()


# Main worker entrypoint running in the MetaDrive subprocess.
def metadrive_process(
  dual_camera: bool,
  config: dict,
  camera_array,
  wide_camera_array,
  image_lock,
  controls_recv: Connection,
  simulation_state_send: Connection,
  vehicle_state_send: Connection,
  exit_event,
  op_engaged,
  test_duration,
  test_run,
):
  """Run the MetaDrive simulation worker loop and exchange data with the bridge."""
  # Pull bridge-specific knobs out of the world config dictionary.
  arrive_dest_done = bool(config.pop("arrive_dest_done", True))
  lead_cfg = LeadConfig(
    enabled=bool(config.pop("lead_vehicle_enabled", False)),
    distance_m=float(config.pop("lead_vehicle_distance", 35.0)),
    start_delay_s=float(config.pop("lead_start_delay_s", 0.0)),
    lateral_offset_m=float(config.pop("lead_vehicle_lateral_offset", 0.0)),
    model=config.pop("lead_vehicle_model", "s"),
    render=bool(config.pop("lead_vehicle_render", True)),
    profile_scn=config.pop("lead_profile_scn", None),
    profile_csv=config.pop("lead_profile_csv", None),
    output_csv=config.pop("lead_profile_output_csv", None),
    output_graph=config.pop("lead_profile_output_graph", None),
    output_control_method=config.pop("lead_profile_output_control_method", None),
    output_vehicle_name=config.pop("lead_profile_output_vehicle_name", None),
  )

  steer_command_ratio = float(config.pop("steer_cmd_ratio", 1.2))
  sim_step_interval_frames = max(1, int(config.pop("sim_step_frames", 5)))
  camera_capture_frames = max(1, int(config.pop("camera_capture_frames", 5)))
  sim_step_dt_s = float(config.get("physics_world_step_size", 0.05)) * float(config.get("decision_repeat", 1))

  _patch_metadrive(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  wide_road_image = None
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)
  lead_state = LeadState()
  if lead_cfg.profile_scn is not None:
    if lead_cfg.profile_csv is None:
      raise RuntimeError("lead_profile_scn is set but lead_profile_csv is missing")
    lead_state.profile_t, lead_state.profile_s, lead_state.profile_v = _load_lead_speed_profile(
      lead_cfg.profile_csv, int(lead_cfg.profile_scn)
    )
    print(
      f"[INFO] Loaded lead trajectory speed profile for scenario {lead_cfg.profile_scn} "
      f"from {lead_cfg.profile_csv} ({len(lead_state.profile_t)} samples)"
    )

  def reset_world():
    """Reset env and lead-vehicle state while keeping bridge worker alive."""
    _clear_lead_vehicle(env, lead_state)
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_state.measurement = _lead_measurement(False)
    lead_state.prev_v_rel = 0.0
    _spawn_lead_vehicle(env, lead_cfg, lead_state)
    _send_running_state(simulation_state_send)

  reset_world()

  ratekeeper = Ratekeeper(100, None)
  ego_control = [0.0, 0.0]
  engage_start_time = None
  bridge_start_time_s = time.monotonic()
  prev_ego_speed_mps = _planar_speed(env.vehicle.velocity)

  if lead_cfg.output_csv is None:
    output_csv_path, default_graph_path = _default_output_paths(lead_cfg)
  else:
    output_csv_path = lead_cfg.output_csv
    default_graph_path = _default_output_graph_path(output_csv_path)
  output_file, output_writer, output_csv_path = _open_output_csv(output_csv_path)
  output_graph_path = lead_cfg.output_graph or default_graph_path
  print(f"[INFO] Writing bridge telemetry CSV to {output_csv_path}")
  print(f"[INFO] Writing bridge telemetry graph to {output_graph_path}")
  latest_bridge_telemetry: dict[str, float | bool] = {}
  hccc_parity_logger = HCCCParityLogger()

  try:
    while not exit_event.is_set():
      lead_measurement = lead_state.measurement
      ego_debug_state = _ego_debug_state(env.vehicle)
      vehicle_state_send.send(
        metadrive_vehicle_state(
          velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
          position=env.vehicle.position,
          bearing=float(math.degrees(env.vehicle.heading_theta)),
          steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING,
          lead_status=lead_measurement["status"],
          lead_d_rel=lead_measurement["d_rel"],
          lead_y_rel=lead_measurement["y_rel"],
          lead_v_rel=lead_measurement["v_rel"],
          lead_a_rel=lead_measurement["a_rel"],
          lead_vehicle_valid=lead_state.vehicle is not None,
          lead_vehicle_velocity=vec3(
            x=float(lead_state.vehicle.velocity[0]),
            y=float(lead_state.vehicle.velocity[1]),
            z=0.0,
          ) if lead_state.vehicle is not None else vec3(x=0.0, y=0.0, z=0.0),
          lead_vehicle_bearing=float(math.degrees(lead_state.vehicle.heading_theta)) if lead_state.vehicle is not None else 0.0,
          lead_vehicle_steering_angle=float(lead_state.vehicle.steering * lead_state.vehicle.MAX_STEERING) if lead_state.vehicle is not None else 0.0,
          debug_has_lane=ego_debug_state["has_lane"],
          debug_on_lane=ego_debug_state["on_lane"],
          debug_lane_s=ego_debug_state["lane_s"],
          debug_lane_lateral=ego_debug_state["lane_lateral"],
          debug_lane_heading_error_deg=ego_debug_state["lane_heading_error_deg"],
          debug_on_yellow_line=ego_debug_state["on_yellow_line"],
          debug_on_white_line=ego_debug_state["on_white_line"],
          debug_crash_sidewalk=ego_debug_state["crash_sidewalk"],
          debug_out_of_route=ego_debug_state["out_of_route"],
        )
      )

      should_reset = False
      if controls_recv.poll(0):
        while controls_recv.poll(0):
          payload = controls_recv.recv()
          if len(payload) == 4:
            steer_angle, gas, should_reset, latest_bridge_telemetry = payload
          else:
            steer_angle, gas, should_reset = payload
            latest_bridge_telemetry = {}

        steer_limit = float(env.vehicle.MAX_STEERING) * max(steer_command_ratio, 1e-3)
        steer_cmd = float(np.interp(steer_angle, [-steer_limit, steer_limit], [-1.0, 1.0]))
        ego_control = [float(np.clip(steer_cmd, -1.0, 1.0)), gas]

      if should_reset:
        reset_world()
        engage_start_time = None
        hccc_parity_logger.reset()

      if op_engaged.is_set() and engage_start_time is None:
        engage_start_time = time.monotonic()

      if ratekeeper.frame % sim_step_interval_frames == 0:
        _update_lead_vehicle(env, lead_cfg, lead_state)
        pre_step_debug = ego_debug_state
        pre_step_position_xy = np.array(env.vehicle.position, dtype=np.float64)[:2]
        pre_step_heading = float(env.vehicle.heading_theta)
        _, _, terminated, _, _ = env.step(ego_control)
        _update_lead_measurement(env, lead_state, sim_step_dt_s)

        ego_speed_mps = _planar_speed(env.vehicle.velocity)
        ego_accel_mps2 = (ego_speed_mps - prev_ego_speed_mps) / max(sim_step_dt_s, 1e-3)
        prev_ego_speed_mps = ego_speed_mps
        lead_speed_mps = _planar_speed(lead_state.vehicle.velocity) if lead_state.vehicle is not None else 0.0
        target_speed_pre_mps = float(lead_state.target_speed_mps) if lead_state.target_speed_mps is not None else ""
        lead_d_rel = float(lead_measurement["d_rel"]) if lead_measurement["status"] else ""
        lead_v_rel = float(lead_measurement["v_rel"]) if lead_measurement["status"] else ""
        lead_a_rel = float(lead_measurement["a_rel"]) if lead_measurement["status"] else ""
        headway_m = (float(lead_measurement["d_rel"]) - 4.5) if lead_measurement["status"] else ""
        deltav_mps = float(ego_speed_mps - lead_speed_mps) if lead_state.vehicle is not None else ""

        engine_long_cmd = float(getattr(env.vehicle, "throttle_brake", ego_control[1]))
        engine_throttle = max(engine_long_cmd, 0.0)
        engine_brake = max(-engine_long_cmd, 0.0)
        controller_throttle = max(float(ego_control[1]), 0.0)
        controller_brake = max(-float(ego_control[1]), 0.0)
        hccc_actual_cmd = float(latest_bridge_telemetry.get("hccc_accel", 0.0))
        hccc_parity_trace = hccc_parity_logger.update(
          float(latest_bridge_telemetry.get("carstate_v_ego", 0.0)),
          float(latest_bridge_telemetry.get("radar_lead_speed_est", 0.0)),
          bool(latest_bridge_telemetry.get("hccc_active", False)),
          hccc_actual_cmd,
        )

        ego_position = np.array(env.vehicle.position, dtype=np.float64)
        ego_position_xyz = [
          float(ego_position[0]),
          float(ego_position[1]),
          float(ego_position[2]) if ego_position.size > 2 else 0.0,
        ]
        if lead_state.vehicle is not None:
          lead_position = np.array(lead_state.vehicle.position, dtype=np.float64)
          lead_position_xyz = [
            float(lead_position[0]),
            float(lead_position[1]),
            float(lead_position[2]) if lead_position.size > 2 else 0.0,
          ]
        else:
          lead_position_xyz = ["", "", ""]
        lead_debug_state = _lead_debug_state(lead_state.vehicle) if lead_state.vehicle is not None else {
          "lane_id": "",
          "on_lane": False,
          "lane_lateral": "",
        }

        elapsed_time_s = round(time.monotonic() - bridge_start_time_s, 6)
        output_writer.writerow([
          elapsed_time_s,
          ego_position_xyz,
          lead_position_xyz,
          float(ego_speed_mps),
          float(lead_speed_mps),
          target_speed_pre_mps,
          float(ego_accel_mps2),
          float(latest_bridge_telemetry.get("carstate_a_ego", 0.0)),
          lead_d_rel,
          lead_v_rel,
          lead_a_rel,
          headway_m,
          deltav_mps,
          float(latest_bridge_telemetry.get("carstate_v_ego", 0.0)),
          float(latest_bridge_telemetry.get("radar_lead_v_rel", 0.0)),
          float(latest_bridge_telemetry.get("radar_lead_d_rel", 0.0)),
          bool(latest_bridge_telemetry.get("radar_lead_is_radar", False)),
          int(latest_bridge_telemetry.get("radar_lead_track_id", -1)),
          float(latest_bridge_telemetry.get("radar_lead_speed_est", 0.0)),
          bool(latest_bridge_telemetry.get("sim_track_status", False)),
          float(latest_bridge_telemetry.get("sim_track_d_rel", 0.0)),
          float(latest_bridge_telemetry.get("sim_track_v_rel", 0.0)),
          int(latest_bridge_telemetry.get("live_tracks_seq", 0)),
          int(latest_bridge_telemetry.get("live_tracks_point_count", 0)),
          float(latest_bridge_telemetry.get("live_tracks_d_rel", 0.0)),
          float(latest_bridge_telemetry.get("live_tracks_v_rel", 0.0)),
          float(hccc_parity_trace.lead_speed_mps),
          float(hccc_parity_trace.lead_accel_mps2),
          float(hccc_parity_trace.feedforward_mps2),
          float(hccc_parity_trace.raw_command_mps2),
          float(hccc_parity_trace.command_error_mps2),
          ego_debug_state["lane_id"],
          bool(ego_debug_state["on_lane"]),
          float(ego_debug_state["lane_s"]) if ego_debug_state["has_lane"] else "",
          float(ego_debug_state["lane_lateral"]) if ego_debug_state["has_lane"] else "",
          float(ego_debug_state["lane_heading_error_deg"]) if ego_debug_state["has_lane"] else "",
          lead_debug_state["lane_id"],
          lead_debug_state["on_lane"],
          lead_debug_state["lane_lateral"],
          bool(lead_state.pose_replay_applied),
          bool(lead_state.pose_replay_fallback_active),
          float(latest_bridge_telemetry.get("planner_a_target", 0.0)),
          float(latest_bridge_telemetry.get("hccc_accel", 0.0)),
          float(latest_bridge_telemetry.get("manual_accel", 0.0)),
          float(latest_bridge_telemetry.get("final_accel", 0.0)),
          bool(latest_bridge_telemetry.get("hccc_active", False)),
          float(latest_bridge_telemetry.get("driver_gas", 0.0)),
          float(latest_bridge_telemetry.get("driver_brake", 0.0)),
          float(engine_throttle),
          float(engine_brake),
          float(controller_throttle),
          float(controller_brake),
        ])

        now_s = time.monotonic()
        if _profile_is_complete(lead_cfg, lead_state, now_s):
          done_result = (
            True,
            {
              "csv_profile_complete": True,
              "profile_scn": lead_cfg.profile_scn,
              "output_csv": output_csv_path,
              "output_graph": output_graph_path,
            },
          )
          _send_done_state(simulation_state_send, done_result)
          break

        timeout = engage_start_time is not None and (now_s - engage_start_time) >= test_duration
        if terminated or (timeout and test_run):
          done_result = env.done_function("default_agent") if terminated else (True, {"timeout": True})

          if terminated and bool(done_result[1].get("out_of_road", False)):
            post_step_debug = _ego_debug_state(env.vehicle)
            post_step_position_xy = np.array(env.vehicle.position, dtype=np.float64)[:2]
            post_step_heading = float(env.vehicle.heading_theta)
            print("[DEBUG] MetaDrive out_of_road termination details:")
            print(f"[DEBUG] done_info={done_result[1]}")
            print(_format_debug_state("  preStep", pre_step_debug, pre_step_position_xy, pre_step_heading))
            print(_format_debug_state("  postStep", post_step_debug, post_step_position_xy, post_step_heading))
            print(f"[DEBUG] laneChanged={pre_step_debug['lane_id'] != post_step_debug['lane_id']}")
            print("[WARNING] Episode hit out_of_road. Auto-resetting scenario instead of exiting.")
            reset_world()
            engage_start_time = None
            continue

          _send_done_state(simulation_state_send, done_result)

      if ratekeeper.frame % camera_capture_frames == 0:
        if dual_camera and wide_road_image is not None:
          wide_road_image[...] = _capture_rgb_image(env, "rgb_wide")
        road_image[...] = _capture_rgb_image(env, "rgb_road")
        image_lock.release()

      ratekeeper.keep_time()
  finally:
    output_file.flush()
    output_file.close()
    try:
      _write_output_graph(output_csv_path, output_graph_path)
      print(f"[INFO] Saved bridge telemetry graph to {output_graph_path}")
    except Exception as exc:
      print(f"[WARNING] Failed to generate bridge telemetry graph: {exc}")
