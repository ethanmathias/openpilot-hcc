import csv
import math
import time
from collections import namedtuple
from dataclasses import dataclass, field
from multiprocessing.connection import Connection

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

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0, 0)

LEGACY_LEAD_KEYS = (
  "lead_speed_profile",
  "lead_speed_start_mph",
  "lead_speed_end_mph",
  "lead_speed_ramp_sec",
  "lead_vehicle_speed",
)

LEAD_STEER_BLEND_IDM = 0.2
LEAD_STEER_BLEND_LANE = 0.8
EGO_LANE_LOOKAHEAD_M = 6.0

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


@dataclass
class LeadConfig:
  enabled: bool
  distance_m: float
  start_delay_s: float
  lateral_offset_m: float
  model: str
  render: bool
  profile_scn: int | None
  profile_csv: str | None


@dataclass
class LeadState:
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


def _lead_measurement(enabled=False, d_rel=0.0, y_rel=0.0, v_rel=0.0, a_rel=0.0):
  return {
    "status": bool(enabled),
    "d_rel": float(d_rel),
    "y_rel": float(y_rel),
    "v_rel": float(v_rel),
    "a_rel": float(a_rel),
  }


def _compute_rel(ego_position, ego_heading, lead_position):
  c = math.cos(float(ego_heading))
  s = math.sin(float(ego_heading))
  forward = np.array([c, s], dtype=np.float64)
  left = np.array([-s, c], dtype=np.float64)
  delta = np.array(lead_position, dtype=np.float64) - np.array(ego_position, dtype=np.float64)
  return float(np.dot(delta, forward)), float(np.dot(delta, left))


def _wrap_to_pi(angle: float) -> float:
  return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _planar_speed(velocity_xy) -> float:
  return float(np.linalg.norm([velocity_xy[0], velocity_xy[1]]))


def _lane_identifier(lane) -> str:
  if lane is None:
    return "<none>"

  parts = [lane.__class__.__name__]
  for attr in ("index", "lane_index", "name"):
    value = getattr(lane, attr, None)
    if value is not None:
      parts.append(f"{attr}={value}")
  return " ".join(parts)


def _ego_debug_state(vehicle):
  debug = {
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
    return debug

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

    debug.update({
      "has_lane": True,
      "on_lane": on_lane,
      "lane_s": float(lane_s),
      "lane_lateral": float(lane_lateral),
      "lane_heading_error_deg": float(lane_heading_error_deg),
    })
  except Exception:
    pass

  return debug


def _format_debug_state(prefix: str, debug: dict, position_xy, heading_theta: float) -> str:
  pos = np.array(position_xy, dtype=np.float64)[:2]
  return (
    f"{prefix}: pos=({pos[0]:.2f}, {pos[1]:.2f}) "
    f"bearing={math.degrees(float(heading_theta)):.2f} deg "
    f"laneId={debug['lane_id']} hasLane={debug['has_lane']} onLane={debug['on_lane']} "
    f"s={debug['lane_s']:.2f} latOff={debug['lane_lateral']:.2f} "
    f"hdgErr={debug['lane_heading_error_deg']:.2f} deg "
    f"yellow={debug['on_yellow_line']} white={debug['on_white_line']} "
    f"sidewalk={debug['crash_sidewalk']} outOfRoute={debug['out_of_route']}"
  )


def _load_lead_speed_profile(csv_path: str, scenario_index: int):
  if scenario_index < 1:
    raise ValueError(f"Scenario index must be >= 1, got {scenario_index}")

  t_samples: list[float] = []
  v_samples: list[float] = []
  with open(csv_path, newline="") as f:
    reader = csv.reader(f)
    for row in reader:
      if not row or not row[0].strip():
        continue

      try:
        t_val = float(row[0].strip())
      except ValueError:
        continue

      if scenario_index >= len(row):
        if v_samples:
          break
        continue

      v_text = row[scenario_index].strip()
      if not v_text:
        if v_samples:
          break
        continue

      try:
        v_val = max(float(v_text), 0.0)
      except ValueError:
        if v_samples:
          break
        continue

      t_samples.append(t_val)
      v_samples.append(v_val)

  if len(t_samples) < 2:
    raise RuntimeError(f"Scenario {scenario_index} has insufficient samples in {csv_path}")

  t_arr = np.asarray(t_samples, dtype=np.float64)
  v_arr = np.asarray(v_samples, dtype=np.float64)
  order = np.argsort(t_arr)
  t_arr = t_arr[order]
  v_arr = v_arr[order]

  uniq_t, uniq_idx = np.unique(t_arr, return_index=True)
  uniq_v = v_arr[uniq_idx]
  if uniq_t.size < 2:
    raise RuntimeError(f"Scenario {scenario_index} needs at least two unique time samples in {csv_path}")

  # Integrate speed profile into cumulative longitudinal distance (meters).
  uniq_s = np.zeros_like(uniq_v)
  for i in range(1, uniq_t.size):
    dt = max(float(uniq_t[i] - uniq_t[i - 1]), 0.0)
    uniq_s[i] = uniq_s[i - 1] + 0.5 * (uniq_v[i - 1] + uniq_v[i]) * dt

  return uniq_t, uniq_s, uniq_v


def _profile_interp_position(lead_state: LeadState, elapsed_s: float) -> float:
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
  x = float(position_xy[0])
  y = float(position_xy[1])
  pos_ok = False

  if hasattr(vehicle, "set_position"):
    try:
      vehicle.set_position([x, y])
      pos_ok = True
    except Exception:
      pass
  if not pos_ok and hasattr(vehicle, "set_pos"):
    try:
      vehicle.set_pos([x, y])
      pos_ok = True
    except Exception:
      pass
  if not pos_ok:
    try:
      vehicle.position = [x, y]
      pos_ok = True
    except Exception:
      pass

  heading_ok = False
  for setter in ("set_heading_theta", "set_heading"):
    if hasattr(vehicle, setter):
      try:
        getattr(vehicle, setter)(float(heading_theta))
        heading_ok = True
        break
      except Exception:
        pass
  if not heading_ok:
    try:
      vehicle.heading_theta = float(heading_theta)
      heading_ok = True
    except Exception:
      pass

  vx = float(math.cos(heading_theta) * speed_mps)
  vy = float(math.sin(heading_theta) * speed_mps)
  for setter in ("set_velocity",):
    if hasattr(vehicle, setter):
      try:
        getattr(vehicle, setter)([vx, vy])
        break
      except Exception:
        pass

  return pos_ok and heading_ok


def _patch_metadrive(arrive_dest_done: bool):
  def add_image_sensor_patched(self, name: str, cls, args):
    use_cuda = bool(self.global_config.get("image_on_cuda", False))
    sensor = cls(*args, self, cuda=use_cuda)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  def observe_patched(self, *args, **kwargs):
    return self.state

  ImageObservation.observe = observe_patched

  if not arrive_dest_done:
    def arrive_destination_patch(self, *args, **kwargs):
      return False
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch


def _vehicle_cls_for_model(model_name, fallback_cls):
  cls = vehicle_type.get(model_name)
  return cls if cls is not None else fallback_cls


def _candidate_models(primary_model, ego_model):
  ordered = [primary_model, ego_model, "s", "m", "l", "xl", "default"]
  return [model for i, model in enumerate(ordered) if model and model not in ordered[:i]]


def _lane_center_steer(vehicle, lane) -> float | None:
  if lane is None:
    return None

  try:
    s_coord, lateral_offset = lane.local_coordinates(vehicle.position)
    p0 = np.array(lane.position(float(s_coord), 0.0), dtype=np.float64)[:2]
    p1 = np.array(lane.position(float(s_coord) + 1.0, 0.0), dtype=np.float64)[:2]
    tangent = p1 - p0
    if np.linalg.norm(tangent) <= 1e-6:
      return None

    lane_heading = float(math.atan2(tangent[1], tangent[0]))
    heading_error = _wrap_to_pi(lane_heading - float(vehicle.heading_theta))
    return float(np.clip(1.2 * heading_error - 0.18 * float(lateral_offset), -1.0, 1.0))
  except Exception:
    return None


def _ego_lane_fallback_steer(lead_vehicle, ego_vehicle, lead_distance_m: float) -> float | None:
  try:
    ego_lane = getattr(ego_vehicle, "lane", None)
    if ego_lane is None:
      return None

    ego_s, _ = ego_lane.local_coordinates(ego_vehicle.position)
    target_s = float(ego_s + lead_distance_m + EGO_LANE_LOOKAHEAD_M)
    target_xy = np.array(ego_lane.position(target_s, 0.0), dtype=np.float64)[:2]
    lead_xy = np.array(lead_vehicle.position, dtype=np.float64)[:2]
    vec = target_xy - lead_xy
    if np.linalg.norm(vec) <= 1e-6:
      return None

    desired_heading = float(math.atan2(vec[1], vec[0]))
    heading_error = _wrap_to_pi(desired_heading - float(lead_vehicle.heading_theta))
    return float(np.clip(1.8 * heading_error, -1.0, 1.0))
  except Exception:
    return None


def _lead_spawn_pose(ego_vehicle, lead_cfg: LeadConfig):
  fallback_heading = float(ego_vehicle.heading_theta)
  heading_vec = np.array([math.cos(fallback_heading), math.sin(fallback_heading)], dtype=np.float64)
  fallback_position = np.array(ego_vehicle.position, dtype=np.float64) + heading_vec * lead_cfg.distance_m
  if abs(lead_cfg.lateral_offset_m) > 1e-3:
    lateral_direction = np.array([-heading_vec[1], heading_vec[0]], dtype=np.float64)
    fallback_position += lateral_direction * lead_cfg.lateral_offset_m

  lane = getattr(ego_vehicle, "lane", None)
  if lane is None:
    return fallback_position, fallback_heading

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
      target_heading = fallback_heading

    return target_position, target_heading
  except Exception:
    return fallback_position, fallback_heading


def _spawn_lead_vehicle(env: MetaDriveEnv, lead_cfg: LeadConfig, lead_state: LeadState):
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

  lead_config_base = dict(env.config.get("vehicle_config", {}))
  lead_config_base["render_vehicle"] = lead_cfg.render
  for key in ("show_navi_mark", "show_dest_mark", "show_line_to_dest", "show_line_to_navi_mark"):
    if key in lead_config_base:
      lead_config_base[key] = False

  fallback_cls = ego_vehicle.__class__
  spawn_error = None
  for model_name in _candidate_models(lead_cfg.model, ego_model):
    lead_config = dict(lead_config_base)
    lead_config["vehicle_model"] = model_name
    vehicle_cls = _vehicle_cls_for_model(model_name, fallback_cls)

    try:
      lead_state.vehicle = env.engine.spawn_object(
        vehicle_cls,
        vehicle_config=lead_config,
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
  vehicle = lead_state.vehicle
  if vehicle is not None:
    cleared = False
    clear_errors = []

    object_keys = []
    for attr in ("name", "id", "index"):
      value = getattr(vehicle, attr, None)
      if value is not None:
        object_keys.append(value)

    for key in object_keys:
      try:
        env.engine.clear_objects([key])
        cleared = True
        break
      except Exception as e:
        clear_errors.append(str(e))

    if not cleared:
      try:
        env.engine.clear_objects([vehicle])
        cleared = True
      except Exception as e:
        clear_errors.append(str(e))

    if not cleared and hasattr(vehicle, "destroy"):
      try:
        vehicle.destroy()
      except Exception as e:
        clear_errors.append(str(e))

    if not cleared and clear_errors:
      print(f"[WARNING] Failed to clear lead vehicle before reset: {' | '.join(clear_errors)}")

  lead_state.vehicle = None
  lead_state.policy = None
  lead_state.start_time = None
  lead_state.profile_lane = None
  lead_state.profile_s_base = None
  lead_state.pose_replay_failed = False


def _update_lead_vehicle(env: MetaDriveEnv, lead_cfg: LeadConfig, lead_state: LeadState):
  if lead_state.vehicle is None:
    return

  try:
    elapsed = 0.0 if lead_state.start_time is None else (time.monotonic() - lead_state.start_time)
    if lead_cfg.start_delay_s > 0.0 and elapsed < lead_cfg.start_delay_s:
      lead_action = np.array([0.0, 0.0], dtype=np.float64)
    elif lead_state.profile_t is not None and lead_state.profile_s is not None:
      if lead_state.profile_lane is None:
        lane_for_profile = getattr(lead_state.vehicle, "lane", None) or getattr(env.vehicle, "lane", None)
        if lane_for_profile is not None:
          lead_s, _ = lane_for_profile.local_coordinates(lead_state.vehicle.position)
          lead_state.profile_lane = lane_for_profile
          lead_state.profile_s_base = float(lead_s - lead_state.profile_s[0])

      if lead_state.profile_lane is not None and lead_state.profile_s_base is not None:
        s_rel = _profile_interp_position(lead_state, elapsed - lead_cfg.start_delay_s)
        v_target = _profile_interp_speed(lead_state, elapsed - lead_cfg.start_delay_s)
        target_s = float(lead_state.profile_s_base + s_rel)

        p0 = np.array(lead_state.profile_lane.position(target_s, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
        p1 = np.array(lead_state.profile_lane.position(target_s + 0.5, lead_cfg.lateral_offset_m), dtype=np.float64)[:2]
        tangent = p1 - p0
        if np.linalg.norm(tangent) > 1e-6:
          heading = float(math.atan2(tangent[1], tangent[0]))
        else:
          heading = float(lead_state.vehicle.heading_theta)

        if not _set_vehicle_pose_2d(lead_state.vehicle, p0, heading, max(v_target, 0.0)):
          if not lead_state.pose_replay_failed:
            print("[WARNING] Lead pose replay failed; falling back to controller tracking.")
            lead_state.pose_replay_failed = True
        else:
          lead_state.pose_replay_failed = False
          lead_action = np.array([0.0, 0.0], dtype=np.float64)
          lead_state.vehicle.before_step(lead_action.tolist())
          return

      v_target = _profile_interp_speed(lead_state, elapsed - lead_cfg.start_delay_s)
      v_now = _planar_speed(lead_state.vehicle.velocity)
      lon_cmd = float(np.clip((v_target - v_now) * 0.35, -1.0, 1.0))
      lane_steer = _lane_center_steer(lead_state.vehicle, getattr(lead_state.vehicle, "lane", None))
      if lane_steer is None:
        lane_steer = _ego_lane_fallback_steer(lead_state.vehicle, env.vehicle, lead_cfg.distance_m)
      steer_cmd = 0.0 if lane_steer is None else float(np.clip(lane_steer, -1.0, 1.0))
      lead_action = np.array([steer_cmd, lon_cmd], dtype=np.float64)
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
  if lead_state.vehicle is None:
    lead_state.prev_v_rel = 0.0
    lead_state.measurement = _lead_measurement(False)
    return

  d_rel, y_rel = _compute_rel(env.vehicle.position, env.vehicle.heading_theta, lead_state.vehicle.position)
  if d_rel <= 0.5:
    lead_state.prev_v_rel = 0.0
    lead_state.measurement = _lead_measurement(False)
    return

  v_rel = _planar_speed(lead_state.vehicle.velocity) - _planar_speed(env.vehicle.velocity)
  dt = max(step_dt, 1e-3)
  a_rel = (v_rel - lead_state.prev_v_rel) / dt
  lead_state.prev_v_rel = v_rel
  lead_state.measurement = _lead_measurement(True, d_rel, y_rel, v_rel, a_rel)


def _capture_rgb_image(env: MetaDriveEnv, sensor_name: str):
  cam = env.engine.sensors[sensor_name]
  cam.get_cam().reparentTo(env.vehicle.origin)
  cam.get_cam().setPos(C3_POSITION)
  cam.get_cam().setHpr(C3_HPR)
  image = cam.perceive(to_float=False)
  return image if isinstance(image, np.ndarray) else image.get()


def _send_running_state(simulation_state_send: Connection):
  simulation_state_send.send(metadrive_simulation_state(running=True, done=False, done_info=None))


def _send_done_state(simulation_state_send: Connection, done_result):
  simulation_state_send.send(metadrive_simulation_state(running=False, done=done_result[0], done_info=done_result[1]))


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
  )
  for legacy_key in LEGACY_LEAD_KEYS:
    config.pop(legacy_key, None)

  steer_cmd_ratio = float(config.pop("steer_cmd_ratio", 1.2))
  sim_step_frames = max(1, int(config.pop("sim_step_frames", 5)))
  camera_capture_frames = max(1, int(config.pop("camera_capture_frames", 5)))
  step_dt = float(config.get("physics_world_step_size", 0.05)) * float(config.get("decision_repeat", 1))

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
    _clear_lead_vehicle(env, lead_state)
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_state.measurement = _lead_measurement(False)
    lead_state.prev_v_rel = 0.0
    _spawn_lead_vehicle(env, lead_cfg, lead_state)
    _send_running_state(simulation_state_send)

  reset_world()

  rk = Ratekeeper(100, None)
  ego_control = [0.0, 0.0]
  engage_start_time = None

  while not exit_event.is_set():
    measurement = lead_state.measurement
    debug = _ego_debug_state(env.vehicle)
    vehicle_state_send.send(
      metadrive_vehicle_state(
        velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
        position=env.vehicle.position,
        bearing=float(math.degrees(env.vehicle.heading_theta)),
        steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING,
        lead_status=measurement["status"],
        lead_d_rel=measurement["d_rel"],
        lead_y_rel=measurement["y_rel"],
        lead_v_rel=measurement["v_rel"],
        lead_a_rel=measurement["a_rel"],
        debug_has_lane=debug["has_lane"],
        debug_on_lane=debug["on_lane"],
        debug_lane_s=debug["lane_s"],
        debug_lane_lateral=debug["lane_lateral"],
        debug_lane_heading_error_deg=debug["lane_heading_error_deg"],
        debug_on_yellow_line=debug["on_yellow_line"],
        debug_on_white_line=debug["on_white_line"],
        debug_crash_sidewalk=debug["crash_sidewalk"],
        debug_out_of_route=debug["out_of_route"],
      )
    )

    should_reset = False
    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_limit = float(env.vehicle.MAX_STEERING) * max(steer_cmd_ratio, 1e-3)
      steer_cmd = float(np.interp(steer_angle, [-steer_limit, steer_limit], [-1.0, 1.0]))
      ego_control = [float(np.clip(steer_cmd, -1.0, 1.0)), gas]

    if should_reset:
      reset_world()
      engage_start_time = None

    if op_engaged.is_set() and engage_start_time is None:
      engage_start_time = time.monotonic()

    if rk.frame % sim_step_frames == 0:
      _update_lead_vehicle(env, lead_cfg, lead_state)
      pre_debug = debug
      pre_position = np.array(env.vehicle.position, dtype=np.float64)[:2]
      pre_heading = float(env.vehicle.heading_theta)
      _, _, terminated, _, _ = env.step(ego_control)
      _update_lead_measurement(env, lead_state, step_dt)

      timeout = engage_start_time is not None and (time.monotonic() - engage_start_time) >= test_duration
      if terminated or (timeout and test_run):
        done_result = env.done_function("default_agent") if terminated else (True, {"timeout": True})

        if terminated and bool(done_result[1].get("out_of_road", False)):
          post_debug = _ego_debug_state(env.vehicle)
          post_position = np.array(env.vehicle.position, dtype=np.float64)[:2]
          post_heading = float(env.vehicle.heading_theta)
          print("[DEBUG] MetaDrive out_of_road termination details:")
          print(f"[DEBUG] done_info={done_result[1]}")
          print(_format_debug_state("  preStep", pre_debug, pre_position, pre_heading))
          print(_format_debug_state("  postStep", post_debug, post_position, post_heading))
          print(f"[DEBUG] laneChanged={pre_debug['lane_id'] != post_debug['lane_id']}")
          print("[WARNING] Episode hit out_of_road. Auto-resetting scenario instead of exiting.")
          reset_world()
          engage_start_time = None
          continue

        _send_done_state(simulation_state_send, done_result)

    if rk.frame % camera_capture_frames == 0:
      if dual_camera and wide_road_image is not None:
        wide_road_image[...] = _capture_rgb_image(env, "rgb_wide")
      road_image[...] = _capture_rgb_image(env, "rgb_road")
      image_lock.release()

    rk.keep_time()
