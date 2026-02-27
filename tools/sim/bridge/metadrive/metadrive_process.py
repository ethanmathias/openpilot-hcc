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
  ["velocity", "position", "bearing", "steering_angle", "lead_status", "lead_d_rel", "lead_y_rel", "lead_v_rel", "lead_a_rel"],
)


@dataclass
class LeadConfig:
  enabled: bool
  distance_m: float
  start_delay_s: float
  lateral_offset_m: float
  model: str
  render: bool


@dataclass
class LeadState:
  vehicle: object | None = None
  policy: IDMPolicy | None = None
  start_time: float | None = None
  measurement: dict[str, float | bool] = field(default_factory=lambda: _lead_measurement(False))
  prev_v_rel: float = 0.0


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
      policy_seed = int((time.time() * 1000) % (2**31 - 1))
      lead_state.policy = IDMPolicy(lead_state.vehicle, policy_seed)
      lead_state.start_time = time.monotonic()
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


def _update_lead_vehicle(env: MetaDriveEnv, lead_cfg: LeadConfig, lead_state: LeadState):
  if lead_state.vehicle is None or lead_state.policy is None:
    return

  try:
    if lead_cfg.start_delay_s > 0.0 and lead_state.start_time is not None and (time.monotonic() - lead_state.start_time) < lead_cfg.start_delay_s:
      lead_action = np.array([0.0, 0.0], dtype=np.float64)
    else:
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
      _, _, terminated, _, _ = env.step(ego_control)
      _update_lead_measurement(env, lead_state, step_dt)

      timeout = engage_start_time is not None and (time.monotonic() - engage_start_time) >= test_duration
      if terminated or (timeout and test_run):
        done_result = env.done_function("default_agent") if terminated else (True, {"timeout": True})

        if terminated and bool(done_result[1].get("out_of_road", False)):
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
