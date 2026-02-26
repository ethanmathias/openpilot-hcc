import math
import time
import numpy as np

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

from metadrive.engine.core.engine_core import EngineCore
from metadrive.engine.core.image_buffer import ImageBuffer
from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.obs.image_obs import ImageObservation
from metadrive.policy.idm_policy import IDMPolicy
from metadrive.component.vehicle.vehicle_type import vehicle_type

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0, 0)


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle",
                                                                 "lead_status", "lead_d_rel", "lead_y_rel", "lead_v_rel", "lead_a_rel"])

def _get_lead_measurement(enabled=False, d_rel=0.0, y_rel=0.0, v_rel=0.0, a_rel=0.0):
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
  d_rel = float(np.dot(delta, forward))
  y_rel = float(np.dot(delta, left))
  return d_rel, y_rel


def _wrap_to_pi(angle: float) -> float:
  return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _lane_center_steer_from_lane(vehicle, lane) -> float | None:
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


def apply_metadrive_patches(arrive_dest_done=True):
  # By default, metadrive won't try to use cuda images unless it's used as a sensor for vehicles, so patch that in
  def add_image_sensor_patched(self, name: str, cls, args):
    if self.global_config["image_on_cuda"]:# and name == self.global_config["vehicle_config"]["image_source"]:
        sensor = cls(*args, self, cuda=True)
    else:
        sensor = cls(*args, self, cuda=False)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  # we aren't going to use the built-in observation stack, so disable it to save time
  def observe_patched(self, *args, **kwargs):
    return self.state

  ImageObservation.observe = observe_patched

  # disable destination, we want to loop forever
  def arrive_destination_patch(self, *args, **kwargs):
    return False

  if not arrive_dest_done:
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch

def metadrive_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)
  lead_vehicle_enabled = bool(config.pop("lead_vehicle_enabled", False))
  lead_vehicle_distance = float(config.pop("lead_vehicle_distance", 35.0))
  # Consume legacy speed knobs so older configs still work with IDM lead.
  for legacy_key in ("lead_speed_profile", "lead_speed_start_mph", "lead_speed_end_mph", "lead_speed_ramp_sec", "lead_vehicle_speed"):
    config.pop(legacy_key, None)
  lead_start_delay_s = float(config.pop("lead_start_delay_s", 0.0))
  lead_vehicle_lateral_offset = float(config.pop("lead_vehicle_lateral_offset", 0.0))
  lead_vehicle_model = config.pop("lead_vehicle_model", "s")
  lead_vehicle_render = bool(config.pop("lead_vehicle_render", True))
  steer_cmd_ratio = float(config.pop("steer_cmd_ratio", 12.0))
  step_dt = float(config.get("physics_world_step_size", 0.05)) * float(config.get("decision_repeat", 1))
  apply_metadrive_patches(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)
  lead_vehicle = None
  lead_policy = None
  lead_start_time = None
  lead_measurement = _get_lead_measurement(False)
  lead_prev_v_rel = 0.0

  def _vehicle_cls_for_model(model_name, fallback_cls):
    cls = vehicle_type.get(model_name)
    return cls if cls is not None else fallback_cls

  def _candidate_models(primary_model, ego_model):
    ordered = [primary_model, ego_model, "s", "m", "l", "xl", "default"]
    unique_models = []
    for model in ordered:
      if model and model not in unique_models:
        unique_models.append(model)
    return unique_models

  def _lead_target_pose(ego_vehicle):
    fallback_heading = float(ego_vehicle.heading_theta)
    heading_vec = np.array([math.cos(fallback_heading), math.sin(fallback_heading)], dtype=np.float64)
    fallback_position = np.array(ego_vehicle.position, dtype=np.float64) + heading_vec * lead_vehicle_distance
    if abs(lead_vehicle_lateral_offset) > 1e-3:
      lateral_direction = np.array([-heading_vec[1], heading_vec[0]], dtype=np.float64)
      fallback_position += lateral_direction * lead_vehicle_lateral_offset

    lane = getattr(ego_vehicle, "lane", None)
    if lane is None:
      return fallback_position, fallback_heading

    try:
      ego_longitudinal, _ = lane.local_coordinates(ego_vehicle.position)
      target_longitudinal = float(ego_longitudinal + lead_vehicle_distance)
      lane_target = np.array(lane.position(target_longitudinal, lead_vehicle_lateral_offset), dtype=np.float64)
      target_position = lane_target[:2]

      # Use lane tangent to align heading with the lane centerline.
      p0 = np.array(lane.position(target_longitudinal, lead_vehicle_lateral_offset), dtype=np.float64)[:2]
      p1 = np.array(lane.position(target_longitudinal + 0.5, lead_vehicle_lateral_offset), dtype=np.float64)[:2]
      tangent = p1 - p0
      if np.linalg.norm(tangent) > 1e-6:
        target_heading = float(math.atan2(tangent[1], tangent[0]))
      else:
        target_heading = fallback_heading
      return target_position, target_heading
    except Exception:
      return fallback_position, fallback_heading

  def spawn_lead_vehicle():
    nonlocal lead_vehicle, lead_policy, lead_start_time
    lead_vehicle = None
    lead_policy = None

    if not lead_vehicle_enabled:
      return

    ego_vehicle = env.vehicle
    target_position, target_heading = _lead_target_pose(ego_vehicle)
    ego_model = ego_vehicle.config.get("vehicle_model", None)

    lead_config_base = dict(env.config["vehicle_config"])
    lead_config_base["render_vehicle"] = lead_vehicle_render
    for config_key in ("show_navi_mark", "show_dest_mark", "show_line_to_dest", "show_line_to_navi_mark"):
      if config_key in lead_config_base:
        lead_config_base[config_key] = False

    fallback_cls = ego_vehicle.__class__
    spawn_error = None
    for model_name in _candidate_models(lead_vehicle_model, ego_model):
      lead_config = dict(lead_config_base)
      lead_config["vehicle_model"] = model_name
      vehicle_cls = _vehicle_cls_for_model(model_name, fallback_cls)

      try:
        lead_vehicle = env.engine.spawn_object(
          vehicle_cls,
          vehicle_config=lead_config,
          position=target_position.tolist(),
          heading=target_heading,
        )
        policy_seed = int((time.time() * 1000) % (2**31 - 1))
        lead_policy = IDMPolicy(lead_vehicle, policy_seed)
        lead_start_time = time.monotonic()
        print(f"[INFO] Spawned lead vehicle model '{model_name}'")
        return
      except (OSError, FileNotFoundError) as e:
        spawn_error = e
        print(f"[WARNING] Failed lead model '{model_name}': {e}")
      except Exception as e:
        spawn_error = e
        print(f"[WARNING] Lead spawn error for '{model_name}': {e}")

    print(f"[WARNING] Lead vehicle disabled after model spawn failures: {spawn_error}")

  def update_lead_vehicle():
    nonlocal lead_vehicle, lead_policy
    if lead_vehicle is None or lead_policy is None:
      return

    try:
      waiting_for_start = (
        lead_start_delay_s > 0.0 and
        lead_start_time is not None and
        (time.monotonic() - lead_start_time) < lead_start_delay_s
      )
      if waiting_for_start:
        lead_action = [0.0, 0.0]
      else:
        lead_action = lead_policy.act()
        lead_action = np.array(lead_action, dtype=np.float64)
        # Add lane-centering correction so lead follows curved lane geometry.
        lane_steer = _lane_center_steer_from_lane(lead_vehicle, getattr(lead_vehicle, "lane", None))
        if lane_steer is None:
          # Fallback: project to ego lane and steer toward a point ahead on that centerline.
          try:
            ego_lane = getattr(env.vehicle, "lane", None)
            if ego_lane is not None:
              ego_s, _ = ego_lane.local_coordinates(env.vehicle.position)
              target_s = float(ego_s + lead_vehicle_distance + 6.0)
              target_xy = np.array(ego_lane.position(target_s, 0.0), dtype=np.float64)[:2]
              lead_xy = np.array(lead_vehicle.position, dtype=np.float64)[:2]
              vec = target_xy - lead_xy
              if np.linalg.norm(vec) > 1e-6:
                desired_heading = float(math.atan2(vec[1], vec[0]))
                heading_error = _wrap_to_pi(desired_heading - float(lead_vehicle.heading_theta))
                lane_steer = float(np.clip(1.8 * heading_error, -1.0, 1.0))
          except Exception:
            lane_steer = None

        if lane_steer is not None:
          # Blend IDM steer with lane-follow steer, biased toward lane tracking.
          lead_action[0] = 0.2 * float(lead_action[0]) + 0.8 * lane_steer

        # Keep steering command in simulator bounds.
        lead_action[0] = float(np.clip(lead_action[0], -1.0, 1.0))
        lead_action = lead_action.tolist()
      lead_vehicle.before_step(lead_action)
    except Exception as e:
      print(f"[WARNING] Lead update failed, removing lead vehicle: {e}")
      lead_vehicle = None
      lead_policy = None

  def update_lead_measurement():
    nonlocal lead_measurement, lead_prev_v_rel
    if lead_vehicle is None:
      lead_prev_v_rel = 0.0
      lead_measurement = _get_lead_measurement(False)
      return

    ego_position = env.vehicle.position
    lead_position = lead_vehicle.position
    d_rel, y_rel = _compute_rel(ego_position, env.vehicle.heading_theta, lead_position)
    if d_rel <= 0.5:
      lead_prev_v_rel = 0.0
      lead_measurement = _get_lead_measurement(False)
      return

    ego_speed = float(np.linalg.norm([env.vehicle.velocity[0], env.vehicle.velocity[1]]))
    lead_speed = float(np.linalg.norm([lead_vehicle.velocity[0], lead_vehicle.velocity[1]]))
    v_rel = lead_speed - ego_speed
    dt = max(step_dt, 1e-3)
    a_rel = (v_rel - lead_prev_v_rel) / dt
    lead_prev_v_rel = v_rel
    lead_measurement = _get_lead_measurement(True, d_rel, y_rel, v_rel, a_rel)

  def reset():
    nonlocal lead_start_time
    nonlocal lead_measurement, lead_prev_v_rel
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_start_time = None
    spawn_lead_vehicle()
    lead_measurement = _get_lead_measurement(False)
    lead_prev_v_rel = 0.0

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)
  reset()
  start_time = None

  def get_cam_as_rgb(cam):
    cam = env.engine.sensors[cam]
    cam.get_cam().reparentTo(env.vehicle.origin)
    cam.get_cam().setPos(C3_POSITION)
    cam.get_cam().setHpr(C3_HPR)
    img = cam.perceive(to_float=False)
    if not isinstance(img, np.ndarray):
      img = img.get() # convert cupy array to numpy
    return img

  rk = Ratekeeper(100, None)

  vc = [0,0]

  while not exit_event.is_set():
    vehicle_state = metadrive_vehicle_state(
      velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
      position=env.vehicle.position,
      bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING,
      lead_status=lead_measurement["status"],
      lead_d_rel=lead_measurement["d_rel"],
      lead_y_rel=lead_measurement["y_rel"],
      lead_v_rel=lead_measurement["v_rel"],
      lead_a_rel=lead_measurement["a_rel"],
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_input_limit = float(env.vehicle.MAX_STEERING) * max(steer_cmd_ratio, 1e-3)
      steer_metadrive = float(np.interp(steer_angle, [-steer_input_limit, steer_input_limit], [-1.0, 1.0]))
      steer_metadrive = float(np.clip(steer_metadrive, -1.0, 1.0))

      vc = [steer_metadrive, gas]

      if should_reset:
        reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      update_lead_vehicle()
      _, _, terminated, _, _ = env.step(vc)
      update_lead_measurement()
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False

      if terminated or (timeout and test_run):
        if terminated:
          done_result = env.done_function("default_agent")
        elif timeout:
          done_result = (True, {"timeout" : True})

        simulation_state = metadrive_simulation_state(
          running=False,
          done=done_result[0],
          done_info=done_result[1],
        )
        simulation_state_send.send(simulation_state)

      if dual_camera:
        wide_road_image[...] = get_cam_as_rgb("rgb_wide")
      road_image[...] = get_cam_as_rgb("rgb_road")
      image_lock.release()

    rk.keep_time()
