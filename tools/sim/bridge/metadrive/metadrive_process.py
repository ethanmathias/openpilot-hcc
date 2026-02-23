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
from metadrive.component.vehicle.vehicle_type import vehicle_type

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)


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
  mph_to_ms = 0.44704
  arrive_dest_done = config.pop("arrive_dest_done", True)
  lead_vehicle_enabled = bool(config.pop("lead_vehicle_enabled", False))
  lead_vehicle_distance = float(config.pop("lead_vehicle_distance", 35.0))
  lead_speed_profile = str(config.pop("lead_speed_profile", "constant"))
  lead_speed_start_mph = float(config.pop("lead_speed_start_mph", 10.0))
  lead_speed_end_mph = float(config.pop("lead_speed_end_mph", 30.0))
  lead_speed_ramp_sec = float(config.pop("lead_speed_ramp_sec", 20.0))
  lead_start_delay_s = float(config.pop("lead_start_delay_s", 0.0))
  lead_vehicle_speed = float(config.pop("lead_vehicle_speed", lead_speed_end_mph * mph_to_ms))
  lead_vehicle_lateral_offset = float(config.pop("lead_vehicle_lateral_offset", 0.0))
  lead_vehicle_model = config.pop("lead_vehicle_model", "s")
  lead_vehicle_render = bool(config.pop("lead_vehicle_render", True))
  steer_cmd_ratio = float(config.pop("steer_cmd_ratio", 15.0))
  steer_cmd_rate_limit = float(config.pop("steer_cmd_rate_limit", 0.08))
  steer_cmd_deadband_deg = float(config.pop("steer_cmd_deadband_deg", 0.0))
  step_dt = float(config.get("physics_world_step_size", 0.05)) * float(config.get("decision_repeat", 1))
  apply_metadrive_patches(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)
  lead_vehicle = None
  lead_distance_dynamic = lead_vehicle_distance
  lead_profile_start_time = None
  lead_start_time = None
  lead_speed_start = lead_speed_start_mph * mph_to_ms
  lead_speed_end = lead_speed_end_mph * mph_to_ms
  lead_world_heading = None
  lead_world_position = None
  lead_measurement = _get_lead_measurement(False)
  lead_prev_v_rel = 0.0

  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

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

  def _target_lead_speed():
    nonlocal lead_profile_start_time, lead_start_time
    if lead_start_delay_s > 0.0:
      now = time.monotonic()
      if lead_start_time is None:
        lead_start_time = now
      if now - lead_start_time < lead_start_delay_s:
        return 0.0

    if lead_speed_profile == "loop_ramp":
      if not op_engaged.is_set():
        return lead_speed_start

      now = time.monotonic()
      if lead_profile_start_time is None:
        lead_profile_start_time = now

      ramp_sec = max(lead_speed_ramp_sec, 1e-3)
      phase = ((now - lead_profile_start_time) / ramp_sec) % 2.0
      alpha = phase if phase <= 1.0 else (2.0 - phase)
      return lead_speed_start + alpha * (lead_speed_end - lead_speed_start)

    if lead_speed_profile != "ramp":
      return lead_vehicle_speed

    if not op_engaged.is_set():
      return lead_speed_start

    now = time.monotonic()
    if lead_profile_start_time is None:
      lead_profile_start_time = now

    ramp_sec = max(lead_speed_ramp_sec, 1e-3)
    alpha = float(np.clip((now - lead_profile_start_time) / ramp_sec, 0.0, 1.0))
    return lead_speed_start + alpha * (lead_speed_end - lead_speed_start)

  def _lead_target_position(ego_vehicle):
    heading = np.array([math.cos(ego_vehicle.heading_theta), math.sin(ego_vehicle.heading_theta)], dtype=np.float64)
    target_position = np.array(ego_vehicle.position, dtype=np.float64) + heading * lead_distance_dynamic
    if abs(lead_vehicle_lateral_offset) > 1e-3:
      lateral_direction = np.array([-heading[1], heading[0]], dtype=np.float64)
      target_position += lateral_direction * lead_vehicle_lateral_offset
    return heading, target_position

  def spawn_lead_vehicle():
    nonlocal lead_vehicle, lead_world_heading, lead_world_position
    lead_vehicle = None

    if not lead_vehicle_enabled:
      return

    ego_vehicle = env.vehicle
    heading, target_position = _lead_target_position(ego_vehicle)
    ego_model = ego_vehicle.config.get("vehicle_model", None)

    lead_config_base = dict(env.config["vehicle_config"])
    lead_config_base["render_vehicle"] = lead_vehicle_render
    for config_key in ("show_navi_mark", "show_dest_mark", "show_line_to_dest", "show_line_to_navi_mark"):
      if config_key in lead_config_base:
        lead_config_base[config_key] = False
    if "navigation_module" in lead_config_base:
      lead_config_base["navigation_module"] = None
    if "navigation" in lead_config_base:
      lead_config_base["navigation"] = None

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
          heading=float(ego_vehicle.heading_theta),
        )
        heading_norm = np.linalg.norm(heading)
        if heading_norm > 1e-6:
          lead_world_heading = heading / heading_norm
        else:
          lead_world_heading = np.array([1.0, 0.0], dtype=np.float64)
        lead_world_position = np.array(target_position, dtype=np.float64)
        lead_vehicle.set_velocity(lead_world_heading, _target_lead_speed(), in_local_frame=False)
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
    nonlocal lead_vehicle, lead_world_heading, lead_world_position
    if lead_vehicle is None:
      return

    current_lead_speed = _target_lead_speed()
    if lead_world_heading is None:
      lead_heading_theta = float(lead_vehicle.heading_theta)
      lead_world_heading = np.array([math.cos(lead_heading_theta), math.sin(lead_heading_theta)], dtype=np.float64)
    if lead_world_position is None:
      lead_world_position = np.array(lead_vehicle.position, dtype=np.float64)

    lead_world_position = lead_world_position + lead_world_heading * current_lead_speed * step_dt
    try:
      lead_vehicle.set_position(lead_world_position.tolist())
      lead_vehicle.set_velocity(lead_world_heading, current_lead_speed, in_local_frame=False)
    except Exception as e:
      print(f"[WARNING] Lead update failed, removing lead vehicle: {e}")
      lead_vehicle = None

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
    nonlocal lead_distance_dynamic, lead_profile_start_time, lead_world_heading, lead_world_position, lead_start_time
    nonlocal lead_measurement, lead_prev_v_rel
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_distance_dynamic = lead_vehicle_distance
    lead_profile_start_time = None
    lead_start_time = None
    lead_world_heading = None
    lead_world_position = None
    spawn_lead_vehicle()
    lead_measurement = _get_lead_measurement(False)
    lead_prev_v_rel = 0.0
    lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

    return lane_idx_prev

  lane_idx_prev = reset()
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

  steer_target = 0.0
  steer_cmd = 0.0
  gas_cmd = 0.0

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

      if abs(steer_angle) <= steer_cmd_deadband_deg:
        steer_angle = 0.0

      steer_metadrive = steer_angle / (env.vehicle.MAX_STEERING * steer_cmd_ratio)
      steer_target = float(np.clip(steer_metadrive, -1.0, 1.0))
      gas_cmd = float(gas)

      if should_reset:
        lane_idx_prev = reset()
        start_time = None
        steer_target = 0.0
        steer_cmd = 0.0
        gas_cmd = 0.0

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      if steer_cmd_rate_limit <= 0.0:
        steer_cmd = steer_target
      else:
        steer_delta = float(np.clip(steer_target - steer_cmd, -steer_cmd_rate_limit, steer_cmd_rate_limit))
        steer_cmd += steer_delta

      vc = [steer_cmd, gas_cmd]
      update_lead_vehicle()
      _, _, terminated, _, _ = env.step(vc)
      update_lead_measurement()
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
      out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
      lane_idx_prev = lane_idx_curr

      if terminated or ((out_of_lane or timeout) and test_run):
        if terminated:
          done_result = env.done_function("default_agent")
        elif out_of_lane:
          done_result = (True, {"out_of_lane" : True})
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
