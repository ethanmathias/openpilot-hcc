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
from metadrive.component.vehicle.vehicle_type import vehicle_type, DefaultVehicle
from metadrive.policy.expert_policy import ExpertPolicy

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle"])

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
  # Backward-compatible fixed speed option for old config callers.
  lead_vehicle_speed = float(config.pop("lead_vehicle_speed", lead_speed_end_mph * mph_to_ms))
  lead_vehicle_lateral_offset = float(config.pop("lead_vehicle_lateral_offset", 0.0))
  lead_vehicle_model = config.pop("lead_vehicle_model", "s")
  lead_vehicle_render = bool(config.pop("lead_vehicle_render", True))
  lead_vehicle_expert_policy = bool(config.pop("lead_vehicle_expert_policy", config.pop("lead_vehicle_idm_policy", True)))
  lead_vehicle_expert_min_speed_mph = float(config.pop("lead_vehicle_expert_min_speed_mph",
                                                        config.pop("lead_vehicle_idm_min_speed_mph", 8.0)))
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
  lead_speed_start = lead_speed_start_mph * mph_to_ms
  lead_speed_end = lead_speed_end_mph * mph_to_ms
  expert_min_speed = max(0.0, lead_vehicle_expert_min_speed_mph * mph_to_ms)
  expert_assist_printed = False

  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

  def _vehicle_cls_for_model(model_name, fallback_cls):
    cls = vehicle_type.get(model_name)
    return cls if cls is not None else fallback_cls

  def _candidate_models(primary_model, ego_model):
    # These models are part of standard MetaDrive model ids.
    ordered = [primary_model, ego_model, "s", "m", "l", "xl", "default"]
    unique_models = []
    for model in ordered:
      if model and model not in unique_models:
        unique_models.append(model)
    return unique_models

  def _target_lead_speed():
    nonlocal lead_profile_start_time
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
    nonlocal lead_vehicle, lead_vehicle_expert_policy
    lead_vehicle = None

    if not lead_vehicle_enabled:
      return

    ego_vehicle = env.vehicle
    heading, target_position = _lead_target_position(ego_vehicle)
    ego_model = ego_vehicle.config.get("vehicle_model", None)

    if lead_vehicle_expert_policy:
      expert_config = dict(env.config["vehicle_config"])
      expert_config["render_vehicle"] = lead_vehicle_render
      expert_config["spawn_velocity"] = heading.tolist()
      expert_config["spawn_velocity_car_frame"] = False
      for config_key in ("show_navi_mark", "show_dest_mark", "show_line_to_dest", "show_line_to_navi_mark"):
        if config_key in expert_config:
          expert_config[config_key] = False
      try:
        lead_vehicle = env.engine.spawn_object(
          DefaultVehicle,
          position=target_position.tolist(),
          heading=float(ego_vehicle.heading_theta),
          random_seed=env.engine.generate_seed(),
          vehicle_config=expert_config,
        )
        lead_vehicle.set_velocity(heading, _target_lead_speed(), in_local_frame=False)
        env.engine.add_policy(lead_vehicle.id, ExpertPolicy, lead_vehicle, env.engine.generate_seed())
        print("[INFO] Lead vehicle is controlled by ExpertPolicy")
        print("[INFO] Spawned lead vehicle model 'DefaultVehicle'")
        return
      except Exception as e:
        print(f"[WARNING] Expert lead spawn/policy attach failed: {e}. Falling back to manual lead control.")
        lead_vehicle = None
        lead_vehicle_expert_policy = False

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
        lead_vehicle.set_velocity(heading, _target_lead_speed(), in_local_frame=False)
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
    nonlocal lead_vehicle, lead_distance_dynamic, expert_assist_printed
    if lead_vehicle is None:
      return
    if lead_vehicle_expert_policy:
      try:
        current_lead_speed = float(np.linalg.norm(np.array(lead_vehicle.velocity[:2], dtype=np.float64)))
        target_speed = max(_target_lead_speed(), expert_min_speed)
        if current_lead_speed < max(0.5, expert_min_speed):
          lead_heading = np.array([math.cos(float(lead_vehicle.heading_theta)), math.sin(float(lead_vehicle.heading_theta))], dtype=np.float64)
          lead_vehicle.set_velocity(lead_heading, target_speed, in_local_frame=False)
          if not expert_assist_printed:
            print("[INFO] Expert policy keep-moving assist is active for lead vehicle")
            expert_assist_printed = True
        else:
          expert_assist_printed = False
      except Exception as e:
        print(f"[WARNING] Expert keep-moving assist failed: {e}")
      return

    ego_vehicle = env.vehicle
    current_lead_speed = _target_lead_speed()
    ego_speed = float(np.linalg.norm(np.array(ego_vehicle.velocity[:2], dtype=np.float64)))
    lead_distance_dynamic += (current_lead_speed - ego_speed) * step_dt
    lead_distance_dynamic = float(np.clip(lead_distance_dynamic, 8.0, 120.0))

    heading, target_position = _lead_target_position(ego_vehicle)
    try:
      lead_vehicle.set_position(target_position.tolist())
      lead_vehicle.set_velocity(heading, current_lead_speed, in_local_frame=False)
    except Exception as e:
      print(f"[WARNING] Lead update failed, removing lead vehicle: {e}")
      lead_vehicle = None

  def reset():
    nonlocal lead_distance_dynamic, lead_profile_start_time
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_distance_dynamic = lead_vehicle_distance
    lead_profile_start_time = None
    spawn_lead_vehicle()
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

  steer_ratio = 8
  vc = [0,0]

  while not exit_event.is_set():
    vehicle_state = metadrive_vehicle_state(
      velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
      position=env.vehicle.position,
      bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_metadrive = steer_angle * 1 / (env.vehicle.MAX_STEERING * steer_ratio)
      steer_metadrive = np.clip(steer_metadrive, -1, 1)

      vc = [steer_metadrive, gas]

      if should_reset:
        lane_idx_prev = reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      update_lead_vehicle()
      _, _, terminated, _, _ = env.step(vc)
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
