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
try:
  from metadrive.component.vehicle.vehicle_type import DefaultVehicle
except Exception:
  DefaultVehicle = None
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

def _create_hccc_lead_state(hccc_scenario):
  enabled = bool(hccc_scenario.get("enabled", False))
  now = time.monotonic()
  lead_start_delay_s = float(hccc_scenario.get("lead_start_delay_s", 0.0))
  return {
    "enabled": enabled,
    "lead_start_delay_s": lead_start_delay_s,
    "d_rel": float(hccc_scenario.get("initial_d_rel", 0.0)),
    "y_rel": float(hccc_scenario.get("y_rel", 0.0)),
    "v_lead": float(hccc_scenario.get("initial_v_lead", 0.0)),
    "prev_v_rel": 0.0,
    "last_step_mono": None,
    "start_time": now,
    "profile_start_time": now + lead_start_delay_s,
    "started": lead_start_delay_s <= 0.0,
  }

def _get_profile_target_speed(profile, elapsed_s):
  if len(profile) == 0:
    return 0.0

  times = [float(p[0]) for p in profile]
  speeds = [float(p[1]) for p in profile]
  return float(np.interp(elapsed_s, times, speeds))

def _update_virtual_hccc_lead(hccc_scenario, lead_state, ego_speed, dt):
  if not lead_state["enabled"]:
    return _get_lead_measurement(False)

  now = time.monotonic()
  if not lead_state["started"]:
    if now < lead_state["profile_start_time"]:
      lead_state["prev_v_rel"] = 0.0
      return _get_lead_measurement(False)
    lead_state["started"] = True
    lead_state["profile_start_time"] = now
    lead_state["prev_v_rel"] = 0.0

  dt = max(dt, 1e-3)
  profile = hccc_scenario.get("speed_profile", [])
  target_v_lead = _get_profile_target_speed(profile, now - lead_state["profile_start_time"])

  max_accel = float(hccc_scenario.get("max_accel", 2.0))
  max_decel = float(hccc_scenario.get("max_decel", 3.0))
  dv = np.clip(target_v_lead - lead_state["v_lead"], -max_decel * dt, max_accel * dt)
  lead_state["v_lead"] = max(0.0, lead_state["v_lead"] + float(dv))

  v_rel = lead_state["v_lead"] - ego_speed
  a_rel = (v_rel - lead_state["prev_v_rel"]) / dt
  lead_state["prev_v_rel"] = v_rel

  lead_state["d_rel"] = max(2.0, lead_state["d_rel"] + v_rel * dt)

  return _get_lead_measurement(True, lead_state["d_rel"], lead_state["y_rel"], v_rel, a_rel)

def _compute_lead_position(ego_position, ego_heading, d_rel, y_rel):
  c = math.cos(float(ego_heading))
  s = math.sin(float(ego_heading))
  x = float(ego_position[0]) + c * float(d_rel) - s * float(y_rel)
  y = float(ego_position[1]) + s * float(d_rel) + c * float(y_rel)
  return [x, y]

def _spawn_visual_hccc_lead(env, hccc_scenario, lead_state):
  if not lead_state["enabled"] or not bool(hccc_scenario.get("visual_lead", False)):
    return None
  if DefaultVehicle is None:
    print("warning: DefaultVehicle import failed; visual lead disabled")
    return None

  lead_pos = _compute_lead_position(env.vehicle.position, env.vehicle.heading_theta, lead_state["d_rel"], lead_state["y_rel"])
  lead_heading = float(env.vehicle.heading_theta)
  lead_cfg = dict(env.vehicle.config)
  lead_cfg["enable_reverse"] = False
  lead_cfg["show_navi_mark"] = False

  try:
    return env.engine.spawn_object(DefaultVehicle, vehicle_config=lead_cfg, position=lead_pos, heading=lead_heading)
  except TypeError:
    try:
      return env.engine.spawn_object(DefaultVehicle, vehicle_config=lead_cfg, position=lead_pos, heading_theta=lead_heading)
    except Exception:
      print("warning: failed to spawn visual lead vehicle")
      return None
  except Exception:
    print("warning: failed to spawn visual lead vehicle")
    return None

def _sync_visual_hccc_lead(env, visual_lead, lead_measurement):
  if visual_lead is None or not lead_measurement["status"]:
    return

  ego_heading = float(env.vehicle.heading_theta)
  ego_pos = env.vehicle.position
  lead_pos = _compute_lead_position(ego_pos, ego_heading, lead_measurement["d_rel"], lead_measurement["y_rel"])

  try:
    visual_lead.set_position(lead_pos)
  except Exception:
    return

  try:
    if hasattr(visual_lead, "set_heading_theta"):
      visual_lead.set_heading_theta(ego_heading)
    elif hasattr(visual_lead, "heading_theta"):
      visual_lead.heading_theta = ego_heading
  except Exception:
    pass

  v_lead = max(0.0, float(np.linalg.norm([env.vehicle.velocity[0], env.vehicle.velocity[1]]) + lead_measurement["v_rel"]))
  fwd = np.asarray([math.cos(ego_heading), math.sin(ego_heading)])
  try:
    visual_lead.set_velocity(fwd, v_lead)
  except TypeError:
    try:
      visual_lead.set_velocity(fwd * v_lead)
    except Exception:
      pass
  except Exception:
    pass

def _create_overlay_state():
  return {
    "initialized": False,
    "x": 0.0,
    "y": 0.0,
    "w": 0.0,
    "h": 0.0,
    "alpha": 0.0,
  }

def _blend_color(region, color, alpha):
  if alpha <= 0.0 or region.size == 0:
    return
  color_arr = np.array(color, dtype=np.float32)
  region_float = region.astype(np.float32)
  region[:] = np.clip((1.0 - alpha) * region_float + alpha * color_arr, 0, 255).astype(np.uint8)

def _draw_virtual_lead_overlay(image, lead_measurement, overlay_state):
  status = bool(lead_measurement["status"])

  h, w, _ = image.shape
  d_rel = max(2.0, float(lead_measurement["d_rel"])) if status else 2.0
  y_rel = float(lead_measurement["y_rel"]) if status else 0.0

  target_h = float(np.clip(520.0 / d_rel, 18, 120))
  target_w = float(target_h * 1.8)
  target_x = float(w * 0.5 + np.clip(y_rel * 12.0, -w * 0.35, w * 0.35))
  target_y = float(h * 0.44 + np.clip((45.0 - d_rel) * 2.0, -h * 0.12, h * 0.30))

  if status and not overlay_state["initialized"]:
    overlay_state["x"] = target_x
    overlay_state["y"] = target_y
    overlay_state["w"] = target_w
    overlay_state["h"] = target_h
    overlay_state["initialized"] = True
  elif status:
    pos_alpha = 0.22
    size_alpha = 0.18
    overlay_state["x"] += pos_alpha * (target_x - overlay_state["x"])
    overlay_state["y"] += pos_alpha * (target_y - overlay_state["y"])
    overlay_state["w"] += size_alpha * (target_w - overlay_state["w"])
    overlay_state["h"] += size_alpha * (target_h - overlay_state["h"])

  alpha_target = 1.0 if status else 0.0
  overlay_state["alpha"] += 0.25 * (alpha_target - overlay_state["alpha"])
  alpha = float(np.clip(overlay_state["alpha"], 0.0, 1.0))
  if alpha < 0.03 or not overlay_state["initialized"]:
    return

  box_w = int(max(8.0, overlay_state["w"]))
  box_h = int(max(8.0, overlay_state["h"]))
  x_center = int(overlay_state["x"])
  y_center = int(overlay_state["y"])

  x0 = max(0, x_center - box_w // 2)
  x1 = min(w, x_center + box_w // 2)
  y0 = max(0, y_center - box_h // 2)
  y1 = min(h, y_center + box_h // 2)
  if x1 <= x0 or y1 <= y0:
    return

  roi = image[y0:y1, x0:x1]
  _blend_color(roi, [230, 55, 55], 0.30 * alpha)

  border = 2
  _blend_color(image[y0:y0 + border, x0:x1], [255, 255, 255], 0.80 * alpha)
  _blend_color(image[y1 - border:y1, x0:x1], [255, 255, 255], 0.80 * alpha)
  _blend_color(image[y0:y1, x0:x0 + border], [255, 255, 255], 0.80 * alpha)
  _blend_color(image[y0:y1, x1 - border:x1], [255, 255, 255], 0.80 * alpha)

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
  hccc_scenario = config.pop("hccc_scenario", {"enabled": False})
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
  # Backward-compat: discard old policy config keys if present.
  config.pop("lead_vehicle_idm_policy", None)
  config.pop("lead_vehicle_idm_min_speed_mph", None)
  config.pop("lead_vehicle_expert_policy", None)
  config.pop("lead_vehicle_expert_min_speed_mph", None)
  step_dt = float(config.get("physics_world_step_size", 0.05)) * float(config.get("decision_repeat", 1))
  if bool(hccc_scenario.get("enabled", False)):
    lead_vehicle_enabled = False
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
  lead_world_heading = None
  lead_world_position = None
  lead_state = None
  lead_measurement = _get_lead_measurement(False)
  visual_lead = None
  overlay_state = _create_overlay_state()

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

  def reset():
    nonlocal lead_distance_dynamic, lead_profile_start_time, lead_world_heading, lead_world_position
    nonlocal lead_state, lead_measurement, visual_lead, overlay_state
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lead_distance_dynamic = lead_vehicle_distance
    lead_profile_start_time = None
    lead_world_heading = None
    lead_world_position = None
    spawn_lead_vehicle()
    lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)
    lead_state = _create_hccc_lead_state(hccc_scenario)
    lead_measurement = _get_lead_measurement(False)
    visual_lead = _spawn_visual_hccc_lead(env, hccc_scenario, lead_state)
    lead_active = bool(lead_state["enabled"] and lead_state["started"])
    lead_measurement = _get_lead_measurement(
      enabled=lead_active,
      d_rel=lead_state["d_rel"],
      y_rel=lead_state["y_rel"],
      v_rel=lead_state["v_lead"] - float(np.linalg.norm([env.vehicle.velocity[0], env.vehicle.velocity[1]])),
      a_rel=0.0,
    )
    overlay_state = _create_overlay_state()

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
      step_mono = time.monotonic()
      if lead_state is not None:
        if lead_state["last_step_mono"] is None:
          lead_dt = 0.05
        else:
          lead_dt = step_mono - lead_state["last_step_mono"]
        lead_state["last_step_mono"] = step_mono
        ego_speed = float(np.linalg.norm([env.vehicle.velocity[0], env.vehicle.velocity[1]]))
        lead_measurement = _update_virtual_hccc_lead(hccc_scenario, lead_state, ego_speed, lead_dt)
        _sync_visual_hccc_lead(env, visual_lead, lead_measurement)
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
      if bool(hccc_scenario.get("visual_lead_overlay", False)):
        _draw_virtual_lead_overlay(road_image, lead_measurement, overlay_state)
      image_lock.release()

    rk.keep_time()
