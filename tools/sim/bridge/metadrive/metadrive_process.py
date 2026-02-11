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
  return {
    "enabled": enabled,
    "d_rel": float(hccc_scenario.get("initial_d_rel", 0.0)),
    "y_rel": float(hccc_scenario.get("y_rel", 0.0)),
    "v_lead": float(hccc_scenario.get("initial_v_lead", 0.0)),
    "prev_v_rel": 0.0,
    "last_step_mono": None,
    "start_time": time.monotonic(),
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

  dt = max(dt, 1e-3)
  profile = hccc_scenario.get("speed_profile", [])
  target_v_lead = _get_profile_target_speed(profile, time.monotonic() - lead_state["start_time"])

  max_accel = float(hccc_scenario.get("max_accel", 2.0))
  max_decel = float(hccc_scenario.get("max_decel", 3.0))
  dv = np.clip(target_v_lead - lead_state["v_lead"], -max_decel * dt, max_accel * dt)
  lead_state["v_lead"] = max(0.0, lead_state["v_lead"] + float(dv))

  v_rel = lead_state["v_lead"] - ego_speed
  a_rel = (v_rel - lead_state["prev_v_rel"]) / dt
  lead_state["prev_v_rel"] = v_rel

  lead_state["d_rel"] = max(2.0, lead_state["d_rel"] + v_rel * dt)

  return _get_lead_measurement(True, lead_state["d_rel"], lead_state["y_rel"], v_rel, a_rel)

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
  hccc_scenario = config.pop("hccc_scenario", {"enabled": False})
  apply_metadrive_patches(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)

  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

  def reset():
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

    lead_state = _create_hccc_lead_state(hccc_scenario)
    lead_measurement = _get_lead_measurement(False)

    return lane_idx_prev, lead_state, lead_measurement

  lane_idx_prev, lead_state, lead_measurement = reset()
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
        lane_idx_prev, lead_state, lead_measurement = reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      _, _, terminated, _, _ = env.step(vc)
      step_mono = time.monotonic()
      if lead_state["last_step_mono"] is None:
        lead_dt = 0.05
      else:
        lead_dt = step_mono - lead_state["last_step_mono"]
      lead_state["last_step_mono"] = step_mono
      ego_speed = float(np.linalg.norm([env.vehicle.velocity[0], env.vehicle.velocity[1]]))
      lead_measurement = _update_virtual_hccc_lead(hccc_scenario, lead_state, ego_speed, lead_dt)

      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
      out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
      lane_idx_prev = lane_idx_curr

      if terminated:
        done_result = env.done_function("default_agent")
        if test_run:
          simulation_state = metadrive_simulation_state(
            running=False,
            done=done_result[0],
            done_info=done_result[1],
          )
          simulation_state_send.send(simulation_state)
        else:
          # In interactive usage, reset instead of tearing down the full bridge.
          lane_idx_prev, lead_state, lead_measurement = reset()
          start_time = None
          continue
      elif (out_of_lane or timeout) and test_run:
        if out_of_lane:
          done_result = (True, {"out_of_lane" : True})
        else:
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
