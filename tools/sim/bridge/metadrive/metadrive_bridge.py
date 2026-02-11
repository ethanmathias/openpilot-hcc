import math
from multiprocessing import Queue

from metadrive.component.sensors.base_camera import _cuda_enable
from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.metadrive.metadrive_world import MetaDriveWorld
from openpilot.tools.sim.lib.camerad import W, H


def straight_block(length):
  return {
    "id": "S",
    "pre_block_socket_index": 0,
    "length": length
  }

def curve_block(length, angle=45, direction=0):
  return {
    "id": "C",
    "pre_block_socket_index": 0,
    "length": length,
    "radius": length,
    "angle": angle,
    "dir": direction
  }

def create_map(track_size=60):
  curve_len = track_size * 2
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=1,
    lane_width=4.5,
    config=[
      None,
      straight_block(track_size),
      curve_block(curve_len, 90),
      straight_block(track_size),
      curve_block(curve_len, 90),
      straight_block(track_size),
      curve_block(curve_len, 90),
      straight_block(track_size),
      curve_block(curve_len, 90),
    ]
  )


def get_hccc_step_scenario():
  mph_to_ms = 0.44704
  v0 = 0.0
  v10 = 5.0 * mph_to_ms
  v30 = 10.0 * mph_to_ms
  return {
    "enabled": True,
    "visual_lead": True,
    "visual_lead_overlay": True,
    "lead_start_delay_s": 10.0,
    "initial_d_rel": 8.0,
    "initial_v_lead": v0,
    "y_rel": 0.0,
    # Keep transitions smooth and slower so hCCC response is easier to observe.
    "max_accel": 0.5,
    "max_decel": 1.2,
    # (time_seconds, target_speed_mps)
    "speed_profile": [
      # hold stopped
      (0.0, v0),
      (8.0, v0),
      # ramp to 10 mph
      (20.0, v10),
      # hold 10 mph
      (28.0, v10),
      # ramp to 30 mph
      (40.0, v30),
      # hold 30 mph
      (48.0, v30),
      # ramp to 10 mph
      (60.0, v10),
      # hold 10 mph
      (68.0, v10),
      # ramp back to 30 mph
      (80.0, v30),
      # hold 30 mph
      (88.0, v30),
    ],
  }


class MetaDriveBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False, scenario="default",
               force_engage_on_startup=True):
    super().__init__(dual_camera, high_quality, force_engage_on_startup=force_engage_on_startup)

    # Keep CI/tests headless, but render in normal interactive runs.
    self.should_render = not test_run
    self.test_run = test_run
    self.test_duration = test_duration if self.test_run else math.inf
    self.scenario = scenario

    if self.scenario == "hccc_step":
      self.params.put_bool("EnableHCCC", True)
    elif not force_engage_on_startup:
      self.params.put_bool("EnableHCCC", False)

  def spawn_world(self, queue: Queue):
    sensors = {
      "rgb_road": (RGBCameraRoad, W, H, )
    }

    if self.dual_camera:
      sensors["rgb_wide"] = (RGBCameraWide, W, H)

    # Use a very long single-lane loop so hCCC tests have long uninterrupted stretches.
    map_track_size = 2000 if self.scenario == "hccc_step" else 800
    hccc_scenario = get_hccc_step_scenario() if self.scenario == "hccc_step" else {"enabled": False}

    config = dict(
      use_render=self.should_render,
      vehicle_config=dict(
        enable_reverse=False,
        # Some MetaDrive installs miss optional vehicle model assets (e.g. ferra wheel glTF),
        # so keep mesh rendering disabled for stability.
        render_vehicle=False,
        image_source="rgb_road",
      ),
      sensors=sensors,
      image_on_cuda=_cuda_enable,
      image_observation=True,
      interface_panel=[],
      out_of_route_done=False,
      on_continuous_line_done=False,
      crash_vehicle_done=False,
      crash_object_done=False,
      arrive_dest_done=False,
      traffic_density=0.0, # traffic is incredibly expensive
      map_config=create_map(track_size=map_track_size),
      decision_repeat=1,
      physics_world_step_size=self.TICKS_PER_FRAME/100,
      preload_models=False,
      show_logo=False,
      anisotropic_filtering=False,
      hccc_scenario=hccc_scenario,
    )

    return MetaDriveWorld(queue, config, self.test_duration, self.test_run, self.dual_camera)
