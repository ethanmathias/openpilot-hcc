import math
from multiprocessing import Queue

from metadrive.component.sensors.base_camera import _cuda_enable
from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.metadrive.metadrive_world import MetaDriveWorld
from openpilot.tools.sim.lib.camerad import W, H

SCENARIO_DEFAULT = "default"
SCENARIO_HCCC_STEP = "hccc_step"
SCENARIO_LEAD_LOOP = "lead_loop"
STRAIGHT_ROAD_SCENARIOS = {SCENARIO_LEAD_LOOP, SCENARIO_HCCC_STEP}
LEAD_SCENARIOS = {SCENARIO_LEAD_LOOP, SCENARIO_HCCC_STEP}


def straight_block(length: float):
  """Return a PGMap straight-road block configuration."""
  return {
    "id": "S",
    "pre_block_socket_index": 0,
    "length": length
  }


def curve_block(length: float, angle: float = 45, direction: int = 0):
  """Return a PGMap curve block configuration."""
  return {
    "id": "C",
    "pre_block_socket_index": 0,
    "length": length,
    "radius": length,
    "angle": angle,
    "dir": direction
  }

def create_map(track_size=60):
  """Build a closed-loop map composed of straights and 90-degree curves."""
  curve_len = track_size * 2
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=2,
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

def create_straight_map(length=10000):
  """Build a long straight map used by replay/lead-follow scenarios."""
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=2,
    lane_width=4.5,
    config=[
      None,
      straight_block(length),
    ],
  )


class MetaDriveBridge(SimulatorBridge):
  """SimulatorBridge implementation backed by MetaDrive."""
  TICKS_PER_FRAME = 2

  def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False, scenario=SCENARIO_DEFAULT,
               enable_hcc=False, scn=None, scn_csv=None, output_csv=None, output_graph=None):
    """Configure bridge behavior and test/scenario options before spawning world."""
    should_enable_hcc = enable_hcc or scenario in STRAIGHT_ROAD_SCENARIOS or scn is not None
    super().__init__(dual_camera, high_quality, enable_hcc=should_enable_hcc)

    self.should_render = False
    self.test_run = test_run
    self.test_duration = test_duration if self.test_run else math.inf
    self.scenario = scenario
    self.scn = scn
    self.scn_csv = scn_csv
    self.output_csv = output_csv
    self.output_graph = output_graph
    self.output_control_method = "hccc" if should_enable_hcc else "default"
    self.output_vehicle_name = "honda_civic_2022"

  def spawn_world(self, queue: Queue):
    """Create and return a MetaDriveWorld instance with scenario-specific config."""
    # Base camera sensor setup expected by camerad bridge.
    sensor_config = {
      "rgb_road": (RGBCameraRoad, W, H, )
    }

    if self.dual_camera:
      sensor_config["rgb_wide"] = (RGBCameraWide, W, H)

    is_replay_profile = self.scn is not None
    uses_straight_map = is_replay_profile or self.scenario in STRAIGHT_ROAD_SCENARIOS
    enable_lead_vehicle = is_replay_profile or self.scenario in LEAD_SCENARIOS
    map_config = create_straight_map() if uses_straight_map else create_map()
    lead_start_delay_s = 0.0 if is_replay_profile else 5.0

    world_config = dict(
      use_render=self.should_render,
      vehicle_config=dict(
        enable_reverse=False,
        render_vehicle=False,
        image_source="rgb_road",
      ),
      sensors=sensor_config,
      image_on_cuda=_cuda_enable,
      image_observation=True,
      interface_panel=[],
      out_of_route_done=False,
      on_continuous_line_done=False,
      crash_vehicle_done=False,
      crash_object_done=False,
      arrive_dest_done=False,
      traffic_density=0.0,
      lead_vehicle_enabled=enable_lead_vehicle,
      lead_vehicle_distance=8.0,
      lead_start_delay_s=lead_start_delay_s,
      lead_vehicle_lateral_offset=0.0,
      lead_vehicle_model="m",
      lead_vehicle_render=True,
      lead_profile_scn=self.scn,
      lead_profile_csv=self.scn_csv,
      lead_profile_output_csv=self.output_csv,
      lead_profile_output_graph=self.output_graph,
      lead_profile_output_control_method=self.output_control_method,
      lead_profile_output_vehicle_name=self.output_vehicle_name,
      steer_cmd_ratio=1.2,
      sim_step_frames=self.TICKS_PER_FRAME,
      camera_capture_frames=5,
      map_config=map_config,
      decision_repeat=1,
      physics_world_step_size=self.TICKS_PER_FRAME/100,
      preload_models=False,
      show_logo=False,
      anisotropic_filtering=False
    )

    return MetaDriveWorld(queue, world_config, self.test_duration, self.test_run, self.dual_camera)
