import ctypes
import functools
import multiprocessing
import time
import numpy as np

from multiprocessing import Pipe, Array

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType
from openpilot.tools.sim.bridge.metadrive.metadrive_process import metadrive_process, metadrive_simulation_state, metadrive_vehicle_state
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H

DISTANCE_MOVE_THRESHOLD_M = 1.0
NOT_MOVING_CHECK_AFTER_ENGAGE_S = 5.0
NOT_MOVING_CHECK_PERIOD_S = 29.0


class MetaDriveWorld(World):
  """Bridge-side world wrapper that hosts a dedicated MetaDrive worker process."""
  def __init__(self, status_q, config: dict, test_duration, test_run, dual_camera=False):
    """Start the MetaDrive worker process and initialize bridge-side shared state."""
    super().__init__(dual_camera)
    self.status_q = status_q

    # Shared memory backing for camera frames produced by the MetaDrive process.
    self.camera_array = Array(ctypes.c_uint8, W * H * 3)
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.wide_camera_array = None
    if dual_camera:
      self.wide_camera_array = Array(ctypes.c_uint8, W * H * 3)
      self.wide_road_image = np.frombuffer(self.wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    # IPC channels with the dedicated MetaDrive worker process.
    self.controls_send, self.controls_recv = Pipe()
    self.simulation_state_send, self.simulation_state_recv = Pipe()
    self.vehicle_state_send, self.vehicle_state_recv = Pipe()

    self.exit_event = multiprocessing.Event()
    self.op_engaged = multiprocessing.Event()

    self.test_run = test_run

    self.first_engage_time = None
    self.last_motion_check_time = 0.0
    self.distance_moved_since_check = 0.0

    self.metadrive_process = multiprocessing.Process(
      name="metadrive process",
      target=functools.partial(
        metadrive_process,
        dual_camera,
        config,
        self.camera_array,
        self.wide_camera_array,
        self.image_lock,
        self.controls_recv,
        self.simulation_state_send,
        self.vehicle_state_send,
        self.exit_event,
        self.op_engaged,
        test_duration,
        self.test_run,
      ),
    )

    self.metadrive_process.start()
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "starting"))

    print("----------------------------------------------------------")
    print("---- Spawning Metadrive world, this might take awhile ----")
    print("----------------------------------------------------------")

    # Wait for a state message to ensure metadrive has launched.
    self.last_vehicle_position_xy = self.vehicle_state_recv.recv().position
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))

    self.ego_control_command = [0.0, 0.0]
    self.should_reset = False

  def apply_controls(self, steer_angle, throttle_out, brake_out, bridge_telemetry=None):
    """Send latest actuation plus optional replay telemetry to the worker process."""
    self.ego_control_command[0] = steer_angle
    self.ego_control_command[1] = throttle_out if throttle_out else -brake_out

    self.controls_send.send([*self.ego_control_command, self.should_reset, bridge_telemetry or {}])
    self.should_reset = False

  def read_state(self):
    """Consume simulation lifecycle updates (running/done) from the worker."""
    while self.simulation_state_recv.poll(0):
      md_state: metadrive_simulation_state = self.simulation_state_recv.recv()
      if md_state.done:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, md_state.done_info))
        self.exit_event.set()

  def _track_test_motion(self, curr_pos, is_engaged: bool):
    """Track post-engagement movement and terminate test-runs if ego is stuck."""
    if is_engaged and self.first_engage_time is None:
      self.first_engage_time = time.monotonic()
      self.op_engaged.set()

    should_check_not_moving = (
      is_engaged and
      self.first_engage_time is not None and
      (time.monotonic() - self.first_engage_time >= NOT_MOVING_CHECK_AFTER_ENGAGE_S) and
      self.test_run
    )

    delta_x_m = abs(curr_pos[0] - self.last_vehicle_position_xy[0])
    delta_y_m = abs(curr_pos[1] - self.last_vehicle_position_xy[1])
    if delta_x_m >= DISTANCE_MOVE_THRESHOLD_M or delta_y_m >= DISTANCE_MOVE_THRESHOLD_M:
      self.distance_moved_since_check += delta_x_m + delta_y_m

    now_monotonic = time.monotonic()
    seconds_since_last_check = now_monotonic - self.last_motion_check_time
    if seconds_since_last_check >= NOT_MOVING_CHECK_PERIOD_S:
      if should_check_not_moving and self.distance_moved_since_check == 0:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, {"vehicle_not_moving": True}))
        self.exit_event.set()

      self.last_motion_check_time = now_monotonic
      self.distance_moved_since_check = 0.0
      self.last_vehicle_position_xy = curr_pos

  def read_sensors(self, state: SimulatorState):
    """Drain latest vehicle state samples from worker into SimulatorState."""
    while self.vehicle_state_recv.poll(0):
      md_vehicle: metadrive_vehicle_state = self.vehicle_state_recv.recv()
      curr_pos = md_vehicle.position

      state.velocity = md_vehicle.velocity
      state.bearing = md_vehicle.bearing
      state.steering_angle = md_vehicle.steering_angle
      state.position_xy = (float(curr_pos[0]), float(curr_pos[1]))
      state.gps.from_xy(curr_pos)
      state.lead_status = md_vehicle.lead_status
      state.lead_d_rel = md_vehicle.lead_d_rel
      state.lead_y_rel = md_vehicle.lead_y_rel
      state.lead_v_rel = md_vehicle.lead_v_rel
      state.lead_a_rel = md_vehicle.lead_a_rel
      state.lead_vehicle_valid = md_vehicle.lead_vehicle_valid
      state.lead_vehicle_velocity = md_vehicle.lead_vehicle_velocity
      state.lead_vehicle_bearing = md_vehicle.lead_vehicle_bearing
      state.lead_vehicle_steering_angle = md_vehicle.lead_vehicle_steering_angle
      state.debug_has_lane = md_vehicle.debug_has_lane
      state.debug_on_lane = md_vehicle.debug_on_lane
      state.debug_lane_s = md_vehicle.debug_lane_s
      state.debug_lane_lateral = md_vehicle.debug_lane_lateral
      state.debug_lane_heading_error_deg = md_vehicle.debug_lane_heading_error_deg
      state.debug_on_yellow_line = md_vehicle.debug_on_yellow_line
      state.debug_on_white_line = md_vehicle.debug_on_white_line
      state.debug_crash_sidewalk = md_vehicle.debug_crash_sidewalk
      state.debug_out_of_route = md_vehicle.debug_out_of_route
      state.valid = True

      self._track_test_motion(curr_pos, state.is_engaged)

  def read_cameras(self):
    """Camera frames are already written via shared memory; no pull step needed."""
    pass

  def tick(self):
    """No-op tick hook for API compatibility with other simulator worlds."""
    pass

  def reset(self):
    """Request a world reset in the worker process on next control cycle."""
    self.should_reset = True

  def close(self, reason: str):
    """Stop the worker process and publish bridge close status."""
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    self.metadrive_process.join()
