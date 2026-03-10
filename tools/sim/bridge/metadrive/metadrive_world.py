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
  def __init__(self, status_q, config: dict, test_duration, test_run, dual_camera=False):
    super().__init__(dual_camera)
    self.status_q = status_q
    self.camera_array = Array(ctypes.c_uint8, W * H * 3)
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.wide_camera_array = None
    if dual_camera:
      self.wide_camera_array = Array(ctypes.c_uint8, W * H * 3)
      self.wide_road_image = np.frombuffer(self.wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.controls_send, self.controls_recv = Pipe()
    self.simulation_state_send, self.simulation_state_recv = Pipe()
    self.vehicle_state_send, self.vehicle_state_recv = Pipe()

    self.exit_event = multiprocessing.Event()
    self.op_engaged = multiprocessing.Event()

    self.test_run = test_run

    self.first_engage = None
    self.last_check_timestamp = 0
    self.distance_moved = 0

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
    self.vehicle_last_pos = self.vehicle_state_recv.recv().position
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))

    self.vc = [0.0, 0.0]
    self.should_reset = False

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    self.vc[0] = steer_angle
    self.vc[1] = throttle_out if throttle_out else -brake_out

    self.controls_send.send([*self.vc, self.should_reset])
    self.should_reset = False

  def read_state(self):
    while self.simulation_state_recv.poll(0):
      md_state: metadrive_simulation_state = self.simulation_state_recv.recv()
      if md_state.done:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, md_state.done_info))
        self.exit_event.set()

  def _track_test_motion(self, curr_pos, is_engaged: bool):
    if is_engaged and self.first_engage is None:
      self.first_engage = time.monotonic()
      self.op_engaged.set()

    after_engaged_check = is_engaged and self.first_engage is not None and (time.monotonic() - self.first_engage >= NOT_MOVING_CHECK_AFTER_ENGAGE_S) and self.test_run

    x_dist = abs(curr_pos[0] - self.vehicle_last_pos[0])
    y_dist = abs(curr_pos[1] - self.vehicle_last_pos[1])
    if x_dist >= DISTANCE_MOVE_THRESHOLD_M or y_dist >= DISTANCE_MOVE_THRESHOLD_M:
      self.distance_moved += x_dist + y_dist

    current_time = time.monotonic()
    since_last_check = current_time - self.last_check_timestamp
    if since_last_check >= NOT_MOVING_CHECK_PERIOD_S:
      if after_engaged_check and self.distance_moved == 0:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, {"vehicle_not_moving": True}))
        self.exit_event.set()

      self.last_check_timestamp = current_time
      self.distance_moved = 0
      self.vehicle_last_pos = curr_pos

  def read_sensors(self, state: SimulatorState):
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
    pass

  def tick(self):
    pass

  def reset(self):
    self.should_reset = True

  def close(self, reason: str):
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    self.metadrive_process.join()
