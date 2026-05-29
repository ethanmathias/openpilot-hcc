import signal
import threading
import functools
import numpy as np
import queue as pyqueue

from collections import namedtuple
from enum import Enum
from multiprocessing import Process, Queue, Value
from abc import ABC, abstractmethod

import cereal.messaging as messaging
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

# Scaling factors to convert openpilot's accel (m/s²) to MetaDrive's
# 0–1 throttle/brake inputs. These are empirical tuning constants.
_ACCEL_TO_THROTTLE = 1.6   # accel / _ACCEL_TO_THROTTLE → throttle [0, 1]
_ACCEL_TO_BRAKE = 4.0      # -accel / _ACCEL_TO_BRAKE → brake [0, 1]

# Scaling from manual wheel input to CAN-compatible torque/steer values
_MANUAL_STEER_TO_TORQUE = -10000
_MANUAL_STEER_TO_ANGLE = -40

# Human manual-input time-to-live: after this many seconds without a new
# input message, manual steer/throttle/brake decay to zero.
_MANUAL_INPUT_TTL_SECS = 0.35

# MetaDrive warmup ticks before starting the control loop. Lets the
# physics engine settle and initial frames render.
_WARMUP_TICKS = 20

# Cruise button name → CruiseButtons enum mapping
_CRUISE_COMMANDS = {
  "down": CruiseButtons.DECEL_SET,
  "up": CruiseButtons.RES_ACCEL,
  "cancel": CruiseButtons.CANCEL,
  "main": CruiseButtons.MAIN,
}

QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])

class QueueMessageType(Enum):
  START_STATUS = 0
  CONTROL_COMMAND = 1
  TERMINATION_INFO = 2
  CLOSE_STATUS = 3

def control_cmd_gen(cmd: str):
  return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)

def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()


class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, enable_hcc=False, lead_sim_prefix=None):
    set_params_enabled()
    self.params = Params()
    self.params.put_bool("AlphaLongitudinalEnabled", True)
    if enable_hcc:
      if self.params.check_key("EnableHCCC"):
        self.params.put_bool("EnableHCCC", True)
      else:
        print("[WARNING] EnableHCCC param key is unavailable in this build; --enable_hcc ignored.")

    self.rk = Ratekeeper(100, None)

    self.dual_camera = dual_camera
    self.high_quality = high_quality
    self.lead_sim_prefix = lead_sim_prefix

    self._exit_event: threading.Event | None = None
    self._keep_alive = True
    self.started = Value('i', False)
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self.simulator_state = SimulatorState()
    self.lead_simulator_state = SimulatorState()

    self.world: World | None = None

    self.past_startup_engaged = False
    self.startup_button_prev = True

    self.test_run = False

  def _on_shutdown(self, signal, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False

  def bridge_keep_alive(self, q: Queue, retries: int):
    try:
      self._run(q)
    finally:
      self.close("bridge terminated")

  def close(self, reason):
    self.started.value = False

    if self._exit_event is not None:
      self._exit_event.set()

    if self.world is not None:
      self.world.close(reason)

  def run(self, queue, retries=-1):
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, args=(queue, retries))
    bridge_p.start()
    return bridge_p

  def print_status(self):
    lead_line = "Lead: unavailable"
    ego_line = "Ego: unavailable"
    loc_line = "Location: unavailable"
    lane_line = "Lane: unavailable"

    if hasattr(self, "simulated_car"):
      sm = self.simulated_car.sm
      if sm.valid.get('carState', False):
        v_ego = sm['carState'].vEgo
        a_ego = sm['carState'].aEgo
        accel_cmd = sm['carControl'].actuators.accel
        ego_line = f"Ego: vEgo={v_ego:.2f} m/s aEgo={a_ego:.2f} m/s^2 accelCmd={accel_cmd:.2f} m/s^2"

      if sm.valid.get('radarState', False):
        lead = sm['radarState'].leadOne
        if lead.status:
          lead_line = (
            f"Lead: dRel={lead.dRel:.2f} m vLead={lead.vLead:.2f} m/s "
            f"vRel={lead.vRel:.2f} m/s yRel={lead.yRel:.2f} m"
          )
        else:
          lead_line = "Lead: no valid lead"

    if self.simulator_state.position_xy is not None:
      x, y = self.simulator_state.position_xy
      loc_line = (
        f"Location: x={x:.2f} m y={y:.2f} m "
        f"lat={self.simulator_state.gps.latitude:.6f} lon={self.simulator_state.gps.longitude:.6f} "
        f"bearing={self.simulator_state.bearing:.1f} deg"
      )

    if self.simulator_state.debug_has_lane:
      lane_s = self.simulator_state.debug_lane_s
      lane_lateral = self.simulator_state.debug_lane_lateral
      lane_heading_error_deg = self.simulator_state.debug_lane_heading_error_deg
      lane_line = (
        f"Lane: onLane={self.simulator_state.debug_on_lane} "
        f"s={lane_s:.2f} m latOff={lane_lateral:.2f} m "
        f"hdgErr={lane_heading_error_deg:.2f} deg "
        f"yellow={self.simulator_state.debug_on_yellow_line} "
        f"white={self.simulator_state.debug_on_white_line} "
        f"sidewalk={self.simulator_state.debug_crash_sidewalk} "
        f"outOfRoute={self.simulator_state.debug_out_of_route}"
      )

    print(
    f"""
State:
Ignition: {self.simulator_state.ignition} Engaged: {self.simulator_state.is_engaged}
{ego_line}
{lead_line}
{loc_line}
{lane_line}
    """)

  @abstractmethod
  def spawn_world(self, q: Queue) -> World:
    pass

  def _run(self, q: Queue):
    self.world = self.spawn_world(q)

    self.simulated_car = SimulatedCar()
    self.lead_simulated_car = None
    if self.lead_sim_prefix:
      # The lead-side simulator needs a real OPENPILOT_PREFIX namespace so its
      # pub/sub sockets bind to hcclead instead of colliding with the ego side.
      with OpenpilotPrefix(self.lead_sim_prefix, create_dirs_on_enter=True, clean_dirs_on_exit=False):
        self.lead_simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors(self.dual_camera)

    self._exit_event = threading.Event()

    self.simulated_car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_car.update, self.simulator_state),
                                                                        100, self._exit_event))
    self.simulated_car_thread.start()
    self.lead_simulated_car_thread = None
    if self.lead_simulated_car is not None:
      self.lead_simulated_car_thread = threading.Thread(
        target=rk_loop,
        args=(functools.partial(self.lead_simulated_car.update, self.lead_simulator_state), 100, self._exit_event),
      )
      self.lead_simulated_car_thread.start()

    self.simulated_camera_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
                                                                        20, self._exit_event))
    self.simulated_camera_thread.start()

    for _ in range(_WARMUP_TICKS):
      self.world.tick()

    throttle_manual = steer_manual = brake_manual = 0.0
    steer_manual_ts = throttle_manual_ts = brake_manual_ts = 0.0

    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = brake_op = 0.0

      self.simulator_state.cruise_button = 0
      self.simulator_state.left_blinker = False
      self.simulator_state.right_blinker = False

      now = self.rk.frame / 100.0

      # Drain queued manual controls each frame and keep latest axis values.
      while True:
        try:
          message = q.get_nowait()
        except pyqueue.Empty:
          break

        if message.type == QueueMessageType.CONTROL_COMMAND:
          command, value = message.info.split('_', 1)
          if command == "steer":
            steer_manual = float(value)
            steer_manual_ts = now
          elif command == "throttle":
            throttle_manual = float(value)
            throttle_manual_ts = now
          elif command == "brake":
            brake_manual = float(value)
            brake_manual_ts = now
          elif command == "cruise":
            self.simulator_state.cruise_button = _CRUISE_COMMANDS.get(value, 0)
          elif command == "blinker":
            self.simulator_state.left_blinker = (value == "left")
            self.simulator_state.right_blinker = (value == "right")
          elif command == "ignition":
            self.simulator_state.ignition = not self.simulator_state.ignition
          elif command == "reset":
            self.world.reset()
          elif command == "quit":
            self._keep_alive = False
            break

      if not self._keep_alive:
        break

      if now - steer_manual_ts > _MANUAL_INPUT_TTL_SECS:
        steer_manual = 0.0
      if now - throttle_manual_ts > _MANUAL_INPUT_TTL_SECS:
        throttle_manual = 0.0
      if now - brake_manual_ts > _MANUAL_INPUT_TTL_SECS:
        brake_manual = 0.0

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
      self.simulator_state.user_torque = steer_manual * _MANUAL_STEER_TO_TORQUE

      steer_angle = steer_manual * _MANUAL_STEER_TO_ANGLE

      # Update openpilot on current sensor state
      self.simulated_sensors.update(self.simulator_state, self.world)

      self.simulated_car.sm.update(0)
      self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active

      if self.simulator_state.is_engaged:
        accel = self.simulated_car.sm['carControl'].actuators.accel
        throttle_op = float(np.clip(accel / _ACCEL_TO_THROTTLE, 0.0, 1.0))
        brake_op = float(np.clip(-accel / _ACCEL_TO_BRAKE, 0.0, 1.0))

        self.past_startup_engaged = True
      elif not self.past_startup_engaged and self.simulated_car.sm['selfdriveState'].engageable:
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN # force engagement on startup
        self.startup_button_prev = not self.startup_button_prev

      if self.simulator_state.is_engaged:
        # Cooperative manual input is already blended into actuators.accel upstream.
        throttle_out = throttle_op
        brake_out = brake_op
        steer_out = steer_angle
      else:
        throttle_out = throttle_manual
        brake_out = brake_manual
        steer_out = steer_angle

      carstate = self.simulated_car.sm['carState']
      controls_state = self.simulated_car.sm['controlsState']
      carstate_a_ego = float(getattr(carstate, 'aEgo', 0.0))
      carstate_v_ego = float(getattr(carstate, 'vEgo', 0.0))
      # Do not zero these debug fields just because controlsState.valid lags for
      # a frame. For replay analysis we care about the last published values the
      # bridge observed, especially when carControl is already nonzero.
      planner_a_target = float(getattr(controls_state, 'upAccelCmd', 0.0))
      hccc_accel = float(getattr(controls_state, 'uiAccelCmd', 0.0))
      manual_accel = float(getattr(controls_state, 'ufAccelCmd', 0.0))
      final_accel = float(self.simulated_car.sm['carControl'].actuators.accel) if self.simulated_car.sm.valid.get('carControl', False) else 0.0
      sim_track_status = bool(self.simulator_state.lead_status)
      sim_track_d_rel = float(self.simulator_state.lead_d_rel) if sim_track_status else 0.0
      sim_track_v_rel = float(self.simulator_state.lead_v_rel) if sim_track_status else 0.0
      live_tracks_seq = int(getattr(self.simulated_car, 'debug_live_tracks_seq', 0))
      live_tracks_point_count = int(getattr(self.simulated_car, 'debug_live_tracks_point_count', 0))
      live_tracks_d_rel = float(getattr(self.simulated_car, 'debug_live_tracks_d_rel', 0.0))
      live_tracks_v_rel = float(getattr(self.simulated_car, 'debug_live_tracks_v_rel', 0.0))

      radar_lead_v_rel = 0.0
      radar_lead_d_rel = 0.0
      radar_lead_is_radar = False
      radar_lead_track_id = -1
      hccc_active = False
      if self.simulated_car.sm.valid.get('radarState', False):
        radar_lead = self.simulated_car.sm['radarState'].leadOne
        if radar_lead.status:
          radar_lead_v_rel = float(radar_lead.vRel)
          radar_lead_d_rel = float(radar_lead.dRel)
          radar_lead_is_radar = bool(getattr(radar_lead, "radar", False))
          radar_lead_track_id = int(getattr(radar_lead, "radarTrackId", -1))
          hccc_active = bool(self.simulator_state.is_engaged)

      # Carry these debug values over to the simulator worker so replay CSVs can
      # include both the plant response and the openpilot-side HC3 diagnostics.
      bridge_telemetry = {
        "carstate_a_ego": carstate_a_ego,
        "carstate_v_ego": carstate_v_ego,
        "planner_a_target": planner_a_target,
        "hccc_accel": hccc_accel,
        "manual_accel": manual_accel,
        "final_accel": final_accel,
        "sim_track_status": sim_track_status,
        "sim_track_d_rel": sim_track_d_rel,
        "sim_track_v_rel": sim_track_v_rel,
        "live_tracks_seq": live_tracks_seq,
        "live_tracks_point_count": live_tracks_point_count,
        "live_tracks_d_rel": live_tracks_d_rel,
        "live_tracks_v_rel": live_tracks_v_rel,
        "radar_lead_v_rel": radar_lead_v_rel,
        "radar_lead_d_rel": radar_lead_d_rel,
        "radar_lead_is_radar": radar_lead_is_radar,
        "radar_lead_track_id": radar_lead_track_id,
        "radar_lead_speed_est": carstate_v_ego + radar_lead_v_rel if hccc_active else 0.0,
        "hccc_active": hccc_active,
        "driver_gas": float(throttle_manual),
        "driver_brake": float(brake_manual),
      }

      self.world.apply_controls(steer_out, throttle_out, brake_out, bridge_telemetry)
      self.world.read_state()
      self.world.read_sensors(self.simulator_state)
      self._update_lead_simulator_state()

      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      # don't print during test, so no print/IO Block between OP and metadrive processes
      if not self.test_run and self.rk.frame % 25 == 0:
        self.print_status()

      self.started.value = True

      self.rk.keep_time()

  def _update_lead_simulator_state(self):
    if self.lead_simulated_car is None:
      return

    lead_velocity = self.simulator_state.lead_vehicle_velocity
    lead_valid = bool(self.simulator_state.lead_vehicle_valid and lead_velocity is not None)
    self.lead_simulator_state.valid = lead_valid
    self.lead_simulator_state.ignition = True
    self.lead_simulator_state.is_engaged = False
    self.lead_simulator_state.user_gas = 0.0
    self.lead_simulator_state.user_brake = 0.0
    self.lead_simulator_state.user_torque = 0.0
    self.lead_simulator_state.cruise_button = 0
    self.lead_simulator_state.left_blinker = False
    self.lead_simulator_state.right_blinker = False
    self.lead_simulator_state.lead_status = False
    self.lead_simulator_state.lead_d_rel = 0.0
    self.lead_simulator_state.lead_y_rel = 0.0
    self.lead_simulator_state.lead_v_rel = 0.0
    self.lead_simulator_state.lead_a_rel = 0.0

    if not lead_valid:
      return

    self.lead_simulator_state.velocity = lead_velocity
    self.lead_simulator_state.bearing = float(self.simulator_state.lead_vehicle_bearing)
    self.lead_simulator_state.imu.bearing = float(self.simulator_state.lead_vehicle_bearing)
    self.lead_simulator_state.steering_angle = float(self.simulator_state.lead_vehicle_steering_angle)
