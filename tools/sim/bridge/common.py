import signal
import threading
import functools
import numpy as np
import queue as pyqueue

from collections import namedtuple
from enum import Enum
from multiprocessing import Process, Queue, Value
from abc import ABC, abstractmethod

from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

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

  def __init__(self, dual_camera, high_quality, enable_hcc=False):
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

    self._exit_event: threading.Event | None = None
    self._threads = []
    self._keep_alive = True
    self.started = Value('i', False)
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self.simulator_state = SimulatorState()

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
    self.simulated_sensors = SimulatedSensors(self.dual_camera)

    self._exit_event = threading.Event()

    self.simulated_car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_car.update, self.simulator_state),
                                                                        100, self._exit_event))
    self.simulated_car_thread.start()

    self.simulated_camera_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
                                                                        20, self._exit_event))
    self.simulated_camera_thread.start()

    # Simulation tends to be slow in the initial steps. This prevents lagging later
    for _ in range(20):
      self.world.tick()

    throttle_manual = steer_manual = brake_manual = 0.0
    manual_ttl = 0.35
    steer_manual_ts = throttle_manual_ts = brake_manual_ts = 0.0

    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = steer_op = brake_op = 0.0

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
          m = message.info.split('_')
          if m[0] == "steer":
            steer_manual = float(m[1])
            steer_manual_ts = now
          elif m[0] == "throttle":
            throttle_manual = float(m[1])
            throttle_manual_ts = now
          elif m[0] == "brake":
            brake_manual = float(m[1])
            brake_manual_ts = now
          elif m[0] == "cruise":
            if m[1] == "down":
              self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
            elif m[1] == "up":
              self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
            elif m[1] == "cancel":
              self.simulator_state.cruise_button = CruiseButtons.CANCEL
            elif m[1] == "main":
              self.simulator_state.cruise_button = CruiseButtons.MAIN
          elif m[0] == "blinker":
            if m[1] == "left":
              self.simulator_state.left_blinker = True
            elif m[1] == "right":
              self.simulator_state.right_blinker = True
          elif m[0] == "ignition":
            self.simulator_state.ignition = not self.simulator_state.ignition
          elif m[0] == "reset":
            self.world.reset()
          elif m[0] == "quit":
            self._keep_alive = False
            break

      if not self._keep_alive:
        break

      if now - steer_manual_ts > manual_ttl:
        steer_manual = 0.0
      if now - throttle_manual_ts > manual_ttl:
        throttle_manual = 0.0
      if now - brake_manual_ts > manual_ttl:
        brake_manual = 0.0

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
      self.simulator_state.user_torque = steer_manual * -10000

      steer_manual = steer_manual * -40

      # Update openpilot on current sensor state
      self.simulated_sensors.update(self.simulator_state, self.world)

      self.simulated_car.sm.update(0)
      self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active

      if self.simulator_state.is_engaged:
        throttle_op = np.clip(self.simulated_car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
        brake_op = np.clip(-self.simulated_car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)

        self.past_startup_engaged = True
      elif not self.past_startup_engaged and self.simulated_car.sm['selfdriveState'].engageable:
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN # force engagement on startup
        self.startup_button_prev = not self.startup_button_prev

      if self.simulator_state.is_engaged:
        # Human keeps authority: manual brake/throttle can override engaged longitudinal control.
        if brake_manual > 1e-6:
          throttle_out = 0.0
          brake_out = np.clip(brake_op + brake_manual * (1.0 - brake_op), 0.0, 1.0)
        elif throttle_manual > 1e-6:
          throttle_out = np.clip(throttle_op + throttle_manual * (1.0 - throttle_op), 0.0, 1.0)
          brake_out = 0.0
        else:
          throttle_out = throttle_op
          brake_out = brake_op

        steer_out = steer_manual
      else:
        throttle_out = throttle_manual
        brake_out = brake_manual
        steer_out = steer_manual

      self.world.apply_controls(steer_out, throttle_out, brake_out)
      self.world.read_state()
      self.world.read_sensors(self.simulator_state)

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
