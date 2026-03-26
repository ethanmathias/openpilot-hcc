import functools
import queue as pyqueue
import signal
import threading
from collections import namedtuple
from multiprocessing import Process, Queue, Value

from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.bridge.common import QueueMessageType
from openpilot.tools.sim.lib.common import SimulatorState
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

from openpilot.tools.sim.beamng.hc3_adapter import (
  ReferenceLongitudinalBlender,
  accel_to_pedal_commands,
  calibrated_hc3_pedal_commands,
  create_hc3_controller,
)
from openpilot.tools.sim.beamng.telemetry import TelemetryLogger
from openpilot.tools.sim.beamng.world import BeamNGConfig, BeamNGWorld

BRIDGE_MODE_OPENPILOT = "openpilot"
BRIDGE_MODE_REFERENCE = "beamng_reference"

QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])


def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()


class BeamNGBridge:
  TICKS_PER_FRAME = 1

  def __init__(self, dual_camera=False, high_quality=False, enable_hcc=False, bridge_mode=BRIDGE_MODE_OPENPILOT,
               hc3_variant="new", hc3_controller_path=None, beamng_config: BeamNGConfig | None = None,
               output_csv: str | None = None):
    set_params_enabled()
    self.params = Params()
    self.params.put_bool("AlphaLongitudinalEnabled", True)
    if self.params.check_key("EnableHCCC"):
      self.params.put_bool("EnableHCCC", bool(enable_hcc and bridge_mode == BRIDGE_MODE_OPENPILOT))

    self.dual_camera = dual_camera
    self.high_quality = high_quality
    self.enable_hcc = bool(enable_hcc)
    self.bridge_mode = bridge_mode
    self.hc3_variant = hc3_variant
    self.hc3_controller_path = hc3_controller_path
    self.beamng_config = beamng_config or BeamNGConfig()
    self.output_csv = output_csv

    self.started = Value("i", False)
    self._keep_alive = True
    self._exit_event: threading.Event | None = None
    self._reference_blender = None
    self._hccc_throttle_prev = 0.0
    self._hccc_brake_prev = 0.0
    self._past_startup_engaged = False
    self._startup_button_prev = True

    signal.signal(signal.SIGTERM, self._on_shutdown)

  def _on_shutdown(self, signal_num, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False
    if self._exit_event is not None:
      self._exit_event.set()

  def run(self, queue: Queue):
    process = Process(name="beamng_bridge", target=self._bridge_keep_alive, args=(queue,))
    process.start()
    return process

  def _bridge_keep_alive(self, queue: Queue):
    try:
      self._run(queue)
    finally:
      self.started.value = False

  def _ensure_reference_blender(self):
    if self._reference_blender is None:
      controller = create_hc3_controller(
        self.hc3_variant,
        self.world.ego_vehicle,
        self.world.preceding_vehicle,
        dt=0.1,
        controller_path=self.hc3_controller_path,
      )
      self._reference_blender = ReferenceLongitudinalBlender(controller, update_dt=0.1, sim_dt=self.beamng_config.step_s)

  def _compute_openpilot_longitudinal(self, throttle_manual: float, brake_manual: float) -> dict[str, float | bool]:
    accel_cmd = float(self.simulated_car.sm["carControl"].actuators.accel)
    if self.enable_hcc:
      throttle_out, brake_out = calibrated_hc3_pedal_commands(accel_cmd, self._hccc_throttle_prev, self._hccc_brake_prev)
      self._hccc_throttle_prev = throttle_out
      self._hccc_brake_prev = brake_out
    else:
      throttle_out, brake_out = accel_to_pedal_commands(accel_cmd, False)

    return {
      "throttle_out": float(throttle_out),
      "brake_out": float(brake_out),
      "manual_accel": float(self.simulated_car.sm["controlsState"].ufAccelCmd) if self.simulated_car.sm.valid.get("controlsState", False) else 0.0,
      "hccc_accel": float(self.simulated_car.sm["controlsState"].uiAccelCmd) if self.simulated_car.sm.valid.get("controlsState", False) else 0.0,
      "final_accel": float(accel_cmd),
      "hccc_active": bool(
        self.enable_hcc and
        self.simulated_car.sm.valid.get("radarState", False) and
        self.simulated_car.sm["radarState"].leadOne.status
      ),
      "driver_gas": float(throttle_manual),
      "driver_brake": float(brake_manual),
    }

  def _compute_reference_longitudinal(self, throttle_manual: float, brake_manual: float) -> dict[str, float | bool]:
    self._ensure_reference_blender()
    result = self._reference_blender.step(throttle_manual, brake_manual)
    return {
      "throttle_out": float(result.throttle_out),
      "brake_out": float(result.brake_out),
      "manual_accel": float(result.manual_accel),
      "hccc_accel": float(result.hccc_accel),
      "final_accel": float(result.final_accel),
      "hccc_active": True,
      "driver_gas": float(throttle_manual),
      "driver_brake": float(brake_manual),
    }

  def _compute_engaged_longitudinal(self, throttle_manual: float, brake_manual: float) -> dict[str, float | bool]:
    if self.bridge_mode == BRIDGE_MODE_REFERENCE:
      return self._compute_reference_longitudinal(throttle_manual, brake_manual)
    return self._compute_openpilot_longitudinal(throttle_manual, brake_manual)

  def _run(self, queue: Queue):
    self.world = BeamNGWorld(self.beamng_config, self.dual_camera)
    self.simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors(self.dual_camera)
    self.simulator_state = SimulatorState()
    self.telemetry = TelemetryLogger(self.output_csv, self.bridge_mode, self.hc3_variant if self.bridge_mode == BRIDGE_MODE_REFERENCE else None)

    self._exit_event = threading.Event()
    self.simulated_car_thread = threading.Thread(
      target=rk_loop,
      args=(functools.partial(self.simulated_car.update, self.simulator_state), 100, self._exit_event),
      daemon=True,
    )
    self.simulated_camera_thread = threading.Thread(
      target=rk_loop,
      args=(functools.partial(self.simulated_sensors.send_camera_images, self.world), 20, self._exit_event),
      daemon=True,
    )
    self.simulated_car_thread.start()
    self.simulated_camera_thread.start()

    rk = Ratekeeper(100, None)
    throttle_manual = steer_manual = brake_manual = 0.0
    throttle_ts = steer_ts = brake_ts = 0.0
    manual_ttl = 0.35

    try:
      while self._keep_alive:
        now = rk.frame / 100.0
        self.simulator_state.cruise_button = 0
        self.simulator_state.left_blinker = False
        self.simulator_state.right_blinker = False

        while True:
          try:
            message = queue.get_nowait()
          except pyqueue.Empty:
            break

          if message.type == QueueMessageType.CONTROL_COMMAND:
            parts = message.info.split("_")
            if parts[0] == "steer":
              steer_manual = float(parts[1])
              steer_ts = now
            elif parts[0] == "throttle":
              throttle_manual = float(parts[1])
              throttle_ts = now
            elif parts[0] == "brake":
              brake_manual = float(parts[1])
              brake_ts = now
            elif parts[0] == "cruise":
              if parts[1] == "down":
                self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
              elif parts[1] == "up":
                self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
              elif parts[1] == "cancel":
                self.simulator_state.cruise_button = CruiseButtons.CANCEL
              elif parts[1] == "main":
                self.simulator_state.cruise_button = CruiseButtons.MAIN
            elif parts[0] == "blinker":
              if parts[1] == "left":
                self.simulator_state.left_blinker = True
              elif parts[1] == "right":
                self.simulator_state.right_blinker = True
            elif parts[0] == "ignition":
              self.simulator_state.ignition = not self.simulator_state.ignition
            elif parts[0] == "reset":
              self.world.reset()
            elif parts[0] == "quit":
              self._keep_alive = False
              break

        if not self._keep_alive:
          break

        if now - steer_ts > manual_ttl:
          steer_manual = 0.0
        if now - throttle_ts > manual_ttl:
          throttle_manual = 0.0
        if now - brake_ts > manual_ttl:
          brake_manual = 0.0

        self.simulator_state.user_gas = throttle_manual
        self.simulator_state.user_brake = brake_manual
        self.simulator_state.user_torque = steer_manual * -10000.0

        steer_out = steer_manual * -40.0

        self.simulated_sensors.update(self.simulator_state, self.world)
        self.simulated_car.sm.update(0)
        self.simulator_state.is_engaged = bool(self.simulated_car.sm["selfdriveState"].active)

        throttle_out = throttle_manual
        brake_out = brake_manual
        longitudinal = {
          "manual_accel": 0.0,
          "hccc_accel": 0.0,
          "final_accel": 0.0,
          "hccc_active": False,
          "driver_gas": float(throttle_manual),
          "driver_brake": float(brake_manual),
        }

        if self.simulator_state.is_engaged:
          longitudinal = self._compute_engaged_longitudinal(throttle_manual, brake_manual)
          throttle_out = float(longitudinal["throttle_out"])
          brake_out = float(longitudinal["brake_out"])
          self._past_startup_engaged = True
        elif not self._past_startup_engaged and self.simulated_car.sm["selfdriveState"].engageable:
          self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if self._startup_button_prev else CruiseButtons.MAIN
          self._startup_button_prev = not self._startup_button_prev
        else:
          self._hccc_throttle_prev = 0.0
          self._hccc_brake_prev = 0.0
          if self._reference_blender is not None:
            self._reference_blender.reset()

        self.world.apply_controls(steer_out, throttle_out, brake_out)
        self.world.read_state()
        self.world.read_sensors(self.simulator_state)
        self.world.tick()
        self.world.read_cameras()

        self.simulator_state.carstate_a_ego = float(self.simulated_car.sm["carState"].aEgo) if self.simulated_car.sm.valid.get("carState", False) else 0.0
        carstate_v_ego = float(self.simulated_car.sm["carState"].vEgo) if self.simulated_car.sm.valid.get("carState", False) else 0.0
        planner_a_target = float(self.simulated_car.sm["longitudinalPlan"].aTarget) if self.simulated_car.sm.valid.get("longitudinalPlan", False) else 0.0

        self.telemetry.log({
          "target_speed_pre": self.world.target_speed_pre,
          "carstate_v_ego": carstate_v_ego,
          "carstate_a_ego": self.simulator_state.carstate_a_ego,
          "lead_d_rel": self.simulator_state.lead_d_rel,
          "lead_v_rel": self.simulator_state.lead_v_rel,
          "lead_a_rel": self.simulator_state.lead_a_rel,
          "planner_a_target": planner_a_target,
          "hccc_accel": longitudinal["hccc_accel"],
          "manual_accel": longitudinal["manual_accel"],
          "final_accel": longitudinal["final_accel"],
          "hccc_active": longitudinal["hccc_active"],
          "driver_gas": longitudinal["driver_gas"],
          "driver_brake": longitudinal["driver_brake"],
          "controller_throttle": throttle_out,
          "controller_brake": brake_out,
        })

        self.started.value = True
        rk.keep_time()
    finally:
      self.telemetry.close()
      self.shutdown()
      self.world.close("beamng bridge terminated")
