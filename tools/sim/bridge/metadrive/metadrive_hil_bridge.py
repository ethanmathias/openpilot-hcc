"""Hardware-in-the-loop variant of MetaDriveBridge.

Each Comma 3X runs the full openpilot stack on-device. The PC's job is just
to be the world: spawn MetaDrive vehicles, ship synthesized camera frames +
CAN/sensor inputs to each device over USB-RNDIS, and apply the device's
returned `carControl` to the corresponding MetaDrive vehicle.

Two-device runs (--lead) require the msgq ZMQ_BIND_ADDRESS patch on the PC —
see cereal_bridges.py and tools/sim/hil/patches/zmq_bind_address.patch.
Single-device (ego-only) runs work with stock msgq.
"""
from __future__ import annotations

import functools
import queue as pyqueue
import threading
from multiprocessing import Queue

import numpy as np

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.bridge.common import QueueMessageType, rk_loop
from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge
from openpilot.tools.sim.hil import cereal_bridges
from openpilot.tools.sim.hil.device_config import DeviceConfig
from openpilot.tools.sim.hil.remote_sensors import RemoteSensors
from openpilot.tools.sim.hil.window import maybe_start_window, update_telemetry_from_sm
from openpilot.tools.sim.lib.common import SimulatorState
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from opendbc.car.honda.values import CruiseButtons


_EGO_PREFIX = "hccego"
_LEAD_PREFIX = "hcclead"

# Scaling factors to convert openpilot's accel (m/s^2) to MetaDrive's
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

# How often (in bridge ticks at 100 Hz) to push telemetry to the HUD overlay
_TELEMETRY_UPDATE_INTERVAL = 10

# Cruise button name → CruiseButtons enum mapping
_CRUISE_COMMANDS = {
  "down": CruiseButtons.DECEL_SET,
  "up": CruiseButtons.RES_ACCEL,
  "cancel": CruiseButtons.CANCEL,
  "main": CruiseButtons.MAIN,
}


def _accel_to_controls(accel: float) -> tuple[float, float]:
  """Convert openpilot accel (m/s^2) → (throttle, brake) in [0, 1]."""
  throttle = float(np.clip(accel / _ACCEL_TO_THROTTLE, 0.0, 1.0))
  brake = float(np.clip(-accel / _ACCEL_TO_BRAKE, 0.0, 1.0))
  return throttle, brake


def _send_lead_camera(sensors, world) -> None:
  """Lead camera path: ship the world's lead-POV frame instead of ego POV."""
  if world.lead_road_image is None:
    return
  world.image_lock.acquire()
  sensors.road_encoder.send_rgb(world.lead_road_image)
  if sensors.wide_encoder is not None and world.dual_camera:
    # Lead has no separate wide camera in M4; reuse the same lead POV for now.
    sensors.wide_encoder.send_rgb(world.lead_road_image)


class _RoleStack:
  """SimulatedCar + RemoteSensors bound to one OPENPILOT_PREFIX, plus the
  cereal-bridge subprocess pair to the corresponding device."""

  def __init__(self, prefix: str, device: DeviceConfig, dual_camera: bool, raw_yuv: bool):
    self.prefix = prefix
    self.device = device
    self.bridge_pair = cereal_bridges.spawn(prefix, device.ip, bind_ip=device.pc_ip)
    with OpenpilotPrefix(prefix, create_dirs_on_enter=True, clean_dirs_on_exit=False):
      messaging.reset_context()
      self.car = SimulatedCar()
      self.sensors = RemoteSensors(device.ip, dual_camera=dual_camera, raw_yuv=raw_yuv)
    self.simulator_state = SimulatorState()

  def close(self) -> None:
    self.sensors.close()
    self.bridge_pair.stop()


class MetaDriveHILBridge(MetaDriveBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera: bool, high_quality: bool, ego_device: DeviceConfig, lead_device: DeviceConfig | None = None,
               raw_yuv: bool = False, scenario: str = "hccc_step", scn=None, scn_csv=None,
               output_csv=None, output_graph=None, lead_device_drive: bool = False):
    super().__init__(dual_camera=dual_camera, high_quality=high_quality, scenario=scenario,
                     enable_hcc=True, scn=scn, scn_csv=scn_csv, output_csv=output_csv,
                     output_graph=output_graph, lead_sim_prefix=None,
                     hil_two_vehicle=lead_device is not None)
    self.ego_device = ego_device
    self.lead_device = lead_device
    self.raw_yuv = raw_yuv
    self.lead_device_drive = lead_device_drive
    self.ego: _RoleStack | None = None
    self.lead: _RoleStack | None = None

  def _run(self, q: Queue) -> None:
    set_params_enabled()
    Params().put_bool("AlphaLongitudinalEnabled", True)

    self.world = self.spawn_world(q)
    self.ego = _RoleStack(_EGO_PREFIX, self.ego_device, self.dual_camera, self.raw_yuv)
    if self.lead_device is not None:
      self.lead = _RoleStack(_LEAD_PREFIX, self.lead_device, self.dual_camera, self.raw_yuv)
    window = maybe_start_window(self.world)

    self._exit_event = threading.Event()
    threads = self._start_role_threads(self.ego, self.ego.sensors.send_camera_images)
    if self.lead is not None:
      threads += self._start_role_threads(self.lead, functools.partial(_send_lead_camera, self.lead.sensors))

    for _ in range(_WARMUP_TICKS):
      self.world.tick()

    steer_manual = throttle_manual = brake_manual = 0.0
    steer_ts = throttle_ts = brake_ts = 0.0

    while self._keep_alive:
      es = self.ego.simulator_state
      es.cruise_button = 0
      es.left_blinker = False
      es.right_blinker = False

      now = self.rk.frame / 100.0

      steer_manual, throttle_manual, brake_manual, steer_ts, throttle_ts, brake_ts = \
        self._drain_input_queue(q, es, now, steer_manual, throttle_manual, brake_manual,
                                steer_ts, throttle_ts, brake_ts)

      if not self._keep_alive:
        break

      # Decay stale manual inputs
      if now - steer_ts > _MANUAL_INPUT_TTL_SECS:
        steer_manual = 0.0
      if now - throttle_ts > _MANUAL_INPUT_TTL_SECS:
        throttle_manual = 0.0
      if now - brake_ts > _MANUAL_INPUT_TTL_SECS:
        brake_manual = 0.0

      steer_out, throttle_out, brake_out = self._update_ego(
        es, steer_manual, throttle_manual, brake_manual)
      self.world.apply_controls(steer_out, throttle_out, brake_out, {})

      if self.lead is not None:
        self._update_lead()

      self.world.read_state()
      self.world.read_sensors(es)

      if window is not None and self.rk.frame % _TELEMETRY_UPDATE_INTERVAL == 0:
        update_telemetry_from_sm(window, self.ego.car.sm if self.ego is not None else None,
                                 self.lead.car.sm if self.lead is not None else None)

      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      self.started.value = True
      self.rk.keep_time()

    self._cleanup(threads, window)

  def _start_role_threads(self, role: _RoleStack, camera_fn) -> list[threading.Thread]:
    """Start the CAN-update and camera-push threads for one role. Returns the threads."""
    car_thread = threading.Thread(
      target=rk_loop,
      args=(functools.partial(role.car.update, role.simulator_state), 100, self._exit_event))
    cam_thread = threading.Thread(
      target=rk_loop,
      args=(functools.partial(camera_fn, self.world), 20, self._exit_event))
    car_thread.start()
    cam_thread.start()
    return [car_thread, cam_thread]

  def _drain_input_queue(self, q: Queue, es: SimulatorState, now: float,
                         steer: float, throttle: float, brake: float,
                         steer_ts: float, throttle_ts: float, brake_ts: float,
                         ) -> tuple[float, float, float, float, float, float]:
    """Read all pending messages from the input queue, updating manual inputs and sim state."""
    while True:
      try:
        message = q.get_nowait()
      except pyqueue.Empty:
        break
      if message.type != QueueMessageType.CONTROL_COMMAND:
        continue

      command, value = message.info.split('_', 1)
      if command == "steer":
        steer = float(value)
        steer_ts = now
      elif command == "throttle":
        throttle = float(value)
        throttle_ts = now
      elif command == "brake":
        brake = float(value)
        brake_ts = now
      elif command == "cruise":
        es.cruise_button = _CRUISE_COMMANDS.get(value, 0)
      elif command == "blinker":
        es.left_blinker = (value == "left")
        es.right_blinker = (value == "right")
      elif command == "ignition":
        es.ignition = not es.ignition
      elif command == "reset":
        self.world.reset()
      elif command == "quit":
        self._keep_alive = False
        break

    return steer, throttle, brake, steer_ts, throttle_ts, brake_ts

  def _update_ego(self, es: SimulatorState,
                  steer_manual: float, throttle_manual: float, brake_manual: float,
                  ) -> tuple[float, float, float]:
    """Push sensor data to the ego device, read its carControl, and return
    (steer, throttle, brake) to apply to MetaDrive."""
    # Wheel/pedals only steer the ego — keep the lead clean so its on-device
    # hCCC pedal-blend doesn't pick up phantom driver inputs.
    es.user_brake = brake_manual
    es.user_gas = throttle_manual
    es.user_torque = steer_manual * _MANUAL_STEER_TO_TORQUE

    steer_angle = steer_manual * _MANUAL_STEER_TO_ANGLE

    self.ego.sensors.update(es, self.world)
    self.ego.car.sm.update(0)
    es.is_engaged = self.ego.car.sm['selfdriveState'].active

    if es.is_engaged:
      accel = self.ego.car.sm['carControl'].actuators.accel
      throttle_op, brake_op = _accel_to_controls(accel)
      self.past_startup_engaged = True
      return steer_angle, throttle_op, brake_op

    if not self.past_startup_engaged and self.ego.car.sm['selfdriveState'].engageable:
      es.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN
      self.startup_button_prev = not self.startup_button_prev

    return steer_angle, throttle_manual, brake_manual

  def _update_lead(self) -> None:
    """Ship sensors to the lead device and keep its sim state current.

    By default the MetaDrive worker drives the lead vehicle along the CSV
    speed profile (matching the field deployment, where the lead car is
    human-driven and its device only measures + publishes V2V). With
    lead_device_drive, the lead device's carControl is applied instead —
    note the worker currently ships lead_status=False to the lead role, so
    the lead's on-device hCCC has no radar target and will command zero
    until a virtual target is added.
    """
    self.lead.car.sm.update(0)
    ls = self.lead.simulator_state
    ls.is_engaged = self.lead.car.sm['selfdriveState'].active

    if self.lead_device_drive:
      if ls.is_engaged:
        accel = self.lead.car.sm['carControl'].actuators.accel
        lead_throttle, lead_brake = _accel_to_controls(accel)
        lead_steer = 0.0
      else:
        lead_throttle = lead_brake = lead_steer = 0.0
      # Sending any control permanently switches the worker off the CSV
      # profile path (lead_external_action is sticky) — only do so in
      # device-drive mode.
      self.world.apply_lead_controls(lead_steer, lead_throttle, lead_brake)

    self.lead.sensors.update(ls, self.world)
    self.world.read_lead_sensors(ls)

  def _cleanup(self, threads: list[threading.Thread], window) -> None:
    """Signal all worker threads to stop, close device connections, tear down window."""
    self._exit_event.set()
    for t in threads:
      t.join(timeout=1.0)
    if window is not None:
      window.stop()
    if self.ego is not None:
      self.ego.close()
    if self.lead is not None:
      self.lead.close()
