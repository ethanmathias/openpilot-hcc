"""Hardware-in-the-loop variant of MetaDriveBridge.

Each Comma 3X runs the full openpilot stack on-device. The PC's job is just
to be the world: spawn MetaDrive vehicles, ship synthesized camera frames +
CAN/sensor inputs to each device over USB-RNDIS, and apply the device's
returned `carControl` to the corresponding MetaDrive vehicle.

M3 scope: ego device only. M4 adds the lead device + on-ego V2V relay; the
two-prefix port-collision concern documented in cereal_bridges.py applies
only at M4.
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
    self.bridge_pair = cereal_bridges.spawn(prefix, device.ip)
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
               output_csv=None, output_graph=None):
    super().__init__(dual_camera=dual_camera, high_quality=high_quality, scenario=scenario,
                     enable_hcc=True, scn=scn, scn_csv=scn_csv, output_csv=output_csv,
                     output_graph=output_graph, lead_sim_prefix=None,
                     hil_two_vehicle=lead_device is not None)
    self.ego_device = ego_device
    self.lead_device = lead_device
    self.raw_yuv = raw_yuv
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
    threads = []
    car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.ego.car.update, self.ego.simulator_state), 100, self._exit_event))
    car_thread.start()
    threads.append(car_thread)
    cam_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.ego.sensors.send_camera_images, self.world), 20, self._exit_event))
    cam_thread.start()
    threads.append(cam_thread)

    if self.lead is not None:
      lead_car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.lead.car.update, self.lead.simulator_state), 100, self._exit_event))
      lead_car_thread.start()
      threads.append(lead_car_thread)
      lead_cam_thread = threading.Thread(target=rk_loop, args=(functools.partial(_send_lead_camera, self.lead.sensors, self.world), 20, self._exit_event))
      lead_cam_thread.start()
      threads.append(lead_cam_thread)

    for _ in range(20):
      self.world.tick()

    throttle_manual = steer_manual = brake_manual = 0.0
    manual_ttl = 0.35
    steer_ts = throttle_ts = brake_ts = 0.0

    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = brake_op = 0.0

      es = self.ego.simulator_state
      es.cruise_button = 0
      es.left_blinker = False
      es.right_blinker = False

      now = self.rk.frame / 100.0

      while True:
        try:
          message = q.get_nowait()
        except pyqueue.Empty:
          break
        if message.type != QueueMessageType.CONTROL_COMMAND:
          continue
        m = message.info.split('_')
        if m[0] == "steer":
          steer_manual = float(m[1])
          steer_ts = now
        elif m[0] == "throttle":
          throttle_manual = float(m[1])
          throttle_ts = now
        elif m[0] == "brake":
          brake_manual = float(m[1])
          brake_ts = now
        elif m[0] == "cruise":
          es.cruise_button = {
            "down": CruiseButtons.DECEL_SET,
            "up": CruiseButtons.RES_ACCEL,
            "cancel": CruiseButtons.CANCEL,
            "main": CruiseButtons.MAIN,
          }.get(m[1], 0)
        elif m[0] == "blinker":
          es.left_blinker = (m[1] == "left")
          es.right_blinker = (m[1] == "right")
        elif m[0] == "ignition":
          es.ignition = not es.ignition
        elif m[0] == "reset":
          self.world.reset()
        elif m[0] == "quit":
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

      # Wheel/pedals only steer the ego — keep the lead clean so its on-device
      # hCCC pedal-blend doesn't pick up phantom driver inputs.
      es.user_brake = brake_manual
      es.user_gas = throttle_manual
      es.user_torque = steer_manual * -10000

      steer_actuator_input = steer_manual * -40

      self.ego.sensors.update(es, self.world)
      self.ego.car.sm.update(0)
      es.is_engaged = self.ego.car.sm['selfdriveState'].active

      if es.is_engaged:
        throttle_op = float(np.clip(self.ego.car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0))
        brake_op = float(np.clip(-self.ego.car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0))
        self.past_startup_engaged = True
      elif not self.past_startup_engaged and self.ego.car.sm['selfdriveState'].engageable:
        es.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN
        self.startup_button_prev = not self.startup_button_prev

      if es.is_engaged:
        throttle_out = throttle_op
        brake_out = brake_op
        steer_out = steer_actuator_input
      else:
        throttle_out = throttle_manual
        brake_out = brake_manual
        steer_out = steer_actuator_input

      self.world.apply_controls(steer_out, throttle_out, brake_out, {})

      # Lead device: read its carControl, derive throttle/brake/steer, push to MetaDrive's
      # second vehicle. Lead has no human pedal input — purely on-device hCCC + planner.
      if self.lead is not None:
        self.lead.car.sm.update(0)
        ls = self.lead.simulator_state
        ls.is_engaged = self.lead.car.sm['selfdriveState'].active
        if ls.is_engaged:
          lead_throttle = float(np.clip(self.lead.car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0))
          lead_brake = float(np.clip(-self.lead.car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0))
          lead_steer = 0.0
        else:
          lead_throttle = lead_brake = lead_steer = 0.0
        self.world.apply_lead_controls(lead_steer, lead_throttle, lead_brake)
        self.lead.sensors.update(ls, self.world)
        self.world.read_lead_sensors(ls)

      self.world.read_state()
      self.world.read_sensors(es)

      if window is not None and self.rk.frame % 10 == 0:
        update_telemetry_from_sm(window, self.ego.car.sm if self.ego is not None else None,
                                 self.lead.car.sm if self.lead is not None else None)

      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      self.started.value = True
      self.rk.keep_time()

    self._exit_event.set()
    for t in threads:
      t.join(timeout=1.0)
    if window is not None:
      window.stop()
    if self.ego is not None:
      self.ego.close()
    if self.lead is not None:
      self.lead.close()
