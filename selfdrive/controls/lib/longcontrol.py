import os

import numpy as np
from cereal import car

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.hccc_controller import HCCC
from openpilot.selfdrive.controls.lib.hcc_v2v import ROLE_EGO, V2VLeadSignal, V2VLeadSubscriber, load_v2v_config
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
SIMULATION = os.environ.get("SIMULATION", "0") == "1"

SIM_PEDAL_GAS_GAIN = 1.4
SIM_PEDAL_BRAKE_GAIN = 4.0
ROAD_PEDAL_GAS_GAIN = 0.4
ROAD_PEDAL_BRAKE_GAIN = 1.2

V2V_MODE_MANUAL_ONLY = "manual_only"
V2V_MODE_WAITING = "v2v_waiting"
V2V_MODE_ACTIVE = "v2v_active"
V2V_MODE_FAULT_STALE = "v2v_fault_stale"
V2V_MODE_FAULT_TRANSPORT = "v2v_fault_transport"


def _parse_bool_env(value: str | None) -> bool | None:
  if value is None:
    return None
  normalized = value.strip().lower()
  if normalized in ("1", "true", "yes", "on"):
    return True
  if normalized in ("0", "false", "no", "off"):
    return False
  return None


def _normalize_pedal(value: float) -> float:
  v = float(value)
  if v > 1.0 and v <= 100.0:
    v *= 0.01
  return float(np.clip(v, 0.0, 1.0))


def _manual_longitudinal_input(CS, simulation_mode: bool = SIMULATION) -> float:
  gas_raw = getattr(CS, "gas", 0.0)
  brake_raw = getattr(CS, "brake", 0.0)

  gas = _normalize_pedal(gas_raw)
  brake = _normalize_pedal(brake_raw)

  manual_cmd = float(np.clip(gas - brake, -1.0, 1.0))
  gas_gain = SIM_PEDAL_GAS_GAIN if simulation_mode else ROAD_PEDAL_GAS_GAIN
  brake_gain = SIM_PEDAL_BRAKE_GAIN if simulation_mode else ROAD_PEDAL_BRAKE_GAIN
  if manual_cmd >= 0.0:
    return manual_cmd * gas_gain
  return manual_cmd * brake_gain


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.last_output_accel = 0.0
    self.params = Params()
    self.use_hccc = False
    self.hccc = None
    self.v2v_config = load_v2v_config(ROLE_EGO, default_enabled=False, default_device_id="hcc-ego")
    self.v2v_subscriber = V2VLeadSubscriber(self.v2v_config) if self.v2v_config.enabled else None
    if self.v2v_subscriber is not None:
      self.v2v_subscriber.start()
    self.latest_v2v_signal = V2VLeadSignal(status=False)
    self.debug_planner_accel = 0.0
    self.debug_v2v_mode = V2V_MODE_MANUAL_ONLY
    self.debug_v2v_transport_ok = True
    self.debug_v2v_receive_age_ms = float("inf")
    self._reset_debug_state()
    self._refresh_hccc(force_reset=True)

  def _reset_debug_state(self):
    self.debug_hccc_accel = 0.0
    self.debug_manual_accel = 0.0
    self.debug_output_accel = 0.0
    self.debug_hccc_active = False
    self.debug_hccc_lead_speed = 0.0
    self.debug_hccc_lead_accel = 0.0
    self.debug_hccc_feedforward = 0.0

  def reset(self):
    self._reset_debug_state()
    self._refresh_hccc(force_reset=True)

  def _hccc_enabled(self):
    cp_flag = getattr(self.CP, "enableHCCC", None)
    if cp_flag is not None:
      return cp_flag
    if not self.params.check_key("EnableHCCC"):
      return SIMULATION
    return self.params.get_bool("EnableHCCC")

  def _v2v_only_enabled(self):
    env_enabled = _parse_bool_env(os.environ.get("HCC_V2V_ONLY"))
    if env_enabled is not None:
      return env_enabled
    return self.params.check_key("HCCV2VOnly") and self.params.get_bool("HCCV2VOnly")

  def _refresh_hccc(self, force_reset=False):
    enabled = self._hccc_enabled()
    if not enabled:
      self.hccc = None
    elif self.hccc is None or force_reset:
      self.hccc = HCCC(dt=DT_CTRL, max_deceleration=self.CP.stopAccel, max_acceleration=max(self.CP.startAccel, 1.6))
    self.use_hccc = enabled

  def _set_v2v_mode(self, mode: str, signal: V2VLeadSignal, transport_ok: bool):
    self.debug_v2v_mode = mode
    self.debug_v2v_transport_ok = transport_ok
    self.debug_v2v_receive_age_ms = float(signal.receive_age_ms)
    if getattr(self, "_last_logged_v2v_mode", None) != mode:
      cloudlog.info(
        "hcc_v2v_mode_transition mode=%s transport_ok=%s signal_status=%s receive_age_ms=%.2f seq=%d",
        mode, transport_ok, signal.status, float(signal.receive_age_ms), int(signal.seq),
      )
      self._last_logged_v2v_mode = mode

  def _determine_v2v_mode(self, active: bool, signal: V2VLeadSignal, transport_ok: bool) -> str:
    if not self._v2v_only_enabled() or not self.v2v_enabled() or not self.use_hccc:
      return V2V_MODE_MANUAL_ONLY
    if not transport_ok:
      return V2V_MODE_FAULT_TRANSPORT
    if signal.status and active:
      return V2V_MODE_ACTIVE
    if signal.seq < 0:
      return V2V_MODE_WAITING
    if signal.status:
      return V2V_MODE_WAITING
    return V2V_MODE_FAULT_STALE

  def v2v_enabled(self) -> bool:
    return self.v2v_subscriber is not None and self.v2v_config.enabled

  def v2v_only_enabled(self) -> bool:
    return self._v2v_only_enabled()

  def v2v_snapshot(self) -> V2VLeadSignal:
    if not self.v2v_enabled():
      self.latest_v2v_signal = V2VLeadSignal(status=False)
    else:
      self.latest_v2v_signal = self.v2v_subscriber.snapshot()
    return self.latest_v2v_signal

  def v2v_longitudinal_ok(self, v2v_lead: V2VLeadSignal | None = None) -> bool:
    if not self.v2v_enabled():
      return True
    signal = self.v2v_snapshot() if v2v_lead is None else v2v_lead
    self.latest_v2v_signal = signal
    return bool(signal.status)

  def update(self, active, CS, a_target, should_stop, accel_limits, lead=None, v2v_lead: V2VLeadSignal | None = None):
    self._refresh_hccc()
    accel_min, accel_max = accel_limits
    self.debug_planner_accel = float(a_target)

    active_v2v_lead = self.latest_v2v_signal if v2v_lead is None else v2v_lead
    transport_ok = True
    if self.v2v_enabled():
      active_v2v_lead = self.v2v_snapshot() if v2v_lead is None else v2v_lead
      self.latest_v2v_signal = active_v2v_lead
      transport_ok = bool(self.v2v_subscriber.transport_ok()) if self.v2v_subscriber is not None else True

    controller_accel = 0.0
    hccc_output = None
    if self.use_hccc and self.hccc is not None:
      self.hccc.set_accel_limits(accel_min, accel_max)
      radar_lead = None if self._v2v_only_enabled() else lead
      v2v_input = active_v2v_lead if self.v2v_enabled() else None
      hccc_output = self.hccc.run_step(CS, radar_lead, v2v_lead=v2v_input)
      if hccc_output is not None:
        controller_accel = float(hccc_output)

    manual_accel = _manual_longitudinal_input(CS)
    self.debug_hccc_accel = float(controller_accel)
    self.debug_manual_accel = float(manual_accel)
    self.debug_hccc_active = bool(active and hccc_output is not None)
    if self.hccc is not None:
      self.debug_hccc_lead_speed = float(getattr(self.hccc, "debug_lead_speed", 0.0))
      self.debug_hccc_lead_accel = float(getattr(self.hccc, "debug_lead_accel", 0.0))
      self.debug_hccc_feedforward = float(getattr(self.hccc, "debug_feedforward", 0.0))

    self._set_v2v_mode(self._determine_v2v_mode(active, active_v2v_lead, transport_ok), active_v2v_lead, transport_ok)

    if hccc_output is not None:
      should_stop = False

    if self._v2v_only_enabled() and active:
      self.long_control_state = LongCtrlState.pid
    else:
      self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                         should_stop, CS.brakePressed,
                                                         CS.cruiseState.standstill)
      if active and hccc_output is not None:
        self.long_control_state = LongCtrlState.pid

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:
      output_accel = controller_accel + manual_accel

    self.last_output_accel = np.clip(output_accel, accel_min, accel_max)
    self.debug_output_accel = float(self.last_output_accel)
    return self.last_output_accel
