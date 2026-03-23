import numpy as np
import os
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.hccc_controller import hCCC
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
SIMULATION = os.environ.get("SIMULATION", "0") == "1"
HCCC_UPDATE_DT = 0.1
HCCC_UPDATE_STEPS = max(1, int(round(HCCC_UPDATE_DT / DT_CTRL)))

# HCCC_CHANGE_NOTE: longcontrol uses BeamNG-style cooperative blending:
# controller contribution + signed manual pedal input, blended once in the shared
# longitudinal path so it works for both simulation and real-car execution.

def _normalize_pedal(value: float) -> float:
  v = float(value)
  if v > 1.0 and v <= 100.0:
    v *= 0.01
  return float(np.clip(v, 0.0, 1.0))


def _manual_longitudinal_input(CS) -> float:
  if not SIMULATION:
    return 0.0

  gas_raw = getattr(CS, "gas", 0.0)
  brake_raw = getattr(CS, "brake", 0.0)

  gas = _normalize_pedal(gas_raw)
  brake = _normalize_pedal(brake_raw)

  # Fallback for platforms that only expose boolean pressed flags.
  if getattr(CS, "gasPressed", False):
    gas = max(gas, 1.0)
  if getattr(CS, "brakePressed", False):
    brake = max(brake, 1.0)

  manual_cmd = float(np.clip(gas - brake, -1.0, 1.0))

  # BeamNG parity: in simulation the cooperative manual command is the raw
  # signed pedal delta before any additional scaling.
  return manual_cmd


def _hccc_lead_is_usable(lead) -> bool:
  if lead is None or not bool(getattr(lead, "status", False)):
    return False

  # In simulation, HC3 should only trust track-backed radarState leads. This
  # prevents radard's vision fallback from injecting non-physical relative speed
  # spikes into the BeamNG-parity controller while leaving on-road behavior
  # unchanged for comma hardware.
  if SIMULATION and not bool(getattr(lead, "radar", True)):
    return False

  return True


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
    self.hccc_output = None
    self._hccc_update_counter = 0
    self.debug_planner_accel = 0.0
    self.debug_hccc_accel = 0.0
    self.debug_manual_accel = 0.0
    self.debug_output_accel = 0.0
    self.debug_hccc_active = False
    self.debug_hccc_lead_speed = 0.0
    self.debug_hccc_lead_accel = 0.0
    self.debug_hccc_feedforward = 0.0
    self._refresh_hccc(force_reset=True)

  def reset(self):
    self._refresh_hccc(force_reset=True)
    self.hccc_output = None
    self._hccc_update_counter = 0
    self.debug_hccc_accel = 0.0
    self.debug_manual_accel = 0.0
    self.debug_output_accel = 0.0
    self.debug_hccc_active = False
    self.debug_hccc_lead_speed = 0.0
    self.debug_hccc_lead_accel = 0.0
    self.debug_hccc_feedforward = 0.0

  def _reset_hccc_state(self):
    self.hccc_output = None
    self._hccc_update_counter = 0
    if self.hccc is not None:
      self.hccc.reset()

  def _hccc_enabled(self):
    # HCCC_CHANGE_NOTE: enable from CarParams override or persistent EnableHCCC toggle.
    cp_flag = getattr(self.CP, 'enableHCCC', None)
    if cp_flag is not None:
      return cp_flag
    if not self.params.check_key("EnableHCCC"):
      return SIMULATION
    return self.params.get_bool("EnableHCCC")

  def _refresh_hccc(self, force_reset=False):
    enabled = self._hccc_enabled()
    if not enabled:
      self.hccc = None
    elif self.hccc is None or force_reset:
      self.hccc = hCCC(dt=HCCC_UPDATE_DT, max_deceleration=self.CP.stopAccel, max_acceleration=max(self.CP.startAccel, 1.6))
      self._reset_hccc_state()
    self.use_hccc = enabled

  def update(self, active, CS, a_target, should_stop, accel_limits, lead=None):
    """Update longitudinal control. This updates the state machine and runs hCCC."""
    self._refresh_hccc()
    accel_min, accel_max = accel_limits
    self.debug_planner_accel = float(a_target)

    controller_accel = 0.0
    lead_valid = _hccc_lead_is_usable(lead)
    hccc_output = None
    if self.use_hccc and self.hccc is not None and active and lead_valid:
      if self.hccc_output is None or self._hccc_update_counter <= 0:
        hccc_output = self.hccc.run_step(CS, lead)
        self.hccc_output = hccc_output
        self._hccc_update_counter = HCCC_UPDATE_STEPS - 1
      else:
        self._hccc_update_counter -= 1
        hccc_output = self.hccc_output

      if hccc_output is not None:
        controller_accel = hccc_output
    else:
      self._reset_hccc_state()

    # HCCC_CHANGE_NOTE: BeamNG-style cooperative input is a single signed manual command.
    manual_accel = _manual_longitudinal_input(CS)
    self.debug_hccc_accel = float(controller_accel)
    self.debug_manual_accel = float(manual_accel)
    self.debug_hccc_active = bool(self.use_hccc and active and lead_valid and self.hccc_output is not None)
    if self.hccc is not None:
      self.debug_hccc_lead_speed = float(getattr(self.hccc, "debug_lead_speed", 0.0))
      self.debug_hccc_lead_accel = float(getattr(self.hccc, "debug_lead_accel", 0.0))
      self.debug_hccc_feedforward = float(getattr(self.hccc, "debug_feedforward", 0.0))

    if hccc_output is not None:
      should_stop = False

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

    else:  # LongCtrlState.pid
      output_accel = controller_accel + manual_accel

    self.last_output_accel = np.clip(output_accel, accel_min, accel_max)
    self.debug_output_accel = float(self.last_output_accel)
    return self.last_output_accel
