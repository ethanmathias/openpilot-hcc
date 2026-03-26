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
HCCC_UPDATE_FRAMES = max(1, int(round(HCCC_UPDATE_DT / DT_CTRL)))
HCCC_TRACK_LOSS_HOLD_S = 0.3
HCCC_TRACK_LOSS_HOLD_FRAMES = max(1, int(round(HCCC_TRACK_LOSS_HOLD_S / DT_CTRL)))

# HCCC_CHANGE_NOTE: longcontrol uses BeamNG-style cooperative blending:
# controller contribution + signed manual pedal input, blended once in the shared
# longitudinal path so it works for both simulation and real-car execution.

def _normalize_pedal(value: float) -> float:
  v = float(value)
  if v > 1.0 and v <= 100.0:
    v *= 0.01
  return float(np.clip(v, 0.0, 1.0))


def _manual_longitudinal_input(CS) -> float:
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

  # NOTE: This maps manual pedal input into the simulator's accel-command space
  # so the existing bridge conversion reproduces BeamNG-like pedal authority.
  # This should be revisited for the real-car version, where actuators.accel is
  # not simply converted back into throttle/brake with the simulator scaling.
  if manual_cmd >= 0.0:
    return manual_cmd * 1.4
  return manual_cmd * 4.0


def _lead_is_track_backed(lead) -> bool:
  if lead is None or not bool(getattr(lead, "status", False)):
    return False
  return bool(getattr(lead, "radar", True))


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
    # Replay/debug telemetry for simulator analysis. These stay on LongControl
    # so the bridge can log the exact planner/HC3/manual contributions.
    self.debug_planner_accel = 0.0
    self.debug_hccc_accel = 0.0
    self.debug_manual_accel = 0.0
    self.debug_output_accel = 0.0
    self.debug_hccc_active = False
    self.debug_hccc_lead_speed = 0.0
    self.debug_hccc_lead_accel = 0.0
    self.debug_hccc_feedforward = 0.0
    self._reset_hccc_runtime_state()
    self._refresh_hccc(force_reset=True)

  def reset(self):
    self.debug_hccc_accel = 0.0
    self.debug_manual_accel = 0.0
    self.debug_output_accel = 0.0
    self.debug_hccc_active = False
    self.debug_hccc_lead_speed = 0.0
    self.debug_hccc_lead_accel = 0.0
    self.debug_hccc_feedforward = 0.0
    self._reset_hccc_runtime_state()
    self._refresh_hccc(force_reset=True)

  def _hccc_enabled(self):
    # HCCC_CHANGE_NOTE: enable from CarParams override or persistent EnableHCCC toggle.
    cp_flag = getattr(self.CP, 'enableHCCC', None)
    if cp_flag is not None:
      return cp_flag
    if not self.params.check_key("EnableHCCC"):
      return SIMULATION
    return self.params.get_bool("EnableHCCC")

  def _reset_hccc_runtime_state(self):
    # Hold the last HC3 command between 10 Hz updates so the control output
    # does not chatter at controlsd's 100 Hz loop rate.
    self._held_hccc_accel = 0.0
    self._has_held_hccc_output = False
    self._hccc_frames_since_update = HCCC_UPDATE_FRAMES
    # Keep the last valid HC3 command briefly across radar/vision handoffs so
    # short fusion mismatches do not create hard 0 -> X -> 0 accel cliffs.
    self._hccc_track_loss_frames = 0

  def _refresh_hccc(self, force_reset=False):
    enabled = self._hccc_enabled()
    if not enabled:
      self.hccc = None
      self._reset_hccc_runtime_state()
    elif self.hccc is None or force_reset:
      # BeamNG HC3 logic is a 10 Hz controller. We run it at that cadence and
      # hold the latest output between updates in the 100 Hz longitudinal loop.
      self.hccc = hCCC(dt=HCCC_UPDATE_DT, max_deceleration=self.CP.stopAccel, max_acceleration=max(self.CP.startAccel, 1.6))
      self._reset_hccc_runtime_state()
    self.use_hccc = enabled

  def update(self, active, CS, a_target, should_stop, accel_limits, lead=None):
    """Update longitudinal control. This updates the state machine and runs hCCC."""
    self._refresh_hccc()
    accel_min, accel_max = accel_limits
    self.debug_planner_accel = float(a_target)

    controller_accel = 0.0
    hccc_output = None
    if self.use_hccc and self.hccc is not None:
      self.hccc._max_decel = accel_min
      self.hccc._max_accl = accel_max
      lead_status = lead is not None and bool(getattr(lead, "status", False))
      track_backed_lead = lead_status and _lead_is_track_backed(lead)

      if track_backed_lead or (lead_status and not SIMULATION):
        self._hccc_track_loss_frames = 0
        if (not self._has_held_hccc_output) or (self._hccc_frames_since_update >= HCCC_UPDATE_FRAMES):
          hccc_output = self.hccc.run_step(CS, lead)
          if hccc_output is not None:
            self._held_hccc_accel = float(hccc_output)
            self._has_held_hccc_output = True
            self._hccc_frames_since_update = 1
        elif self._has_held_hccc_output:
          hccc_output = self._held_hccc_accel
          self._hccc_frames_since_update += 1

        if hccc_output is not None:
          controller_accel = float(hccc_output)
      elif SIMULATION and lead_status and self._has_held_hccc_output and self._hccc_track_loss_frames < HCCC_TRACK_LOSS_HOLD_FRAMES:
        # Preserve the last command for a short grace window when radard
        # momentarily falls back from a track-backed lead to vision only.
        self._hccc_track_loss_frames += 1
        self._hccc_frames_since_update = min(self._hccc_frames_since_update + 1, HCCC_UPDATE_FRAMES)
        hccc_output = self._held_hccc_accel
        controller_accel = float(hccc_output)
      else:
        # Fully reset once the lead is gone or has stayed non-track-backed
        # longer than the grace window.
        self.hccc.run_step(CS, None)
        self._reset_hccc_runtime_state()

    # HCCC_CHANGE_NOTE: BeamNG-style cooperative input is a single signed manual command.
    manual_accel = _manual_longitudinal_input(CS)
    self.debug_hccc_accel = float(controller_accel)
    self.debug_manual_accel = float(manual_accel)
    self.debug_hccc_active = bool(active and hccc_output is not None)
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
