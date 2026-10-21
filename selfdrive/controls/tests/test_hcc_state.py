"""Mode and stop-state policy tests."""
import pytest

from openpilot.selfdrive.controls.lib.hcc.state import (
  MODE_ACTIVE,
  MODE_FAULT_STALE,
  MODE_FAULT_TRANSPORT,
  MODE_MANUAL_ONLY,
  MODE_OFF,
  MODE_WAITING,
  V2V_STOP_SPEED_MPS,
  HCCResult,
  determine_mode,
)
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import V2VLeadSignal


def _signal(status=True, seq=10, speed=20.0):
  return V2VLeadSignal(status=status, seq=seq, lead_speed_mps=speed)


def _mode(hccc=True, v2v=True, only=True, active=True, signal=None, transport=True):
  return determine_mode(hccc, v2v, only, active, signal if signal is not None else _signal(), transport)


# ---- mode machine --------------------------------------------------------

def test_disabled_controller_is_off_regardless_of_link_health():
  assert _mode(hccc=False) == MODE_OFF
  assert _mode(hccc=False, transport=False) == MODE_OFF


def test_manual_only_when_not_v2v_only_or_link_unused():
  assert _mode(only=False) == MODE_MANUAL_ONLY
  assert _mode(v2v=False) == MODE_MANUAL_ONLY


def test_transport_fault_outranks_signal_state():
  # A dead socket is a different problem from a stale packet and must be
  # distinguishable in the log.
  assert _mode(transport=False) == MODE_FAULT_TRANSPORT
  assert _mode(transport=False, signal=_signal(status=False)) == MODE_FAULT_TRANSPORT


def test_active_requires_both_a_healthy_signal_and_engagement():
  assert _mode(active=True, signal=_signal(status=True)) == MODE_ACTIVE
  assert _mode(active=False, signal=_signal(status=True)) == MODE_WAITING


def test_nothing_received_yet_is_waiting_not_a_fault():
  # seq < 0 means no packet has ever arrived. That is startup, not a dropout.
  assert _mode(signal=_signal(status=False, seq=-1)) == MODE_WAITING


def test_stale_after_having_received_is_a_fault():
  assert _mode(signal=_signal(status=False, seq=10)) == MODE_FAULT_STALE


# ---- should_stop resolution ---------------------------------------------

def test_non_v2v_live_controller_means_not_stopping():
  result = HCCResult(enabled=True, controller_active=True, v2v_only=False)
  assert result.resolve_should_stop(True, v_ego=10.0, v_ego_stopping=0.25) is False


def test_non_v2v_without_a_controller_command_defers_to_upstream():
  result = HCCResult(enabled=True, controller_active=False, v2v_only=False)
  assert result.resolve_should_stop(True, 10.0, 0.25) is True
  assert result.resolve_should_stop(False, 10.0, 0.25) is False


def test_v2v_only_stops_when_lead_is_stopped_and_ego_is_slow():
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True,
                     v2v_lead=_signal(speed=0.0))
  assert result.resolve_should_stop(False, v_ego=0.1, v_ego_stopping=0.25) is True


def test_v2v_only_does_not_stop_while_still_rolling():
  # Running the real state machine matters for the standstill handshake, but
  # it must not trigger a stop while the car is still moving.
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True,
                     v2v_lead=_signal(speed=0.0))
  assert result.resolve_should_stop(False, v_ego=5.0, v_ego_stopping=0.25) is False


def test_v2v_only_does_not_stop_for_a_moving_lead():
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True,
                     v2v_lead=_signal(speed=20.0))
  assert result.resolve_should_stop(False, v_ego=0.1, v_ego_stopping=0.25) is False


def test_v2v_only_stale_signal_at_standstill_holds_the_stop():
  # With no controller command the target is treated as 0, so a stale link at
  # a standstill holds the stop rather than creeping.
  result = HCCResult(enabled=True, controller_active=False, v2v_only=True,
                     v2v_lead=_signal(status=False, speed=20.0))
  assert result.resolve_should_stop(False, v_ego=0.1, v_ego_stopping=0.25) is True


def test_stop_speed_threshold_boundary():
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True,
                     v2v_lead=_signal(speed=V2V_STOP_SPEED_MPS))
  assert result.resolve_should_stop(False, 0.1, 0.25) is False
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True,
                     v2v_lead=_signal(speed=V2V_STOP_SPEED_MPS - 0.01))
  assert result.resolve_should_stop(False, 0.1, 0.25) is True


# ---- output ---------------------------------------------------------------

def test_output_accel_is_the_sum_of_controller_and_manual():
  result = HCCResult(controller_accel=1.5, manual_accel=-0.4)
  assert result.output_accel == pytest.approx(1.1)


def test_default_result_is_inert():
  result = HCCResult()
  assert not result.enabled
  assert not result.controller_active
  assert result.output_accel == 0.0
  assert result.mode == MODE_OFF
  assert not result.v2v_lead.status
