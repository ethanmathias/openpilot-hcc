from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.controls.lib.hcc_v2v import V2VLeadSignal
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, long_control_state_trans




class TestLongControlStateTransition:

  def test_stay_stopped(self):
    CP = car.CarParams.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

def test_engage():
  CP = car.CarParams.new_message()
  active = True
  current_state = LongCtrlState.off
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid

def test_starting():
  CP = car.CarParams.new_message(startingState=True, vEgoStarting=0.5)
  active = True
  current_state = LongCtrlState.starting
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.starting
  next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid


def _test_cp(enable_hccc=False):
  return SimpleNamespace(
    enableHCCC=enable_hccc,
    stopAccel=-3.0,
    startAccel=1.6,
    startingState=False,
    vEgoStarting=0.5,
    stoppingDecelRate=0.8,
  )


def _test_cs(v_ego=20.0, gas=0.0, brake=0.0, gas_pressed=False, brake_pressed=False):
  return SimpleNamespace(
    vEgo=v_ego,
    gas=gas,
    brake=brake,
    gasPressed=gas_pressed,
    brakePressed=brake_pressed,
    cruiseState=SimpleNamespace(standstill=False),
  )


def _test_lead(status=True, v_rel=0.0, radar=True):
  return SimpleNamespace(status=status, vRel=v_rel, radar=radar)


def _test_v2v(status=True, v_lead=25.0, a_lead=1.0, seq=1):
  return V2VLeadSignal(
    status=status,
    lead_speed_mps=v_lead,
    lead_accel_mps2=a_lead,
    seq=seq,
    local_receive_valid=status,
    sender_timestamp_valid=status,
  )


def test_update_uses_only_manual_input_without_valid_lead():
  controller = LongControl(_test_cp(enable_hccc=False))
  output = controller.update(True, _test_cs(gas=0.4), a_target=0.3, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=None)
  assert abs(output - 0.4) < 1e-6


def test_update_prefers_hccc_with_valid_lead_and_blends_manual_input():
  controller = LongControl(_test_cp(enable_hccc=True))
  output = controller.update(True, _test_cs(gas=0.2), a_target=0.8, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
  assert output > 0.2
  assert output <= 2.0


def test_update_recomputes_hccc_each_control_tick():
  controller = LongControl(_test_cp(enable_hccc=True))
  outputs = iter([1.2, 0.7])
  run_step_calls = 0

  def fake_run_step(CS, lead, v2v_lead=None):
    nonlocal run_step_calls
    if lead is None or not lead.status:
      return None
    run_step_calls += 1
    return next(outputs)

  controller.hccc.run_step = fake_run_step

  first = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                            accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=True))
  refreshed = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                                accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=True))

  assert abs(first - 1.2) < 1e-6
  assert abs(refreshed - 0.7) < 1e-6
  assert run_step_calls == 2


def test_update_treats_non_radar_lead_like_any_other_valid_lead():
  controller = LongControl(_test_cp(enable_hccc=True))
  controller.hccc.run_step = lambda CS, lead, v2v_lead=None: 0.9 if lead is not None and lead.status else None

  output = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=False))

  assert abs(output - 0.9) < 1e-6
  assert controller.debug_hccc_active is True


def test_v2v_longitudinal_gate_rejects_invalid_v2v_signal():
  controller = LongControl(_test_cp(enable_hccc=True))
  controller.v2v_config = SimpleNamespace(enabled=True)
  controller.v2v_subscriber = SimpleNamespace(snapshot=lambda: _test_v2v(status=False))

  assert controller.v2v_enabled() is True
  assert controller.v2v_longitudinal_ok(_test_v2v(status=False)) is False


def test_update_uses_v2v_signal_when_enabled():
  controller = LongControl(_test_cp(enable_hccc=True))
  controller.v2v_config = SimpleNamespace(enabled=True)
  controller.v2v_subscriber = SimpleNamespace(snapshot=lambda: _test_v2v(status=True, v_lead=28.0, a_lead=1.6))
  controller.hccc.run_step = lambda CS, lead, v2v_lead=None: 1.1 if v2v_lead is not None and v2v_lead.status else None

  output = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0), v2v_lead=_test_v2v(status=True))

  assert abs(output - 1.1) < 1e-6
