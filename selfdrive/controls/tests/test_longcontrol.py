from types import SimpleNamespace

from cereal import car
import openpilot.selfdrive.controls.lib.longcontrol as longcontrol_mod
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


def test_update_ignores_vision_fallback_leads_in_simulation():
  original_simulation = longcontrol_mod.SIMULATION
  longcontrol_mod.SIMULATION = True
  try:
    controller = LongControl(_test_cp(enable_hccc=True))
    outputs = iter([1.2, 0.7])

    def fake_run_step(CS, lead):
      if lead is None or not lead.status:
        return None
      return next(outputs)

    controller.hccc.run_step = fake_run_step

    first = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                              accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=True))
    # Simulation HC3 should ignore a radar=False lead rather than treating the
    # vision fallback as a valid controller input.
    ignored = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                                accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=False))
    refreshed = controller.update(True, _test_cs(gas=0.0), a_target=0.8, should_stop=False,
                                  accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0, radar=True))

    assert abs(first - 1.2) < 1e-6
    assert abs(ignored - 0.0) < 1e-6
    assert abs(refreshed - 0.7) < 1e-6
  finally:
    longcontrol_mod.SIMULATION = original_simulation
