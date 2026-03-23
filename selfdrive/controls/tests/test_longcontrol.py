from types import SimpleNamespace

from cereal import car
import openpilot.selfdrive.controls.lib.longcontrol as longcontrol_mod
from openpilot.selfdrive.controls.lib.longcontrol import HCCC_UPDATE_STEPS, LongControl, LongCtrlState, long_control_state_trans




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


def _test_lead(status=True, v_rel=0.0):
  return SimpleNamespace(status=status, vRel=v_rel, radar=True)


def test_update_uses_only_manual_input_without_valid_lead():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=False))
  output = controller.update(True, _test_cs(gas=0.4), a_target=0.3, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=None)
  assert abs(output - 0.4) < 1e-6


def test_update_prefers_hccc_with_valid_lead_and_blends_manual_input():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=True))
  output = controller.update(True, _test_cs(gas=0.2), a_target=0.8, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
  assert output > 0.2
  assert output <= 2.0


def test_update_disables_manual_input_outside_simulation():
  longcontrol_mod.SIMULATION = False
  controller = LongControl(_test_cp(enable_hccc=False))
  output = controller.update(True, _test_cs(gas=0.4), a_target=0.3, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=None)
  assert output == 0.0


def test_hccc_updates_at_beamng_10hz_and_holds_between_updates():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=True))
  outputs = iter([0.5, 0.8])
  call_count = {"count": 0}

  def fake_run_step(CS, lead):
    call_count["count"] += 1
    return next(outputs)

  controller.hccc.run_step = fake_run_step

  first = controller.update(True, _test_cs(gas=0.2), a_target=0.8, should_stop=False,
                            accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
  assert abs(first - 0.7) < 1e-6

  for _ in range(HCCC_UPDATE_STEPS - 1):
    held = controller.update(True, _test_cs(gas=0.2), a_target=0.2, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
    assert abs(held - 0.7) < 1e-6

  refreshed = controller.update(True, _test_cs(gas=0.2), a_target=0.2, should_stop=False,
                                accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
  assert abs(refreshed - 1.0) < 1e-6
  assert call_count["count"] == 2
  assert abs(controller.debug_hccc_accel - 0.8) < 1e-6
  assert abs(controller.debug_manual_accel - 0.2) < 1e-6
  assert abs(controller.debug_output_accel - 1.0) < 1e-6


def test_invalid_lead_resets_hccc_hold_state():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=True))
  call_count = {"count": 0}

  def fake_run_step(CS, lead):
    call_count["count"] += 1
    return 0.5

  controller.hccc.run_step = fake_run_step

  controller.update(True, _test_cs(gas=0.0), a_target=0.0, should_stop=False,
                    accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))
  invalid = controller.update(True, _test_cs(gas=0.1), a_target=0.0, should_stop=False,
                              accel_limits=(-3.0, 2.0), lead=_test_lead(False, 0.0))
  resumed = controller.update(True, _test_cs(gas=0.0), a_target=0.0, should_stop=False,
                              accel_limits=(-3.0, 2.0), lead=_test_lead(True, 5.0))

  assert abs(invalid - 0.1) < 1e-6
  assert abs(resumed - 0.5) < 1e-6
  assert call_count["count"] == 2


def test_simulation_hccc_braking_is_not_canceled_by_positive_manual_accel():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=True))
  controller.hccc.run_step = lambda CS, lead: -0.8

  output = controller.update(True, _test_cs(gas=1.0), a_target=0.0, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=_test_lead(True, -2.0))

  assert abs(output - (-0.8)) < 1e-6
  assert abs(controller.debug_hccc_accel - (-0.8)) < 1e-6
  assert abs(controller.debug_manual_accel - 0.0) < 1e-6


def test_simulation_hccc_ignores_vision_fallback_leads():
  longcontrol_mod.SIMULATION = True
  controller = LongControl(_test_cp(enable_hccc=True))
  call_count = {"count": 0}

  def fake_run_step(CS, lead):
    call_count["count"] += 1
    return 0.8

  controller.hccc.run_step = fake_run_step

  vision_fallback_lead = SimpleNamespace(status=True, vRel=5.0, radar=False)
  output = controller.update(True, _test_cs(gas=0.1), a_target=0.3, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=vision_fallback_lead)

  assert abs(output - 0.1) < 1e-6
  assert call_count["count"] == 0
  assert controller.debug_hccc_active is False


def test_non_simulation_hccc_still_accepts_non_radar_leads():
  longcontrol_mod.SIMULATION = False
  controller = LongControl(_test_cp(enable_hccc=True))
  call_count = {"count": 0}

  def fake_run_step(CS, lead):
    call_count["count"] += 1
    return 0.8

  controller.hccc.run_step = fake_run_step

  non_radar_lead = SimpleNamespace(status=True, vRel=5.0, radar=False)
  output = controller.update(True, _test_cs(gas=0.1), a_target=0.3, should_stop=False,
                             accel_limits=(-3.0, 2.0), lead=non_radar_lead)

  assert abs(output - 0.8) < 1e-6
  assert call_count["count"] == 1
