from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.hcc_v2v import V2VLeadSignal
from openpilot.selfdrive.controls.lib.hccc_controller import HCCC, PIDLongitudinal

# HCCC_CHANGE_NOTE: regression coverage for core HCCC lead/no-lead accel behavior
# and the cascade PID speed-tracking stage (BeamNG hCCC_controller_latest port).

def _cs(v_ego):
  return SimpleNamespace(vEgo=v_ego)


def _lead(status, v_rel, a_lead_k=None):
  return SimpleNamespace(status=status, vRel=v_rel, aLeadK=a_lead_k)


def test_hccc_returns_none_without_valid_lead():
  controller = HCCC(dt=0.1)
  assert controller.run_step(_cs(20.0), None) is None
  assert controller.run_step(_cs(20.0), _lead(False, -2.0)) is None


def test_hccc_decelerates_for_slower_lead():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(_cs(20.0), _lead(True, -5.0))
  assert accel is not None
  assert accel < 0.0


def test_hccc_accelerates_for_faster_lead():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(_cs(20.0), _lead(True, 5.0))
  assert accel is not None
  assert accel > 0.0


def test_hccc_uses_filtered_lead_acceleration_when_available():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(_cs(20.0), _lead(True, 0.0, a_lead_k=2.0))
  assert accel is not None
  assert accel > 0.0


def test_hccc_uses_v2v_speed_and_accel_when_present():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(
    _cs(20.0),
    None,
    v2v_lead=V2VLeadSignal(status=True, lead_speed_mps=24.0, lead_accel_mps2=1.5, seq=1, local_receive_valid=True, sender_timestamp_valid=True),
  )
  assert accel is not None
  assert accel > 0.0


def test_hccc_does_not_fallback_to_radar_when_v2v_signal_is_invalid():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(
    _cs(20.0),
    _lead(True, 5.0, a_lead_k=2.0),
    v2v_lead=V2VLeadSignal(status=False),
  )
  assert accel is None


def _v2v(v, a):
  return V2VLeadSignal(status=True, lead_speed_mps=v, lead_accel_mps2=a, seq=1,
                       local_receive_valid=True, sender_timestamp_valid=True)


def test_pid_matches_beamng_reference_semantics_at_reference_dt():
  # First step pure P; second step P + I over both errors + per-step D of 0.2.
  pid = PIDLongitudinal(k_p=0.35, k_i=0.05, k_d_s=0.02, dt=0.1, i_limit=20.0)
  first = pid.run_step(21.0, 20.0)   # error 1.0, v_des outside the attenuation band
  assert abs(first - 0.35) < 1e-12
  second = pid.run_step(21.5, 20.0)  # error 1.5
  expected = 0.35 * 1.5 + 0.2 * (1.5 - 1.0) + 0.05 * (1.0 + 1.5) * 0.1
  assert abs(second - expected) < 1e-12


def test_pid_integral_is_clamped():
  pid = PIDLongitudinal(k_p=0.35, k_i=0.05, k_d_s=0.02, dt=0.01, i_limit=20.0)
  for _ in range(100000):
    pid.run_step(50.0, 0.0)
  assert pid._integral == 20.0


def test_pid_low_speed_attenuation():
  pid_low = PIDLongitudinal(k_p=0.35, k_i=0.0, k_d_s=0.0, dt=0.01, i_limit=20.0)
  pid_high = PIDLongitudinal(k_p=0.35, k_i=0.0, k_d_s=0.0, dt=0.01, i_limit=20.0)
  low = pid_low.run_step(5.0, 4.0)
  high = pid_high.run_step(15.0, 14.0)
  assert abs(low - high * 5.0 / 10.0) < 1e-12


def test_pid_derivative_is_dt_invariant():
  # Same continuous error ramp sampled at 10 Hz and 100 Hz must give ~equal output.
  p10 = PIDLongitudinal(k_p=0.35, k_i=0.05, k_d_s=0.02, dt=0.1, i_limit=20.0)
  p100 = PIDLongitudinal(k_p=0.35, k_i=0.05, k_d_s=0.02, dt=0.01, i_limit=20.0)
  out10 = [p10.run_step(20.0 + 0.5 * (i * 0.1), 20.0) for i in range(1, 21)]
  out100 = [p100.run_step(20.0 + 0.5 * (i * 0.01), 20.0) for i in range(1, 201)]
  assert abs(out10[-1] - out100[-1]) < 0.01


def test_hccc_integral_grows_command_under_persistent_error():
  controller = HCCC(dt=0.1)
  first = controller.run_step(_cs(20.0), None, v2v_lead=_v2v(22.0, 0.0))
  out = first
  for _ in range(48):
    out = controller.run_step(_cs(20.0), None, v2v_lead=_v2v(22.0, 0.0))
  assert first is not None and out is not None
  assert out > first > 0.0


def test_hccc_standstill_with_stopped_lead_commands_no_reverse():
  controller = HCCC(dt=0.1)
  accel = controller.run_step(_cs(0.0), None, v2v_lead=_v2v(0.0, -2.0))
  assert accel is not None
  assert accel >= 0.0  # v_des is clamped at 0; never chases a negative speed


def test_hccc_reset_clears_pid_state():
  controller = HCCC(dt=0.1)
  controller.run_step(_cs(8.0), None, v2v_lead=_v2v(9.0, 0.5))
  controller.reset()
  assert controller._pid._integral == 0.0
  assert controller._pid._prev_error is None
