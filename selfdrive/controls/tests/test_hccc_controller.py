from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.hcc_v2v import V2VLeadSignal
from openpilot.selfdrive.controls.lib.hccc_controller import HCCC

# HCCC_CHANGE_NOTE: regression coverage for core HCCC lead/no-lead accel behavior.

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
