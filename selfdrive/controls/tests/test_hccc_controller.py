from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.hccc_controller import hCCC

# HCCC_CHANGE_NOTE: regression coverage for core hCCC lead/no-lead accel behavior.

def _cs(v_ego):
  return SimpleNamespace(vEgo=v_ego)


def _lead(status, v_rel, v_lead=None):
  lead_speed = (20.0 + v_rel) if v_lead is None else v_lead
  return SimpleNamespace(status=status, vRel=v_rel, vLead=lead_speed)


def test_hccc_returns_none_without_valid_lead():
  controller = hCCC(dt=0.1)
  assert controller.run_step(_cs(20.0), None) is None
  assert controller.run_step(_cs(20.0), _lead(False, -2.0)) is None


def test_hccc_decelerates_for_slower_lead():
  controller = hCCC(dt=0.1)
  accel = controller.run_step(_cs(20.0), _lead(True, -5.0))
  assert accel is not None
  assert accel < 0.0


def test_hccc_accelerates_for_faster_lead():
  controller = hCCC(dt=0.1)
  accel = controller.run_step(_cs(20.0), _lead(True, 5.0))
  assert accel is not None
  assert accel > 0.0


def test_hccc_uses_actual_elapsed_time_between_lead_updates():
  controller = hCCC(dt=0.1)
  controller.run_step(_cs(20.0), _lead(True, 0.0, v_lead=20.0), lead_time_ns=1_000_000_000)
  accel = controller.run_step(_cs(20.0), _lead(True, 1.0, v_lead=21.0), lead_time_ns=2_000_000_000)
  assert accel is not None
  assert 0.35 < accel < 0.5
