from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.hccc_controller import hCCC

# HCCC_CHANGE_NOTE: regression coverage for core hCCC lead/no-lead accel behavior.

def _cs(v_ego):
  return SimpleNamespace(vEgo=v_ego)


def _lead(status, v_rel):
  return SimpleNamespace(status=status, vRel=v_rel)


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
