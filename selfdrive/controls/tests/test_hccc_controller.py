from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.hccc_controller import hCCC

# HCCC_CHANGE_NOTE: regression coverage for core hCCC lead/no-lead accel behavior.

def _cs(v_ego):
  return SimpleNamespace(vEgo=v_ego)


def _lead(status, v_rel, a_lead_k=None):
  return SimpleNamespace(status=status, vRel=v_rel, aLeadK=a_lead_k)


class BeamNGReferenceHCCC:
  def __init__(self, dt=0.1, beta=0.65):
    self.dt = dt
    self.beta = beta
    self.prev_lead_speed = None
    self.ff_y_prev = 0.0

  def step(self, ego_speed, lead_speed):
    if self.prev_lead_speed is not None:
      lead_accel = (lead_speed - self.prev_lead_speed) / self.dt
    else:
      lead_accel = 0.0
    self.prev_lead_speed = lead_speed

    self.ff_y_prev = self.ff_y_prev + self.dt * ((1.0 - self.beta) * lead_accel - self.ff_y_prev)
    return (self.beta * (lead_speed - ego_speed) + self.ff_y_prev) * 0.6


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


def test_hccc_matches_beamng_reference_sequence():
  controller = hCCC(dt=0.1)
  reference = BeamNGReferenceHCCC(dt=0.1)
  ego_speed = 20.0
  lead_speeds = [18.0, 18.5, 19.0, 20.0, 21.5, 21.0]

  for lead_speed in lead_speeds:
    accel = controller.run_step(_cs(ego_speed), _lead(True, lead_speed - ego_speed, a_lead_k=-99.0))
    expected = reference.step(ego_speed, lead_speed)
    assert accel is not None
    assert abs(accel - expected) < 1e-9


def test_hccc_ignores_radar_lead_accel_when_speed_history_is_available():
  controller = hCCC(dt=0.1)
  controller.run_step(_cs(20.0), _lead(True, 0.0, a_lead_k=5.0))
  accel = controller.run_step(_cs(20.0), _lead(True, 1.0, a_lead_k=-20.0))
  assert accel is not None
  assert accel > 0.0
