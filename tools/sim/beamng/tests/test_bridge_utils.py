from pathlib import Path

import pytest

adapter = pytest.importorskip("openpilot.tools.sim.beamng.hc3_adapter")


def test_normalize_hc3_output_supports_scalar_and_tuple():
  assert adapter.normalize_hc3_output(0.4) == pytest.approx(0.4)
  assert adapter.normalize_hc3_output((0.7, 0.2)) == pytest.approx(0.5)


def test_blend_manual_and_hc3_matches_reference_logic():
  blended = adapter.blend_manual_and_hc3(0.8, 0.1, -0.6)
  assert blended["manual_cmd"] == pytest.approx(0.7)
  assert blended["hc3_cmd"] == pytest.approx(-0.6)
  assert blended["final_cmd"] == pytest.approx(0.1)
  assert blended["throttle_out"] == pytest.approx(0.1)
  assert blended["brake_out"] == pytest.approx(0.0)


def test_reference_blender_holds_command_between_updates():
  class DummyController:
    def __init__(self):
      self.outputs = iter([0.5, -0.2])

    def run_step(self):
      return next(self.outputs)

  blender = adapter.ReferenceLongitudinalBlender(adapter.HC3ControllerAdapter(DummyController()), update_dt=0.1, sim_dt=0.02)

  first = blender.step(0.3, 0.0)
  for _ in range(3):
    held = blender.step(0.3, 0.0)
  refreshed = blender.step(0.3, 0.0)

  assert first.final_accel == pytest.approx(0.8)
  assert held.final_accel == pytest.approx(0.8)
  assert refreshed.final_accel == pytest.approx(0.1)


def test_create_hc3_controller_loads_custom_module(tmp_path: Path):
  controller_path = tmp_path / "dummy_controller.py"
  controller_path.write_text(
    "class hCCC:\n"
    "  def __init__(self, ego_vehicle, lead_vehicle, dt=0.1):\n"
    "    self.dt = dt\n"
    "  def run_step(self):\n"
    "    return (0.6, 0.1)\n",
    encoding="utf-8",
  )

  controller = adapter.create_hc3_controller("new", object(), object(), controller_path=str(controller_path))
  assert controller.step() == pytest.approx(0.5)
