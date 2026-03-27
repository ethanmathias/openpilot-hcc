from types import SimpleNamespace

import openpilot.selfdrive.controls.radard as radard_mod


class _FakeTrack:
  def __init__(self, identifier: int, d_rel: float):
    self.identifier = identifier
    self.dRel = d_rel

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": 0.0,
      "vRel": 0.0,
      "vLead": 0.0,
      "vLeadK": 0.0,
      "aLeadK": 0.0,
      "aLeadTau": 1.5,
      "status": True,
      "fcw": False,
      "modelProb": float(model_prob),
      "radar": True,
      "radarTrackId": self.identifier,
    }


def _lead(prob: float = 0.9):
  # Deliberately inconsistent with the fake radar track so non-sim matching
  # falls back to vision, while sim mode can still force trust in liveTracks.
  return SimpleNamespace(
    prob=prob,
    x=[100.0],
    xStd=[0.1],
    y=[10.0],
    yStd=[0.1],
    v=[50.0],
    vStd=[0.1],
    a=[0.0],
  )


def test_get_lead_in_simulation_always_prefers_forward_track():
  original_simulation = radard_mod.SIMULATION
  radard_mod.SIMULATION = True
  try:
    lead = radard_mod.get_lead(
      v_ego=5.0,
      ready=True,
      tracks={0: _FakeTrack(0, 20.0)},
      lead_msg=_lead(),
      model_v_ego=5.0,
    )
  finally:
    radard_mod.SIMULATION = original_simulation

  assert lead["status"] is True
  assert lead["radar"] is True
  assert lead["radarTrackId"] == 0
  assert abs(lead["dRel"] - 20.0) < 1e-6


def test_get_lead_outside_simulation_can_still_fall_back_to_vision():
  original_simulation = radard_mod.SIMULATION
  radard_mod.SIMULATION = False
  try:
    lead = radard_mod.get_lead(
      v_ego=5.0,
      ready=True,
      tracks={0: _FakeTrack(0, 20.0)},
      lead_msg=_lead(),
      model_v_ego=5.0,
    )
  finally:
    radard_mod.SIMULATION = original_simulation

  assert lead["status"] is True
  assert lead["radar"] is False
  assert lead["radarTrackId"] == -1
