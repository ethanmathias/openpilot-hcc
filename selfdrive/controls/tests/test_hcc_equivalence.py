"""Equivalence with hcc-ego: the rebuilt control path must command the same thing.

ego_fall26 was rebuilt from the upstream base rather than branched off
hcc-ego, so nothing guarantees it behaves identically -- it has to be shown.
This file is that proof, and it is a regression lock: the Fall-26 runs are
meant to be comparable with the 2026-07-11 phase-1 data, which they are not if
the controller's output changed on the way.

hcc_ego_decision() below is transcribed from longcontrol.py at cee4e5061. The
subject side uses the real HCCResult policy, and the real upstream
long_control_state_trans (identical on both branches, verified by hash) is
exec'd from source rather than paraphrased.

Two divergences were found and fixed this way:

  1. The force-pid rule had lost hcc-ego's `active` guard, so a DISENGAGED
     controller forced LongCtrlState.pid and emitted its command instead of 0
     -- 1152 of 6144 states.
  2. controlsd gated CC.longActive on V2V health in V2V-only mode, where
     hcc-ego deliberately does not: staleness is handled inside the state
     machine so the standstill release still runs (field bug 5). That one is
     covered by test_v2v_only_does_not_gate_long_active below.
"""
from __future__ import annotations

import itertools
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.hcc.blend import pedal_to_accel
from openpilot.selfdrive.controls.lib.hcc.state import HCCResult
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import V2VLeadSignal

DT_CTRL = 0.01
REPO_ROOT = Path(__file__).resolve().parents[3]


def _repo_file(relative: str) -> str:
  """Read a source file from the working tree, not from git.

  The point is to assert on the code that would actually run, including
  uncommitted edits.
  """
  return (REPO_ROOT / relative).read_text()


class LongCtrlState:
  off, pid, stopping, starting = "off", "pid", "stopping", "starting"


def _load_trans():
  """The real upstream transition function, exec'd with a stand-in enum.

  Avoids importing longcontrol, which pulls in cereal and a compiled Params.
  """
  src = _repo_file("selfdrive/controls/lib/longcontrol.py")
  body = src[src.index("def long_control_state_trans"):src.index("class LongControl")]
  namespace = {"LongCtrlState": LongCtrlState}
  exec(compile(body, "<long_control_state_trans>", "exec"), namespace)
  return namespace["long_control_state_trans"]


trans = _load_trans()


class CP:
  vEgoStarting = 0.1
  vEgoStopping = 0.1
  startingState = True
  startAccel = 0.3
  stopAccel = -2.0
  stoppingDecelRate = 0.3


def _output(state, last_output, controller, manual, a_min, a_max):
  """The output-selection ladder, identical on both branches."""
  if state == LongCtrlState.off:
    out = 0.0
  elif state == LongCtrlState.stopping:
    out = last_output
    if out > CP.stopAccel:
      out = min(out, 0.0) - CP.stoppingDecelRate * DT_CTRL
  elif state == LongCtrlState.starting:
    out = CP.startAccel
  else:
    out = controller + manual
  return float(np.clip(out, a_min, a_max))


def hcc_ego_decision(s):
  """Reference: hcc-ego's LongControl.update(), transcribed from cee4e5061."""
  should_stop = s["should_stop"]
  hccc_output = s["hccc_output"]
  if hccc_output is not None:
    should_stop = False

  if s["v2v_only"]:
    v_target = s["lead_speed"] if hccc_output is not None else 0.0
    v2v_should_stop = (v_target < 0.5) and (s["v_ego"] < CP.vEgoStopping)
    state = trans(CP, s["active"], s["state"], s["v_ego"], v2v_should_stop,
                  s["brake_pressed"], s["cruise_standstill"])
  else:
    state = trans(CP, s["active"], s["state"], s["v_ego"], should_stop,
                  s["brake_pressed"], s["cruise_standstill"])
    if s["active"] and hccc_output is not None:
      state = LongCtrlState.pid

  controller = hccc_output if hccc_output is not None else 0.0
  return state, _output(state, s["last_output"], controller, s["manual"], s["a_min"], s["a_max"])


def fall26_decision(s):
  """Subject: the current HCCResult policy, as longcontrol applies it."""
  hccc_output = s["hccc_output"]
  result = HCCResult(
    enabled=True,
    controller_active=hccc_output is not None,
    controller_accel=hccc_output if hccc_output is not None else 0.0,
    manual_accel=s["manual"],
    accel_limits=(s["a_min"], s["a_max"]),
    v2v_only=s["v2v_only"],
    v2v_lead=V2VLeadSignal(status=True, lead_speed_mps=s["lead_speed"]),
  )
  should_stop = result.resolve_should_stop(s["should_stop"], s["v_ego"], CP.vEgoStopping)
  state = trans(CP, s["active"], s["state"], s["v_ego"], should_stop,
                s["brake_pressed"], s["cruise_standstill"])
  if result.force_pid(s["active"]):
    state = LongCtrlState.pid
  return state, _output(state, s["last_output"], result.controller_accel, result.manual_accel,
                        s["a_min"], s["a_max"])


def _sweep():
  all_states = [LongCtrlState.off, LongCtrlState.pid, LongCtrlState.stopping, LongCtrlState.starting]
  for active, v2v_only, hccc, v_ego, ss, bp, cs, lead, st in itertools.product(
      [True, False], [True, False], [None, 1.5, -1.2, 0.0], [0.0, 0.05, 0.4, 15.0],
      [True, False], [True, False], [True, False], [0.0, 0.3, 20.0], all_states):
    yield {"active": active, "v2v_only": v2v_only, "hccc_output": hccc, "v_ego": v_ego,
           "should_stop": ss, "brake_pressed": bp, "cruise_standstill": cs,
           "lead_speed": lead, "state": st, "manual": 0.0, "last_output": 0.5,
           "a_min": -3.5, "a_max": 2.0}


def test_control_decision_matches_hcc_ego_across_the_state_space():
  mismatches = [(s, hcc_ego_decision(s), fall26_decision(s))
                for s in _sweep() if hcc_ego_decision(s) != fall26_decision(s)]
  assert not mismatches, (
    f"{len(mismatches)} of {sum(1 for _ in _sweep())} states diverge from hcc-ego. "
    f"First: {mismatches[0]}")


def test_the_sweep_is_actually_large():
  # Guards against a refactor silently shrinking the grid to nothing.
  assert sum(1 for _ in _sweep()) >= 6000


def test_disengaged_controller_commands_nothing():
  # The specific regression: this was 1152 of the 6144 mismatches.
  result = HCCResult(enabled=True, controller_active=True, controller_accel=1.5, v2v_only=False)
  assert result.force_pid(active=False) is False
  assert result.force_pid(active=True) is True


def test_force_pid_never_fires_in_v2v_only():
  # V2V-only runs the real state machine so the standstill handshake happens.
  result = HCCResult(enabled=True, controller_active=True, v2v_only=True)
  assert result.force_pid(active=True) is False


def test_force_pid_needs_a_live_controller_command():
  result = HCCResult(enabled=True, controller_active=False, v2v_only=False)
  assert result.force_pid(active=True) is False


def test_v2v_only_does_not_gate_long_active():
  """controlsd must not withdraw longActive on lead staleness in V2V-only mode.

  Asserted on the source, because the alternative is importing controlsd.
  Dropping longActive there would reset the controller and skip the standstill
  release that field bug 5 needed.
  """
  src = _repo_file("selfdrive/controls/controlsd.py")
  assert "longitudinal_input_ok = True if self.LoC.hcc.v2v_only()" in src
  assert "longitudinal_input_ok" in src.split("CC.longActive =")[1].split("\n\n")[0]


@pytest.mark.parametrize("gas,brake", [
  (0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (0.5, 0.5), (0.3, 0.8),
  (50.0, 0.0), (0.0, 100.0), (-5.0, 0.0), (200.0, 0.0), (0.25, 0.75),
])
def test_manual_term_matches_hcc_ego(gas, brake):
  """hcc-ego's _manual_longitudinal_input, on the road (non-simulation) path."""
  def reference(g, b):
    g = float(np.clip(g * 0.01 if 1.0 < g <= 100.0 else g, 0.0, 1.0))
    b = float(np.clip(b * 0.01 if 1.0 < b <= 100.0 else b, 0.0, 1.0))
    cmd = float(np.clip(g - b, -1.0, 1.0))
    return cmd * (0.4 if cmd >= 0.0 else 1.2)

  assert pedal_to_accel(gas, brake) == pytest.approx(reference(gas, brake), abs=1e-12)
