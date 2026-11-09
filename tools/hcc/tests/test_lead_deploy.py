"""The Raspberry Pi lead deployment contract.

The lead vehicle carries a Raspberry Pi, not a comma device and not openpilot.
It runs a ~100 KB bundle built by device/deploy_lead.sh. Three things have to
stay true for that to keep working, and all three are easy to break by
accident from inside the repo:

  1. The bundle's file list still covers everything the publisher imports.
  2. Nothing in the bundle needs openpilot, or tomllib, or devices.toml.
  3. The orchestrator passes the Pi every value it cannot look up itself.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

from openpilot.tools.hcc.common import REPO_ROOT, load_settings
from openpilot.tools.hcc.fieldtest.remote import remote_python

DEPLOY_SH = REPO_ROOT / "tools/hcc/device/deploy_lead.sh"


def _deployed_files() -> list[str]:
  """The FILES=( ... ) list out of deploy_lead.sh."""
  text = DEPLOY_SH.read_text()
  block = re.search(r"^FILES=\((.*?)^\)", text, re.S | re.M)
  assert block, "FILES=( ... ) not found in deploy_lead.sh"
  return [line.strip() for line in block.group(1).splitlines() if line.strip()]


def _openpilot_imports(path: Path) -> set[str]:
  tree = ast.parse(path.read_text())
  modules: set[str] = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("openpilot."):
      modules.add(node.module)
    elif isinstance(node, ast.Import):
      modules.update(a.name for a in node.names if a.name.startswith("openpilot."))
  return modules


def _module_to_path(module: str) -> str:
  return module[len("openpilot."):].replace(".", "/") + ".py"


# ---- 1. the file list covers the import closure ---------------------------

def test_deploy_list_is_closed_under_imports():
  """Every openpilot.* import of a shipped file is itself shipped.

  Without this, adding an import to virtual_lead.py silently breaks the Pi:
  the bundle deploys fine and dies at launch.
  """
  shipped = set(_deployed_files())
  python_files = [f for f in shipped if f.endswith(".py")]
  assert python_files, "no Python files in the deploy list"

  missing = {}
  for relative in python_files:
    for module in _openpilot_imports(REPO_ROOT / relative):
      target = _module_to_path(module)
      # common is imported defensively inside a function, never at module level
      if target == "tools/hcc/common.py":
        continue
      if target not in shipped:
        missing.setdefault(relative, set()).add(target)
  assert not missing, f"deploy_lead.sh FILES is missing imports: {missing}"


def test_deployed_files_all_exist():
  for relative in _deployed_files():
    assert (REPO_ROOT / relative).is_file(), f"deploy_lead.sh lists a missing file: {relative}"


def test_scenario_table_is_shipped():
  # The publisher is useless without it, and it no longer lives in tools/sim.
  assert "tools/hcc/config/scenarios.csv" in _deployed_files()


# ---- 2. the bundle needs nothing a Pi might not have ----------------------

def _imported_names(path: Path) -> set[str]:
  """Top-level module names actually imported, ignoring prose."""
  tree = ast.parse(path.read_text())
  names: set[str] = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      names.update(a.name.split(".")[0] for a in node.names)
    elif isinstance(node, ast.ImportFrom) and node.module:
      names.add(node.module.split(".")[0])
  return names


@pytest.mark.parametrize("forbidden", ["tomllib", "numpy", "cereal"])
def test_bundle_avoids_dependencies_a_pi_may_not_have(forbidden):
  """tomllib is 3.11+; numpy and cereal are not installed on a bare Pi.

  Checked against the parsed imports, not the file text, so documentation
  that merely mentions them does not trip it.
  """
  for relative in _deployed_files():
    if relative.endswith(".py"):
      assert forbidden not in _imported_names(REPO_ROOT / relative), \
          f"{relative} imports {forbidden}"


def test_shipped_files_use_postponed_annotations():
  """`X | None` in annotations needs this below 3.10."""
  for relative in _deployed_files():
    if relative.endswith(".py"):
      src = (REPO_ROOT / relative).read_text()
      assert "from __future__ import annotations" in src, f"{relative} lacks postponed annotations"


def test_shipped_files_have_no_match_statements():
  """match/case is 3.10+."""
  for relative in _deployed_files():
    if relative.endswith(".py"):
      tree = ast.parse((REPO_ROOT / relative).read_text())
      assert not any(isinstance(n, ast.Match) for n in ast.walk(tree)), f"{relative} uses match/case"


def test_publisher_defaults_survive_without_devices_toml(monkeypatch):
  """_defaults() must not raise when common or its config is unavailable.

  This is the deployed case: no devices.toml, maybe no tomllib.
  """
  import openpilot.tools.hcc.publishers.virtual_lead as vl

  def explode(*args, **kwargs):
    raise ImportError("simulated deployed lead: no common module")

  monkeypatch.setattr(vl, "_defaults", vl._defaults)  # keep the real one
  monkeypatch.setitem(__import__("sys").modules, "openpilot.tools.hcc.common", None)
  defaults = vl._defaults()

  assert defaults["relay_host"] == "10.42.0.1"   # NetworkManager shared-mode host
  assert defaults["relay_port"] == 19090
  assert defaults["device_id"] == "hcc-lead"
  assert defaults["hz"] == 50.0
  assert defaults["csv"].endswith("scenarios.csv")


def test_standalone_defaults_point_at_the_hotspot():
  from openpilot.tools.hcc.publishers.virtual_lead import STANDALONE_DEFAULTS
  assert STANDALONE_DEFAULTS["relay_host"] == "10.42.0.1"
  assert STANDALONE_DEFAULTS["device_id"] == "hcc-lead"


# ---- 3. the orchestrator tells the Pi everything --------------------------

def test_run_passes_the_relay_to_the_lead_explicitly():
  """A deployed Pi has no devices.toml, so these cannot be left to default."""
  from openpilot.tools.hcc.fieldtest.session import RunOptions, _virtual_lead_flags

  settings = load_settings()
  flags = _virtual_lead_flags(settings, RunOptions(scenario=48), "/tmp/sent.csv")
  for expected in (f"--relay_host {settings.relay_host}",
                   f"--relay_port {settings.relay_port}",
                   f"--device_id {settings.lead.device_id}",
                   "--scn 48"):
    assert expected in flags, f"missing {expected!r} in {flags!r}"


def test_pinned_interpreter_skips_the_probe():
  # Returns the override without opening an SSH connection; if it probed, this
  # would try to reach a host that does not exist and fail the test.
  assert remote_python("pi", "198.51.100.7", "/home/pi/hcc-lead", "python3") == "python3"


def test_lead_config_is_a_pi_not_a_comma_device():
  lead = load_settings().lead
  assert lead.python, "[lead] python should be pinned for a Pi"
  assert "openpilot" not in lead.repo, "[lead] repo should be the deploy bundle, not a checkout"
  assert lead.log_dir and not lead.log_dir.startswith("/data/"), \
      "[lead] log_dir should be a Pi path, not the comma /data tree"


def test_ego_still_probes_for_its_interpreter():
  # The comma device's venv is not on a non-interactive PATH, so it must probe.
  assert load_settings().ego.python == ""
