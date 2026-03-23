import csv
from types import SimpleNamespace

import pytest

common = pytest.importorskip("openpilot.tools.sim.bridge.common")
metadrive_process = pytest.importorskip("openpilot.tools.sim.bridge.metadrive.metadrive_process")
accel_to_pedal_commands = common.accel_to_pedal_commands


def test_accel_to_pedal_commands_uses_beamng_sign_split_for_hc3():
  throttle, brake = accel_to_pedal_commands(0.75, True)
  assert throttle == 0.75
  assert brake == 0.0

  throttle, brake = accel_to_pedal_commands(-0.4, True)
  assert throttle == 0.0
  assert brake == 0.4


def test_default_output_paths_use_repo_local_hccc_directories(monkeypatch, tmp_path):
  monkeypatch.setattr(metadrive_process, "_default_output_root", lambda: tmp_path)
  lead_cfg = SimpleNamespace(output_control_method="hccc", output_vehicle_name="honda_civic_2022", profile_scn=48)

  csv_path, graph_path = metadrive_process._default_output_paths(lead_cfg)

  assert str(tmp_path / "data" / "hccc") in csv_path
  assert str(tmp_path / "graphs" / "hccc") in graph_path
  assert ".scn48.hccc.csv" in csv_path
  assert ".scn48.hccc.png" in graph_path


def test_open_output_csv_writes_beamng_parity_columns(tmp_path):
  csv_path = tmp_path / "bridge.csv"
  output_file, _, resolved_path = metadrive_process._open_output_csv(str(csv_path))
  output_file.close()

  with open(resolved_path, newline="") as csv_file:
    reader = csv.reader(csv_file)
    header = next(reader)

  assert header == metadrive_process.OUTPUT_CSV_COLUMNS
  assert "target_speed_pre[m/s]" in header
  assert "planner_a_target[m/s2]" in header
  assert "hccc_accel[m/s2]" in header
  assert "manual_accel[m/s2]" in header
  assert "final_accel_cmd[m/s2]" in header
