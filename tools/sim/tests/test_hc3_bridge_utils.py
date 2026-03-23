import csv
from types import SimpleNamespace

import pytest

common = pytest.importorskip("openpilot.tools.sim.bridge.common")
metadrive_bridge = pytest.importorskip("openpilot.tools.sim.bridge.metadrive.metadrive_bridge")
metadrive_process = pytest.importorskip("openpilot.tools.sim.bridge.metadrive.metadrive_process")
accel_to_pedal_commands = common.accel_to_pedal_commands
calibrated_hc3_pedal_commands = common.calibrated_hc3_pedal_commands


def test_accel_to_pedal_commands_keeps_legacy_non_hc3_scaling():
  throttle, brake = accel_to_pedal_commands(0.75, False)
  assert throttle == pytest.approx(0.46875)
  assert brake == 0.0

  throttle, brake = accel_to_pedal_commands(-0.4, False)
  assert throttle == 0.0
  assert brake == pytest.approx(0.1)


def test_calibrated_hc3_pedal_commands_applies_deadband_and_scaling():
  throttle, brake = calibrated_hc3_pedal_commands(0.04, 0.0, 0.0)
  assert throttle == 0.0
  assert brake == 0.0

  throttle, brake = calibrated_hc3_pedal_commands(1.6, 1.0, 0.0)
  assert throttle == 1.0
  assert brake == 0.0

  throttle, brake = calibrated_hc3_pedal_commands(-4.0, 0.0, 1.0)
  assert throttle == 0.0
  assert brake == 1.0


def test_calibrated_hc3_pedal_commands_slew_limits_changes():
  throttle, brake = calibrated_hc3_pedal_commands(1.6, 0.0, 0.0)
  assert throttle == pytest.approx(0.04)
  assert brake == 0.0

  throttle, brake = calibrated_hc3_pedal_commands(0.0, 0.4, 0.2)
  assert throttle == pytest.approx(0.32)
  assert brake == pytest.approx(0.1)


def test_replay_profile_road_length_uses_minimum_and_buffer():
  assert metadrive_bridge.replay_profile_road_length([0.0, 500.0]) == pytest.approx(2000.0)
  assert metadrive_bridge.replay_profile_road_length([10.0, 1810.0]) == pytest.approx(2800.0)


def test_create_straight_map_covers_requested_length():
  map_config = metadrive_bridge.create_straight_map(total_length=2800.0, block_length=1000.0)
  blocks = map_config["config"][1:]
  assert len(blocks) == 3
  assert sum(block["length"] for block in blocks) >= 2800.0


def test_hccc_parity_logger_matches_beamng_reference_and_holds_between_updates():
  logger = metadrive_process.HCCCParityLogger(dt=0.1, beta=0.65)

  first = logger.update(ego_speed_mps=20.0, lead_speed_mps=21.0, active=True, actual_hccc_cmd=0.65)
  assert first.lead_accel_mps2 == 0.0
  assert first.raw_command_mps2 == pytest.approx(0.65)
  assert first.command_error_mps2 == pytest.approx(0.0)

  held = logger.update(ego_speed_mps=20.0, lead_speed_mps=22.0, active=True, actual_hccc_cmd=0.65)
  assert held.raw_command_mps2 == pytest.approx(0.65)

  for _ in range(3):
    logger.update(ego_speed_mps=20.0, lead_speed_mps=22.0, active=True, actual_hccc_cmd=0.65)

  refreshed = logger.update(ego_speed_mps=20.0, lead_speed_mps=22.0, active=True, actual_hccc_cmd=1.35)
  assert refreshed.lead_accel_mps2 == pytest.approx(10.0)
  assert refreshed.feedforward_mps2 == pytest.approx(0.35)
  assert refreshed.raw_command_mps2 == pytest.approx(1.65)
  assert refreshed.command_error_mps2 == pytest.approx(-0.30)


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
  assert "hccc_input_v_ego[m/s]" in header
  assert "hccc_input_lead_is_radar" in header
  assert "hccc_input_lead_track_id" in header
  assert "hccc_input_lead_speed_est[m/s]" in header
  assert "hccc_reference_cmd[m/s2]" in header
  assert "hccc_reference_error[m/s2]" in header
  assert "ego_lane_id" in header
  assert "lead_lane_id" in header
  assert "lead_pose_fallback_active" in header
