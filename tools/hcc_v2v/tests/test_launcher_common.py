from tools.hcc_v2v.launcher_common import (
  LauncherConfig,
  apple_script_string,
  build_bridge_command,
  build_ego_command,
  build_lead_command,
  build_relay_command,
)


def _config(**kwargs) -> LauncherConfig:
  base = LauncherConfig(
    ego_repo="/tmp/openpilot-hcc",
    lead_repo="/tmp/openpilot-hcc-lead",
  )
  values = base.__dict__.copy()
  values.update(kwargs)
  return LauncherConfig(**values)


def test_build_relay_command_includes_csv_logging():
  command = build_relay_command(_config(relay_log_csv="/tmp/relay.csv", relay_port=19091))

  assert "tools/hcc_v2v/relay_server.py" in command
  assert "--port 19091" in command
  assert "--log_csv /tmp/relay.csv" in command


def test_build_ego_command_exports_v2v_env():
  command = build_ego_command(_config(relay_host="10.0.0.8", relay_port=20001))

  assert "launch_openpilot_ego.sh" in command
  assert "HCC_V2V_RELAY_HOST=10.0.0.8" in command
  assert "HCC_V2V_RELAY_PORT=20001" in command
  assert "OPENPILOT_PREFIX=hccego" in command


def test_build_lead_command_exports_prefix_and_device():
  command = build_lead_command(_config(lead_prefix="leadtest", lead_device_id="lead-car"))

  assert "launch_openpilot_lead.sh" in command
  assert "OPENPILOT_PREFIX=leadtest" in command
  assert "HCC_V2V_DEVICE_ID=lead-car" in command


def test_build_bridge_command_uses_scenario_and_keyboard():
  command = build_bridge_command(_config(scenario="48", bridge_output_csv="/tmp/bridge.csv"))

  assert "./run_bridge.py --mode hc3 --lead_prefix hcclead --scn 48 --output_csv /tmp/bridge.csv --keyboard" in command


def test_apple_script_string_escapes_quotes():
  assert apple_script_string('echo "hello"') == '"echo \\"hello\\""'
