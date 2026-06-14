from __future__ import annotations

import os
import platform
import shlex
from dataclasses import dataclass
from pathlib import Path


DEFAULT_LEAD_PREFIX = "hcclead"
DEFAULT_EGO_PREFIX = "hccego"
DEFAULT_RELAY_HOST = "127.0.0.1"
DEFAULT_RELAY_PORT = 19090
DEFAULT_EGO_DEVICE_ID = "ego-sim"
DEFAULT_LEAD_DEVICE_ID = "lead-sim"


@dataclass(frozen=True)
class LauncherConfig:
  ego_repo: str
  lead_repo: str
  relay_host: str = DEFAULT_RELAY_HOST
  relay_port: int = DEFAULT_RELAY_PORT
  relay_log_csv: str = ""
  bridge_output_csv: str = ""
  lead_prefix: str = DEFAULT_LEAD_PREFIX
  ego_prefix: str = DEFAULT_EGO_PREFIX
  ego_device_id: str = DEFAULT_EGO_DEVICE_ID
  lead_device_id: str = DEFAULT_LEAD_DEVICE_ID
  scenario: str = ""
  input_mode: str = "keyboard"


def shell_join(parts: list[str]) -> str:
  return " ".join(shlex.quote(part) for part in parts)


def _activate_prefix(repo_root: str) -> str:
  repo = Path(repo_root).expanduser().resolve()
  return f"cd {shlex.quote(str(repo))}; source .venv/bin/activate"


def _env_exports(env_map: dict[str, str | int]) -> str:
  exports = [f"export {key}={shlex.quote(str(value))}" for key, value in env_map.items()]
  return "; ".join(exports)


def validate_repo(repo_root: str) -> None:
  repo = Path(repo_root).expanduser()
  if not repo.is_dir():
    raise ValueError(f"repo does not exist: {repo}")
  activate = repo / ".venv" / "bin" / "activate"
  if not activate.is_file():
    raise ValueError(f"missing virtualenv activation script: {activate}")


def build_relay_command(config: LauncherConfig) -> str:
  cmd = [
    "python3",
    "tools/hcc_v2v/relay_server.py",
    "--host",
    config.relay_host,
    "--port",
    str(config.relay_port),
  ]
  if config.relay_log_csv.strip():
    cmd.extend(["--log_csv", config.relay_log_csv.strip()])
  return f"{_activate_prefix(config.ego_repo)}; {shell_join(cmd)}"


def build_ego_command(config: LauncherConfig) -> str:
  env_exports = _env_exports({
    "OPENPILOT_PREFIX": config.ego_prefix,
    "HCC_V2V_ENABLED": 1,
    "HCC_V2V_ONLY": 1,
    "HCC_V2V_DEVICE_ID": config.ego_device_id,
    "HCC_V2V_RELAY_HOST": config.relay_host,
    "HCC_V2V_RELAY_PORT": config.relay_port,
  })
  return f"{_activate_prefix(config.ego_repo)}; {env_exports}; ./tools/sim/launch_openpilot_ego.sh"


def build_lead_command(config: LauncherConfig) -> str:
  env_exports = _env_exports({
    "OPENPILOT_PREFIX": config.lead_prefix,
    "HCC_V2V_ENABLED": 1,
    "HCC_V2V_DEVICE_ID": config.lead_device_id,
    "HCC_V2V_RELAY_HOST": config.relay_host,
    "HCC_V2V_RELAY_PORT": config.relay_port,
  })
  return f"{_activate_prefix(config.lead_repo)}; {env_exports}; ./tools/sim/launch_openpilot_lead.sh"


def build_bridge_command(config: LauncherConfig) -> str:
  cmd = [
    "./run_bridge.py",
    "--mode",
    "hc3",
    "--lead_prefix",
    config.lead_prefix,
  ]

  scenario = config.scenario.strip()
  if scenario:
    cmd.extend(["--scn", scenario])

  output_csv = config.bridge_output_csv.strip()
  if output_csv:
    cmd.extend(["--output_csv", output_csv])

  input_mode = config.input_mode.strip()
  if input_mode == "joystick":
    cmd.append("--joystick")
  elif input_mode == "logitech_wheel":
    cmd.append("--logitech_wheel")
  else:
    cmd.append("--keyboard")

  repo = Path(config.ego_repo).expanduser().resolve()
  return f"cd {shlex.quote(str(repo / 'tools' / 'sim'))}; source ../../.venv/bin/activate; {shell_join(cmd)}"


def command_shell_lines(title: str, command: str) -> str:
  return f'clear; echo "{title}"; echo; {command}; status=$?; echo; echo "Process exited with code ${{status}}."; exec $SHELL -l'


def build_terminal_command(title: str, command: str) -> list[str]:
  shell_cmd = command_shell_lines(title, command)
  system = platform.system()

  if system == "Darwin":
    script = (
      'tell application "Terminal"\n'
      '  activate\n'
      f'  do script {apple_script_string(shell_cmd)}\n'
      'end tell\n'
    )
    return ["osascript", "-e", script]

  if system == "Linux":
    terminal_cmd = _select_linux_terminal(shell_cmd)
    if terminal_cmd is None:
      raise RuntimeError("no supported Linux terminal emulator found")
    return terminal_cmd

  raise RuntimeError(f"unsupported OS: {system}")


def _select_linux_terminal(shell_cmd: str) -> list[str] | None:
  quoted_shell = shlex.quote(shell_cmd)
  candidates = [
    ("gnome-terminal", ["gnome-terminal", "--", "bash", "-lc", shell_cmd]),
    ("konsole", ["konsole", "-e", "bash", "-lc", shell_cmd]),
    ("xfce4-terminal", ["xfce4-terminal", "--command", f"bash -lc {quoted_shell}"]),
    ("mate-terminal", ["mate-terminal", "--", "bash", "-lc", shell_cmd]),
    ("x-terminal-emulator", ["x-terminal-emulator", "-e", "bash", "-lc", shell_cmd]),
    ("xterm", ["xterm", "-e", "bash", "-lc", shell_cmd]),
    ("alacritty", ["alacritty", "-e", "bash", "-lc", shell_cmd]),
    ("kitty", ["kitty", "bash", "-lc", shell_cmd]),
    ("tilix", ["tilix", "--new-process", "bash", "-lc", shell_cmd]),
  ]
  for program, cmd in candidates:
    if _which(program):
      return cmd
  return None


def _which(program: str) -> str | None:
  for path in os.environ.get("PATH", "").split(os.pathsep):
    candidate = Path(path) / program
    if candidate.is_file() and os.access(candidate, os.X_OK):
      return str(candidate)
  return None


def apple_script_string(value: str) -> str:
  escaped = value.replace("\\", "\\\\").replace('"', '\\"')
  return f'"{escaped}"'
