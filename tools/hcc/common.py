"""Shared bootstrap and settings for the HCC tooling.

Two jobs:

1. Make `python3 tools/hcc/...` work from the repo root on a checkout that has
   not been built, by putting the repo on sys.path. The transport modules are
   pure stdlib, so the publishers and the relay run anywhere; only the pieces
   that read Params need a built device.

2. Load config/devices.toml once, so no command has to be given hosts, users,
   ports or device ids on the command line.
"""
from __future__ import annotations

import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

HCC_DIR = Path(__file__).resolve().parent
REPO_ROOT = HCC_DIR.parents[1]
CONFIG_DIR = HCC_DIR / "config"
DEVICES_TOML = CONFIG_DIR / "devices.toml"
SCENARIOS_CSV = CONFIG_DIR / "scenarios.csv"
RUNS_DIR = HCC_DIR / "runs"

# openpilot is imported as `openpilot.<...>`, so the repo's PARENT has to be on
# the path as well as the repo itself.
for _path in (REPO_ROOT.parent, REPO_ROOT):
  if str(_path) not in sys.path:
    sys.path.insert(0, str(_path))


@dataclass(frozen=True)
class Peer:
  host: str
  user: str
  repo: str
  device_id: str


@dataclass(frozen=True)
class Settings:
  relay_host: str
  relay_port: int
  send_hz: float
  ego: Peer
  lead: Peer
  pedal_device_id: str
  pedal_stale_ms: int
  scenario: int
  max_speed_mph: float
  monitor_hz: float
  remote_log_dir: str

  @property
  def relay(self) -> tuple[str, int]:
    return (self.relay_host, self.relay_port)


def _peer(table: dict, default_id: str) -> Peer:
  return Peer(
    host=str(table.get("host", "")),
    user=str(table.get("user", "comma")),
    repo=str(table.get("repo", "/data/openpilot")),
    device_id=str(table.get("device_id", default_id)),
  )


def load_settings(path: str | Path | None = None) -> Settings:
  config_path = Path(path) if path is not None else DEVICES_TOML
  with open(config_path, "rb") as f:
    raw = tomllib.load(f)

  network = raw.get("network", {})
  pedal = raw.get("pedal", {})
  run = raw.get("run", {})

  return Settings(
    relay_host=str(network.get("relay_host", "127.0.0.1")),
    relay_port=int(network.get("relay_port", 19090)),
    send_hz=float(network.get("send_hz", 50.0)),
    ego=_peer(raw.get("ego", {}), "hcc-ego"),
    lead=_peer(raw.get("lead", {}), "hcc-lead"),
    pedal_device_id=str(pedal.get("device_id", "hcc-pedals")),
    pedal_stale_ms=int(pedal.get("stale_ms", 150)),
    scenario=int(run.get("scenario", 48)),
    max_speed_mph=float(run.get("max_speed_mph", 35.0)),
    monitor_hz=float(run.get("monitor_hz", 10.0)),
    remote_log_dir=str(run.get("remote_log_dir", "/data/hcc_v2v_logs")),
  )
