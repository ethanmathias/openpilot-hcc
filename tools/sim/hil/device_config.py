"""Loader for tools/sim/hil/devices.toml plus a soft sanity check that the
configured device IP is actually reachable on the named PC interface.

The check warns on mismatch instead of failing — RNDIS interface names
shuffle on hot-plug and the warning is more useful than a hard error.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import subprocess
import sys
import tomllib


@dataclass(frozen=True)
class DeviceConfig:
  role: str          # "lead" or "ego"
  iface: str         # PC-side RNDIS interface name
  ip: str            # device-side RNDIS IP
  hotspot_ip: str | None = None  # only set on the ego entry


@dataclass(frozen=True)
class DevicesConfig:
  lead: DeviceConfig
  ego: DeviceConfig

  def by_role(self, role: str) -> DeviceConfig:
    if role == "lead":
      return self.lead
    if role == "ego":
      return self.ego
    raise ValueError(f"unknown role {role!r}")


def load(path: str | Path) -> DevicesConfig:
  with open(path, "rb") as f:
    raw = tomllib.load(f)
  return DevicesConfig(
    lead=DeviceConfig(role="lead", iface=raw["lead"]["iface"], ip=raw["lead"]["ip"]),
    ego=DeviceConfig(role="ego", iface=raw["ego"]["iface"], ip=raw["ego"]["ip"], hotspot_ip=raw["ego"].get("hotspot_ip")),
  )


def discover_device_ip(iface: str) -> str | None:
  """Best-effort: return the peer (device-side) IP of the named PC interface, or None."""
  if sys.platform == "darwin":
    try:
      out = subprocess.run(["ifconfig", iface], capture_output=True, text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
      return None
    for line in out.splitlines():
      line = line.strip()
      if line.startswith("inet ") and "-->" in line:
        # "inet 192.168.32.1 --> 192.168.32.10 netmask 0xffffff00"
        return line.split("-->")[1].strip().split()[0]
    return None

  try:
    out = subprocess.run(["ip", "-j", "-4", "addr", "show", iface], capture_output=True, text=True, check=True).stdout
  except (subprocess.CalledProcessError, FileNotFoundError):
    return None
  try:
    data = json.loads(out)
  except json.JSONDecodeError:
    return None
  for entry in data:
    for ai in entry.get("addr_info", []):
      peer = ai.get("address")  # on point-to-point links, "address" is local
      if "broadcast" not in ai and "local" in ai:
        return ai.get("address")
      if peer:
        return peer
  return None


def warn_on_mismatch(cfg: DevicesConfig) -> None:
  """Print a loud warning if the configured IP doesn't appear on the iface. Don't raise."""
  for dev in (cfg.lead, cfg.ego):
    found = discover_device_ip(dev.iface)
    if found is None:
      print(f"[hil] WARN: could not inspect interface {dev.iface!r} for {dev.role}; is the device plugged in?", file=sys.stderr)
    elif found != dev.ip:
      print(f"[hil] WARN: {dev.role} configured IP {dev.ip} does not match interface {dev.iface!r} (got {found})", file=sys.stderr)
