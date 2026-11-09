"""Pre-run checks, and finding the lead on the hotspot.

Each check exists because something silently didn't work once. The failure
modes this catches are all invisible at runtime: a relay that isn't running, a
param that isn't set, a root-owned log directory, a clock far enough out that
every packet is rejected without a word.
"""
from __future__ import annotations

import shlex
import subprocess
import sys
import time

from openpilot.tools.hcc.common import Settings
from openpilot.tools.hcc.fieldtest.remote import (
  CheckFailure,
  measure_clock_skew_ms,
  remote_py_prefix,
  ssh,
  ssh_ok,
)

# The subscriber and relay reject packets beyond 500 ms of skew, silently.
CLOCK_SKEW_WARN_MS = 150.0
CLOCK_SKEW_FAIL_MS = 400.0

EGO_PARAM_EXPECTATIONS = (
  ("EnableHCCC", "1"),
  ("AlphaLongitudinalEnabled", "1"),
  ("HCCV2VEnabled", "1"),
  ("HCCV2VOnly", "1"),
)

_READ_PARAMS_PY = """
from openpilot.common.params import Params
p = Params()
bools = {bool_keys}
strs = {str_keys}
for k in bools:
    try:
        print(k + '=' + str(int(bool(p.get_bool(k)))))
    except Exception:
        print(k + '=?')
for k in strs:
    try:
        v = p.get(k)
        v = v.decode() if isinstance(v, (bytes, bytearray)) else (v if v is not None else '')
    except Exception:
        v = ''
    print(k + '=' + str(v).strip())
"""


class Reporter:
  def __init__(self):
    self.failures = 0
    self.warnings = 0

  def __call__(self, status: str, message: str) -> None:
    if status == "FAIL":
      self.failures += 1
    elif status == "WARN":
      self.warnings += 1
    print(f"  [{status:4}] {message}")

  def ok(self, condition: bool, pass_msg: str, fail_msg: str, fail_status: str = "FAIL") -> bool:
    self(("PASS" if condition else fail_status), pass_msg if condition else fail_msg)
    return condition


def read_remote_params(settings: Settings, host: str, user: str, repo: str,
                       bool_keys: list[str], str_keys: list[str]) -> dict[str, str]:
  script = _READ_PARAMS_PY.format(bool_keys=bool_keys, str_keys=str_keys)
  out = ssh_ok(user, host, f"{remote_py_prefix(user, host, repo)} -c {shlex.quote(script)}", timeout=30)
  values: dict[str, str] = {}
  for line in out.splitlines():
    if "=" in line:
      key, _, value = line.partition("=")
      values[key.strip()] = value.strip()
  return values


def preflight(settings: Settings, lead_host: str, check_pedal: bool = False) -> int:
  """Run every check, printing PASS/WARN/FAIL with the fix. Returns FAIL count."""
  report = Reporter()
  ego, lead = settings.ego, settings.lead
  log_dir = settings.remote_log_dir

  print(f"Preflight: ego={ego.host} lead={lead_host} user={ego.user}")

  # --- reachability. Nothing else is checkable without this.
  for role, host, user in (("ego", ego.host, ego.user), ("lead", lead_host, lead.user)):
    try:
      ssh_ok(user, host, "true", timeout=10)
      report("PASS", f"{role}: SSH reachable at {host}")
    except (CheckFailure, subprocess.TimeoutExpired) as exc:
      report("FAIL", f"{role}: SSH to {host} failed ({exc}) — is this laptop on the ego's "
                     f"hotspot? `hcc find-lead` lists what the ego can see.")
      return report.failures

  # --- the relay
  active = ssh(ego.user, ego.host, "systemctl is-active hcc-v2v-relay").stdout.strip() == "active"
  report.ok(active, "ego: hcc-v2v-relay service active",
            "ego: relay not running — on the ego, "
            "`sudo tools/hcc/device/setup_network.sh ego`")

  # --- log directory ownership. The relay runs as root and creates the
  # directory root-owned if it gets there first; every later comma-user launch
  # then dies instantly on its log redirection (bench take 4).
  for role, host, user, peer_log_dir in (("ego", ego.host, ego.user, ego.log_dir or log_dir),
                                         ("lead", lead_host, lead.user, lead.log_dir or log_dir)):
    writable = ssh(user, host, f"mkdir -p {peer_log_dir} 2>/dev/null; "
                               f"touch {peer_log_dir}/.wtest && rm {peer_log_dir}/.wtest").returncode == 0
    report.ok(writable, f"{role}: {peer_log_dir} writable as {user}",
              f"{role}: {peer_log_dir} not writable as {user} — "
              f"ssh in and `sudo chown {user}:{user} {peer_log_dir}`")

  # --- ego params
  bool_keys = [k for k, _ in EGO_PARAM_EXPECTATIONS] + ["HCCPedalEnabled"]
  try:
    values = read_remote_params(settings, ego.host, ego.user, ego.repo,
                                bool_keys, ["HCCV2VRelayHost", "HCCPedalDeviceId"])
    for key, expected in EGO_PARAM_EXPECTATIONS:
      report.ok(values.get(key) == expected,
                f"ego: param {key}={expected}",
                f"ego: param {key}={values.get(key)!r}, expected {expected} — "
                f"re-run setup_network.sh, or set it in the developer settings")

    relay_host = values.get("HCCV2VRelayHost", "")
    report.ok(relay_host in ("127.0.0.1", "localhost"),
              f"ego: HCCV2VRelayHost={relay_host} (on-device relay)",
              f"ego: HCCV2VRelayHost={relay_host!r} — expected 127.0.0.1 for in-car testing",
              fail_status="WARN")

    # The pedal term is the point of the Fall-26 configuration, so its state is
    # reported either way rather than only when it is being used.
    pedal_on = values.get("HCCPedalEnabled") == "1"
    if check_pedal:
      report.ok(pedal_on, "ego: HCCPedalEnabled=1 (pedal rig is the manual input)",
                "ego: HCCPedalEnabled=0 but a pedal run was requested — the blend will "
                "read CarState instead, which is 0.0 on this platform")
      if values.get("HCCPedalDeviceId", "") not in ("", settings.pedal_device_id):
        report("WARN", f"ego: HCCPedalDeviceId={values['HCCPedalDeviceId']!r} but devices.toml "
                       f"says {settings.pedal_device_id!r} — the ego will ignore this rig")
    elif pedal_on:
      report("WARN", "ego: HCCPedalEnabled=1 with no pedal publisher in this run — "
                     "the ego will deny positive accel until the rig publishes")
  except (CheckFailure, subprocess.TimeoutExpired) as exc:
    report("FAIL", f"ego: could not read params ({exc})")

  # --- lead: the deployed publisher bundle
  #
  # The lead does not run openpilot, so there are no params to read here. What
  # matters is that the bundle is deployed, that its interpreter exists, and
  # that the publisher actually imports -- a half-finished rsync looks fine
  # until the launch dies a second in.
  lead_py = lead.python or "python3"
  version = ssh(lead.user, lead_host, f"{lead_py} --version 2>&1").stdout.strip()
  report.ok(version.startswith("Python"), f"lead: {version}",
            f"lead: {lead_py} not found — set [lead] python in devices.toml")

  has_csv = ssh(lead.user, lead_host, f"test -f {lead.repo}/tools/hcc/config/scenarios.csv").returncode == 0
  report.ok(has_csv, "lead: scenario table present",
            f"lead: {lead.repo}/tools/hcc/config/scenarios.csv missing — "
            f"run `./tools/hcc/device/deploy_lead.sh {lead.user}@{lead_host}`")

  imports = ssh(lead.user, lead_host,
                f"cd {lead.repo} && PYTHONPATH={lead.repo} {lead_py} -c "
                f"'import openpilot.tools.hcc.publishers.virtual_lead'").returncode == 0
  report.ok(imports, "lead: publisher imports",
            f"lead: publisher will not import from {lead.repo} — re-run deploy_lead.sh")

  can_ping = ssh(lead.user, lead_host, f"ping -c 1 -W 2 {ego.host}").returncode == 0
  report.ok(can_ping, f"lead: can reach ego at {ego.host}",
            "lead: cannot ping the ego — is the lead on the hotspot? "
            "`sudo tools/hcc/device/setup_network.sh lead`")

  # --- clocks
  try:
    skew_ms = measure_clock_skew_ms(ego.user, ego.host, lead_host)
    if abs(skew_ms) >= CLOCK_SKEW_FAIL_MS:
      report("FAIL", f"lead-ego clock skew {skew_ms:+.0f} ms (limit 500) — V2V packets WILL be "
                     f"rejected, silently. Give the lead internet briefly, or from the lead: "
                     f'`sudo date -s "$(ssh {ego.user}@{ego.host} date)"`')
    elif abs(skew_ms) >= CLOCK_SKEW_WARN_MS:
      report("WARN", f"lead-ego clock skew {skew_ms:+.0f} ms — works, but close to the 500 ms limit")
    else:
      report("PASS", f"lead-ego clock skew {skew_ms:+.0f} ms")
  except (CheckFailure, subprocess.TimeoutExpired, ValueError) as exc:
    report("WARN", f"could not measure clock skew ({exc})")

  if check_pedal:
    # The pedal publisher runs on THIS laptop, so its clock is now in the
    # critical path exactly as the lead's is.
    try:
      ego_dev_us = int(ssh_ok(ego.user, ego.host, "date +%s%N")) / 1000.0
      laptop_us = time.time_ns() / 1000.0
      skew_ms = (laptop_us - ego_dev_us) / 1000.0
      if abs(skew_ms) >= CLOCK_SKEW_FAIL_MS:
        report("FAIL", f"laptop-ego clock skew {skew_ms:+.0f} ms — pedal packets WILL be rejected")
      elif abs(skew_ms) >= CLOCK_SKEW_WARN_MS:
        report("WARN", f"laptop-ego clock skew {skew_ms:+.0f} ms — close to the 500 ms limit")
      else:
        report("PASS", f"laptop-ego clock skew {skew_ms:+.0f} ms")
    except (CheckFailure, subprocess.TimeoutExpired, ValueError) as exc:
      report("WARN", f"could not measure laptop clock skew ({exc})")

  print(f"Preflight done: {report.failures} failure(s), {report.warnings} warning(s)")
  return report.failures


def find_lead(settings: Settings) -> int:
  """List what the ego's hotspot can see; the lead should be one of them."""
  ego = settings.ego
  try:
    leases = ssh(ego.user, ego.host,
                 "cat /var/lib/NetworkManager/dnsmasq-wlan0.leases 2>/dev/null; "
                 "ip neigh show dev wlan0 2>/dev/null")
  except subprocess.TimeoutExpired:
    print(f"could not reach the ego at {ego.host}", file=sys.stderr)
    return 1
  print("Devices on the ego's hotspot:")
  print(leases.stdout.strip() or "  (none — has the lead joined? This laptop should appear here too.)")
  print("\nThe lead publisher (Raspberry Pi) and this laptop should both appear.")
  print("Put the lead's address in config/devices.toml as [lead] host, or pass --lead_host.")
  return 0
