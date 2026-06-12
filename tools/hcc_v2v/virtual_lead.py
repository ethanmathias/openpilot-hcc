#!/usr/bin/env python3
"""Virtual lead vehicle: replay a Scenarios.csv speed profile as V2V packets.

Runs on the LEAD device (or any machine that can reach the ego's V2V relay).
Instead of measuring a real lead car, it reads one scenario column from the
CSV and publishes the speed/accel profile over the normal V2V wire format —
the ego device cannot tell the difference from a real lead.

This is the phase-1 in-car test tool: the ego device drives the car while the
lead device sits powered in the same car, joined to the ego's hotspot, feeding
it scenario data.

Typical use on the lead device (after setup_v2v_network.sh lead has joined the
hotspot and set HCCV2VRelayHost to the ego's hotspot IP):

    cd /data/openpilot
    python3 tools/hcc_v2v/virtual_lead.py --scn 48

Relay host/port/device-id default to the device's HCCV2V* params, so no flags
are needed on a device that has been through setup_v2v_network.sh. Override
with --relay_host etc. when testing from a PC.

The CSV format matches the simulator's scenario replay: column 0 is time in
seconds, column N is scenario N's speed in m/s. Playback ends according to
--end: 'hold' keeps publishing the final speed forever (ego keeps cruising),
'stop' exits and lets the ego disengage via V2V staleness within ~100 ms.
"""
from __future__ import annotations

import argparse
import csv
import os
import socket
import sys
import time
from pathlib import Path

# Allow running as `python3 tools/hcc_v2v/virtual_lead.py` from the repo root
# without an installed openpilot package.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT.parent) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT.parent))
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

# hcc_v2v_core is pure stdlib; load_v2v_config (imported lazily in main) needs
# the compiled openpilot Params module, which only exists on a built checkout.
from openpilot.selfdrive.controls.lib.vendor.hcc_v2v_core import (
  HELLO_INTERVAL_S,
  ROLE_LEAD,
  V2VHelloPacket,
  V2VLeadPacket,
  encode_data_packet,
  encode_hello_packet,
  wall_time_us,
)

DEFAULT_SCENARIOS_CSV = REPO_ROOT / "tools" / "sim" / "lib" / "Scenarios.csv"
SENT_CSV_COLUMNS = ["wall_time_us", "profile_time_s", "seq", "v_lead_mps", "a_lead_mps2"]


def load_speed_profile(csv_path: str | Path, scenario_index: int) -> tuple[list[float], list[float]]:
  """Read (time_s, speed_mps) samples for one scenario column.

  Same semantics as the simulator's loader (_load_lead_speed_profile in
  metadrive_process.py): column 0 is time, column `scenario_index` is speed;
  reading stops at the first blank/invalid cell after data has started.
  """
  if scenario_index < 1:
    raise ValueError(f"scenario index must be >= 1, got {scenario_index}")

  times: list[float] = []
  speeds: list[float] = []
  with open(csv_path, newline="") as f:
    for row in csv.reader(f):
      if not row or not row[0].strip():
        continue
      try:
        t = float(row[0].strip())
      except ValueError:
        continue
      if scenario_index >= len(row):
        if speeds:
          break
        continue
      cell = row[scenario_index].strip()
      if not cell:
        if speeds:
          break
        continue
      try:
        v = max(float(cell), 0.0)
      except ValueError:
        if speeds:
          break
        continue
      times.append(t)
      speeds.append(v)

  if len(times) < 2:
    raise RuntimeError(f"scenario {scenario_index} has fewer than 2 samples in {csv_path}")

  # Sort by time and drop duplicate timestamps (keep first occurrence).
  order = sorted(range(len(times)), key=lambda i: times[i])
  out_t: list[float] = []
  out_v: list[float] = []
  for i in order:
    if out_t and times[i] == out_t[-1]:
      continue
    out_t.append(times[i])
    out_v.append(speeds[i])
  if len(out_t) < 2:
    raise RuntimeError(f"scenario {scenario_index} needs at least two unique time samples in {csv_path}")
  return out_t, out_v


def sample_profile(times: list[float], speeds: list[float], t: float) -> tuple[float, float]:
  """Return (v, a) at profile time t: linear speed interpolation, per-segment accel."""
  if t <= times[0]:
    return speeds[0], 0.0
  if t >= times[-1]:
    return speeds[-1], 0.0
  # Binary search for the segment containing t.
  lo, hi = 0, len(times) - 1
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if times[mid] <= t:
      lo = mid
    else:
      hi = mid
  dt = times[hi] - times[lo]
  a = (speeds[hi] - speeds[lo]) / dt if dt > 0.0 else 0.0
  v = speeds[lo] + a * (t - times[lo])
  return v, a


class VirtualLeadPublisher:
  """Minimal V2V lead publisher with periodic hello re-registration.

  One socket for the whole run: the relay pins the lead's (host, port), so the
  socket must never be recreated mid-run. Hello is re-sent every
  HELLO_INTERVAL_S so a relay restart re-learns this lead automatically.
  """

  def __init__(self, relay_host: str, relay_port: int, device_id: str):
    self.relay = (relay_host, int(relay_port))
    self.device_id = device_id
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.seq = 0
    self._last_hello = 0.0
    self._hello_payload = encode_hello_packet(V2VHelloPacket(device_id=device_id, role=ROLE_LEAD))

  def maybe_hello(self) -> None:
    now = time.monotonic()
    if now - self._last_hello >= HELLO_INTERVAL_S:
      self.sock.sendto(self._hello_payload, self.relay)
      self._last_hello = now

  def publish(self, v_lead: float, a_lead: float) -> V2VLeadPacket:
    packet = V2VLeadPacket(device_id=self.device_id, timestamp_us=wall_time_us(),
                           a_lead=float(a_lead), v_lead=float(v_lead), seq=self.seq)
    self.seq += 1
    self.sock.sendto(encode_data_packet(packet), self.relay)
    return packet

  def close(self) -> None:
    self.sock.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--scn", type=int, required=True, help="Scenario column in the CSV (e.g. 48)")
  p.add_argument("--csv", default=str(DEFAULT_SCENARIOS_CSV), help=f"Scenario CSV path (default: {DEFAULT_SCENARIOS_CSV})")
  p.add_argument("--relay_host", default=None, help="V2V relay host (default: HCCV2VRelayHost param)")
  p.add_argument("--relay_port", type=int, default=None, help="V2V relay port (default: HCCV2VRelayPort param or 19090)")
  p.add_argument("--device_id", default=None, help="Lead device id (default: HCCV2VDeviceId param or hcc-lead)")
  p.add_argument("--hz", type=float, default=None, help="Publish rate (default: HCC_V2V_SEND_HZ or 50)")
  p.add_argument("--start_delay", type=float, default=0.0, help="Seconds to publish the initial speed before the profile starts")
  p.add_argument("--end", choices=("hold", "stop"), default="hold",
                 help="After the profile ends: 'hold' final speed forever, or 'stop' publishing (ego disengages via staleness)")
  p.add_argument("--loop", action="store_true", help="Restart the profile from t=0 when it ends")
  p.add_argument("--duration", type=float, default=None, help="Exit after this many seconds regardless of profile length")
  p.add_argument("--log_csv", default=None, help="Write every sent packet to this CSV for post-run analysis")
  return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)

  times, speeds = load_speed_profile(args.csv, args.scn)
  profile_len = times[-1] - times[0]
  print(f"[virtual_lead] scenario {args.scn}: {len(times)} samples, "
        f"{profile_len:.1f} s, v in [{min(speeds):.2f}, {max(speeds):.2f}] m/s")

  try:
    from openpilot.selfdrive.controls.lib.hcc_v2v import load_v2v_config
    cfg = load_v2v_config(ROLE_LEAD, default_enabled=True, default_device_id="hcc-lead")
    relay_host = args.relay_host or cfg.relay_host
    relay_port = args.relay_port or cfg.relay_port
    device_id = args.device_id or cfg.device_id
    send_hz = args.hz or cfg.send_hz
  except ImportError:
    # Unbuilt checkout (no compiled Params): flags/defaults only.
    relay_host = args.relay_host or "127.0.0.1"
    relay_port = args.relay_port or 19090
    device_id = args.device_id or "hcc-lead"
    send_hz = args.hz or 50.0
  print(f"[virtual_lead] publishing as {device_id!r} -> {relay_host}:{relay_port} at {send_hz:.0f} Hz "
        f"(end={args.end}{', loop' if args.loop else ''})")

  log_file = None
  log_writer = None
  if args.log_csv:
    log_path = Path(os.path.expanduser(args.log_csv))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(SENT_CSV_COLUMNS)

  pub = VirtualLeadPublisher(relay_host, relay_port, device_id)
  interval = 1.0 / send_hz
  t_start = time.monotonic() + args.start_delay
  next_send = time.monotonic()
  last_status = 0.0

  try:
    while True:
      now = time.monotonic()
      if args.duration is not None and now - t_start >= args.duration:
        print("[virtual_lead] --duration reached, stopping")
        break

      t = now - t_start
      if t < 0.0:
        v, a = speeds[0], 0.0  # start_delay: hold the initial speed
        t_disp = 0.0
      else:
        t_profile = times[0] + (t % profile_len if args.loop else t)
        if not args.loop and t_profile >= times[-1] and args.end == "stop":
          print("[virtual_lead] profile finished (end=stop), stopping — ego will disengage via staleness")
          break
        v, a = sample_profile(times, speeds, t_profile)
        t_disp = min(t_profile, times[-1])

      pub.maybe_hello()
      packet = pub.publish(v, a)
      if log_writer is not None:
        log_writer.writerow([wall_time_us(), f"{t_disp:.3f}", packet.seq, f"{v:.4f}", f"{a:.4f}"])

      if now - last_status >= 1.0:
        last_status = now
        print(f"[virtual_lead] t={t_disp:7.1f}s  v={v:6.2f} m/s  a={a:+5.2f} m/s²  seq={packet.seq}")

      next_send += interval
      sleep_s = next_send - time.monotonic()
      if sleep_s > 0:
        time.sleep(sleep_s)
      else:
        next_send = time.monotonic()  # fell behind; don't burst to catch up
  except KeyboardInterrupt:
    print("\n[virtual_lead] interrupted, stopping")
  finally:
    pub.close()
    if log_file is not None:
      log_file.close()
      print(f"[virtual_lead] sent-packet log written to {args.log_csv}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
