#!/usr/bin/env python3
"""Virtual lead vehicle: replay a scenario speed profile as V2V packets.

Runs on the LEAD device, or on any machine that can reach the ego's relay.
Instead of measuring a real lead car it reads one scenario column and
publishes the speed/accel profile over the normal V2V wire format -- the ego
cannot tell it from a real lead.

This is the phase-1 test tool: the ego drives the car while the lead device
sits powered in the same car, joined to the ego's hotspot, feeding it scenario
data.

  python3 tools/hcc/publishers/virtual_lead.py --scn 48

Everything defaults from config/devices.toml, so no flags are needed on a
configured setup. Playback end behavior: 'hold' keeps publishing the final
speed forever (the ego keeps cruising), 'stop' exits and lets the ego withdraw
longitudinal authority through V2V staleness.
"""
from __future__ import annotations

import argparse
import csv
import os
import time
from pathlib import Path

from openpilot.tools.hcc.common import SCENARIOS_CSV, load_settings
from openpilot.tools.hcc.publishers.profile import (
  MPH_TO_MPS,
  SENT_CSV_COLUMNS,
  describe,
  load_speed_profile,
  resolve_speed_scale,
  sample_profile,
)
from openpilot.tools.hcc.publishers.publisher import Publisher
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  ROLE_LEAD,
  V2VLeadPacket,
  encode_data_packet,
  wall_time_us,
)


class VirtualLeadPublisher(Publisher):
  role = ROLE_LEAD

  def publish(self, v_lead: float, a_lead: float) -> V2VLeadPacket:
    packet = V2VLeadPacket(device_id=self.device_id, timestamp_us=wall_time_us(),
                           a_lead=float(a_lead), v_lead=float(v_lead), seq=self.seq)
    self._send(encode_data_packet(packet))
    return packet


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  settings = load_settings()
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--scn", type=int, default=settings.scenario, help="Scenario column (default from devices.toml)")
  p.add_argument("--csv", default=str(SCENARIOS_CSV), help="Scenario CSV path")
  p.add_argument("--relay_host", default=settings.relay_host)
  p.add_argument("--relay_port", type=int, default=settings.relay_port)
  p.add_argument("--device_id", default=settings.lead.device_id)
  p.add_argument("--hz", type=float, default=settings.send_hz)
  p.add_argument("--start_delay", type=float, default=0.0,
                 help="Seconds to publish the initial speed before the profile starts")
  p.add_argument("--speed_scale", type=float, default=1.0, help="Multiply the whole profile's speeds")
  p.add_argument("--max_speed_mph", type=float, default=settings.max_speed_mph,
                 help="Scale the profile down so its top speed is at most this (proportional, not a clip)")
  p.add_argument("--end", choices=("hold", "stop"), default="hold")
  p.add_argument("--loop", action="store_true", help="Restart the profile from t=0 when it ends")
  p.add_argument("--duration", type=float, default=None, help="Exit after this many seconds regardless")
  p.add_argument("--log_csv", default=None, help="Write every sent packet to this CSV")
  return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)

  times, speeds = load_speed_profile(args.csv, args.scn)
  scale = resolve_speed_scale(speeds, args.speed_scale, args.max_speed_mph)
  if scale != 1.0:
    speeds = [v * scale for v in speeds]
    print(f"[virtual_lead] speed scale {scale:.4f} applied to the whole profile")
  profile_len = times[-1] - times[0]
  print(f"[virtual_lead] scenario {args.scn}: {describe(times, speeds)}")
  print(f"[virtual_lead] publishing as {args.device_id!r} -> {args.relay_host}:{args.relay_port} "
        f"at {args.hz:.0f} Hz (end={args.end}{', loop' if args.loop else ''})")

  log_file = None
  log_writer = None
  if args.log_csv:
    log_path = Path(os.path.expanduser(args.log_csv))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(SENT_CSV_COLUMNS)
    log_file.flush()

  pub = VirtualLeadPublisher(args.relay_host, args.relay_port, args.device_id)
  interval = 1.0 / args.hz
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
          print("[virtual_lead] profile finished (end=stop) — ego will withdraw authority via staleness")
          break
        v, a = sample_profile(times, speeds, t_profile)
        t_disp = min(t_profile, times[-1])

      pub.maybe_hello()
      packet = pub.publish(v, a)
      if log_writer is not None:
        log_writer.writerow([wall_time_us(), f"{t_disp:.3f}", packet.seq, f"{v:.4f}", f"{a:.4f}"])

      if now - last_status >= 1.0:
        last_status = now
        print(f"[virtual_lead] t={t_disp:7.1f}s  v={v:6.2f} m/s ({v / MPH_TO_MPS:5.1f} mph)  "
              f"a={a:+5.2f} m/s2  seq={packet.seq}")

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
