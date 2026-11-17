#!/usr/bin/env python3
"""Pedal rig publisher: the human-in-the-loop longitudinal input.

Runs on the LAPTOP, which is a third peer on the ego's hotspot alongside the
ego and the lead. Reads the pedal rig, normalizes to fractions, and publishes
over the V2V wire format at 50 Hz. The ego blends the result into output_accel
in place of CarState.gas / CarState.brake.

  # bench, no hardware needed
  python3 tools/hcc/publishers/pedal.py --source synthetic

  # real rig, after picking axes with --list-axes
  python3 tools/hcc/publishers/pedal.py --throttle_axis 1 --brake_axis 2

Why this exists: on the 2026-07-11 Tucson runs the manual blend term was
exactly 0.0 in 100% of samples. CarState.gas is deprecated out of the opendbc
schema and Hyundai never populates CarState.brake, so there was nothing to
blend. Taking the input from a rig on the V2V network also means the subject's
feet never touch the car's own pedals, so the panda never sees an override --
which is what makes the whole approach viable without patching safety firmware.

It heartbeats at the full rate whether or not the pedals move, so silence on
the wire is unambiguously a dropout rather than a still foot. The ego treats a
stale pedal link as a fault and denies positive acceleration until it
recovers, so this process stopping is safe but will prevent the car
accelerating.
"""
from __future__ import annotations

import argparse
import csv
import os
import time
from pathlib import Path

from openpilot.tools.hcc.common import load_settings
from openpilot.tools.hcc.publishers.pedal_source import (
  AxisCalibration,
  JoystickPedalSource,
  build_source,
  identify_axes,
)
from openpilot.tools.hcc.publishers.publisher import Publisher
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  ROLE_PEDAL,
  V2VPedalPacket,
  encode_pedal_packet,
  wall_time_us,
)

SENT_CSV_COLUMNS = ["wall_time_us", "seq", "throttle", "brake", "throttle_raw", "brake_raw"]


class PedalPublisher(Publisher):
  role = ROLE_PEDAL

  def publish(self, throttle: float, brake: float,
              throttle_raw: float, brake_raw: float) -> V2VPedalPacket:
    packet = V2VPedalPacket(device_id=self.device_id, timestamp_us=wall_time_us(),
                            throttle=float(throttle), brake=float(brake), seq=self.seq,
                            throttle_raw=float(throttle_raw), brake_raw=float(brake_raw))
    self._send(encode_pedal_packet(packet))
    return packet


def report_identification(result: dict, out=print) -> int:
  """Turn an identify_axes() result into operator instructions and an exit code.

  Separated from the device handling so it can be tested without a rig.
  """
  throttle, brake = result.get("throttle"), result.get("brake")

  if throttle is None or brake is None:
    out("\nIdentification incomplete — re-run and hold each pedal until it reads.")
    return 1
  if throttle == brake:
    out("\nBoth pedals resolved to axis "
        f"{throttle} — re-run, releasing fully between steps.")
    return 1

  out("\nRecord this in tools/hcc/config/devices.toml under [pedal]:\n")
  out(f"  throttle_axis = {throttle}")
  out(f"  brake_axis    = {brake}")

  held = result.get("held", {})
  if held:
    out("\nHeld: " + ",  ".join(f"{name} {value:+.3f}" for name, value in held.items()))

  rest = result.get("rest") or []
  brake_held = held.get("brake")
  if brake_held is not None and brake < len(rest):
    travel = abs(brake_held - rest[brake])
    full_travel = 2.0  # the rig spans -1..+1
    if travel < 0.85 * full_travel:
      out(f"\nThe brake held at {travel / full_travel:.0%} of full travel. That is normal "
          "for a load cell:\ncalibration captures whatever maximum you actually apply, so "
          "press it as hard\nas you would in the car when the publisher calibrates.")
  return 0


def identify(args) -> int:
  """Guided axis identification; prints the devices.toml lines to record."""
  source = JoystickPedalSource(throttle_axis=0, brake_axis=0)
  try:
    result = identify_axes(source, timeout_s=args.identify_timeout)
  finally:
    source.close()
  return report_identification(result)


def list_axes(args) -> int:
  """Print live values for every axis so the right indices can be picked."""
  source = JoystickPedalSource(throttle_axis=0, brake_axis=0)
  print(f"device: {source.device_name!r}, {source.num_axes} axes")
  print("Press each pedal in turn; Ctrl-C to stop.\n")
  try:
    while True:
      values = source.all_axes()
      print("  " + "  ".join(f"[{i}] {v:+.3f}" for i, v in enumerate(values)), end="\r", flush=True)
      time.sleep(0.05)
  except KeyboardInterrupt:
    print("\n")
  finally:
    source.close()
  return 0


def calibrate(source, seconds: float = 3.0) -> tuple[AxisCalibration, AxisCalibration]:
  """Capture rest, then a full-travel reference for each axis.

  Per session, not once: a load cell's rest offset moves with temperature and
  mount preload.
  """
  print(f"[pedal] calibration: feet OFF the pedals for {seconds:.0f} s...")
  rest_t, rest_b = _extremes(source, seconds)
  throttle_rest, brake_rest = rest_t[0], rest_b[0]

  print(f"[pedal] rest captured (throttle {throttle_rest:+.3f}, brake {brake_rest:+.3f})")
  print(f"[pedal] now press BOTH pedals to full travel, hard, for {seconds:.0f} s...")
  full_t, full_b = _extremes(source, seconds)
  throttle_full, brake_full = full_t[1], full_b[1]

  throttle_cal = AxisCalibration(rest=throttle_rest, full=throttle_full)
  brake_cal = AxisCalibration(rest=brake_rest, full=brake_full)
  print(f"[pedal] throttle {throttle_rest:+.3f} -> {throttle_full:+.3f}   "
        f"brake {brake_rest:+.3f} -> {brake_full:+.3f}")

  for name, cal in (("throttle", throttle_cal), ("brake", brake_cal)):
    if not cal.is_plausible:
      print(f"[pedal] WARNING: {name} axis did not move during calibration — "
            f"it will read 0.0 for the whole run. Check the axis index.")
  return throttle_cal, brake_cal


def _extremes(source, seconds: float) -> tuple[tuple[float, float], tuple[float, float]]:
  """(min, max) seen on each axis over a window."""
  t_min = b_min = float("inf")
  t_max = b_max = float("-inf")
  deadline = time.monotonic() + seconds
  while time.monotonic() < deadline:
    t_raw, b_raw = source.read()
    t_min, t_max = min(t_min, t_raw), max(t_max, t_raw)
    b_min, b_max = min(b_min, b_raw), max(b_max, b_raw)
    time.sleep(0.02)
  return (t_min, t_max), (b_min, b_max)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  settings = load_settings()
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--source", choices=("joystick", "synthetic"), default="joystick")
  p.add_argument("--list-axes", action="store_true", help="Print live axis values and exit")
  p.add_argument("--identify", action="store_true",
                 help="Work out which axis is throttle and which is brake, then exit")
  p.add_argument("--identify_timeout", type=float, default=30.0,
                 help="How long --identify waits for each pedal to be held")
  p.add_argument("--throttle_axis", type=int, default=settings.pedal_throttle_axis)
  p.add_argument("--brake_axis", type=int, default=settings.pedal_brake_axis)
  p.add_argument("--relay_host", default=settings.relay_host)
  p.add_argument("--relay_port", type=int, default=settings.relay_port)
  p.add_argument("--device_id", default=settings.pedal_device_id)
  p.add_argument("--hz", type=float, default=settings.send_hz)
  p.add_argument("--calibrate_seconds", type=float, default=3.0,
                 help="Length of each calibration capture window (0 skips calibration)")
  p.add_argument("--synthetic_period", type=float, default=20.0)
  p.add_argument("--synthetic_amplitude", type=float, default=0.6)
  p.add_argument("--duration", type=float, default=None, help="Exit after this many seconds")
  p.add_argument("--log_csv", default=None, help="Write every sent packet to this CSV")
  return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  if args.identify:
    return identify(args)
  if args.list_axes:
    return list_axes(args)

  source = build_source(args.source, throttle_axis=args.throttle_axis, brake_axis=args.brake_axis,
                        period_s=args.synthetic_period, amplitude=args.synthetic_amplitude)
  print(f"[pedal] source: {source.name}")

  if args.source == "synthetic" or args.calibrate_seconds <= 0.0:
    # The synthetic source already emits fractions in [0, 1].
    throttle_cal = brake_cal = AxisCalibration(rest=0.0, full=1.0)
  else:
    throttle_cal, brake_cal = calibrate(source, args.calibrate_seconds)

  log_file = None
  log_writer = None
  if args.log_csv:
    log_path = Path(os.path.expanduser(args.log_csv))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(SENT_CSV_COLUMNS)
    log_file.flush()

  pub = PedalPublisher(args.relay_host, args.relay_port, args.device_id)
  print(f"[pedal] publishing as {args.device_id!r} -> {args.relay_host}:{args.relay_port} "
        f"at {args.hz:.0f} Hz; Ctrl-C to stop")
  print("[pedal] NOTE: stopping this process denies the ego positive accel authority.")

  interval = 1.0 / args.hz
  t_start = time.monotonic()
  next_send = time.monotonic()
  last_status = 0.0

  try:
    while True:
      now = time.monotonic()
      if args.duration is not None and now - t_start >= args.duration:
        print("[pedal] --duration reached, stopping")
        break

      throttle_raw, brake_raw = source.read()
      throttle = throttle_cal.normalize(throttle_raw)
      brake = brake_cal.normalize(brake_raw)

      pub.maybe_hello()
      packet = pub.publish(throttle, brake, throttle_raw, brake_raw)
      if log_writer is not None:
        log_writer.writerow([wall_time_us(), packet.seq, f"{throttle:.4f}", f"{brake:.4f}",
                             f"{throttle_raw:.4f}", f"{brake_raw:.4f}"])

      if now - last_status >= 1.0:
        last_status = now
        print(f"[pedal] throttle={throttle:4.2f} brake={brake:4.2f}  "
              f"raw=({throttle_raw:+.3f}, {brake_raw:+.3f})  seq={packet.seq}")

      next_send += interval
      sleep_s = next_send - time.monotonic()
      if sleep_s > 0:
        time.sleep(sleep_s)
      else:
        next_send = time.monotonic()  # fell behind; don't burst
  except KeyboardInterrupt:
    print("\n[pedal] interrupted, stopping")
  finally:
    pub.close()
    source.close()
    if log_file is not None:
      log_file.close()
      print(f"[pedal] sent-packet log written to {args.log_csv}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
