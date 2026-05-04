#!/usr/bin/env python3
"""Top-level PC orchestrator for HIL mode.

Convenience wrapper around tools/sim/run_bridge.py that defaults to
--mode hcc_hil and auto-locates devices.toml. Lives separately so the
HIL bring-up command stays:

    python -m tools.sim.hil.launch_pc

instead of the longer:

    python tools/sim/run_bridge.py --mode hcc_hil --devices_toml ...

Wheel/keyboard input dispatch is delegated to run_bridge.parse_args' existing
logitech_wheel/keyboard plumbing.
"""
from __future__ import annotations

import argparse
import os
import sys

from openpilot.tools.sim.run_bridge import create_bridge


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="HIL bring-up: MetaDrive on PC, openpilot on each Comma 3X.")
  parser.add_argument("--devices_toml", default=None, help="Path to tools/sim/hil/devices.toml")
  parser.add_argument("--raw_yuv", action="store_true", help="Bring-up: ship NV12 instead of H.264")
  parser.add_argument("--lead", action="store_true", help="Enable the lead device (two-vehicle HIL)")
  parser.add_argument("--dual_camera", action="store_true")
  parser.add_argument("--high_quality", action="store_true")
  parser.add_argument("--keyboard", action="store_true", help="Force keyboard input instead of wheel")
  parser.add_argument("--wheel_device", default=None, help="Optional /dev/input/eventX path (Linux only)")
  parser.add_argument("--wheel_hz", type=float, default=100.0)
  args = parser.parse_args(argv)

  here = os.path.dirname(os.path.abspath(__file__))
  if args.devices_toml is None:
    args.devices_toml = os.path.join(here, "devices.toml")
  if not os.path.exists(args.devices_toml):
    sys.stderr.write(f"[launch_pc] devices.toml not found at {args.devices_toml}; copy devices.example.toml and edit.\n")
    return 1

  queue, simulator_process, simulator_bridge = create_bridge(
    args.dual_camera, args.high_quality, mode="hcc_hil",
    devices_toml=args.devices_toml, raw_yuv=args.raw_yuv, with_lead=args.lead,
  )

  if args.keyboard:
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(queue)
  else:
    try:
      from openpilot.tools.sim.lib.logitech_wheel_ctrl import logitech_wheel_poll_thread
      logitech_wheel_poll_thread(queue, device_path=args.wheel_device, publish_hz=args.wheel_hz)
    except RuntimeError as err:
      print(f"[launch_pc] wheel unavailable: {err}; falling back to keyboard")
      from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
      keyboard_poll_thread(queue)

  simulator_bridge.shutdown()
  simulator_process.join()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
