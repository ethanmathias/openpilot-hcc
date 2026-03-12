#!/usr/bin/env python3
import argparse
import os

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

SIM_MODE_TO_SCENARIO = {
  "default": "default",
  "hc3": "hccc_step",
}


def _resolve_scn_csv_path(scn_csv: str | None) -> str | None:
  if scn_csv:
    return os.path.expanduser(scn_csv)

  sim_dir = os.path.dirname(os.path.abspath(__file__))
  candidate_paths = [
    os.path.join(sim_dir, "lib", "Scenarios.csv"),
    os.path.join(sim_dir, "bridge", "lib", "Scenarios.csv"),
    os.path.expanduser("~/Downloads/hc3-manual-code/scn/Scenarios.csv"),
  ]
  for path in candidate_paths:
    if os.path.isfile(path):
      return path
  return None


def create_bridge(dual_camera, high_quality, mode="default", scn=None, scn_csv=None, output_csv=None):
  queue: Any = Queue()

  if scn is not None and scn < 1:
    raise RuntimeError("`--scn` must be >= 1.")

  scn_csv = _resolve_scn_csv_path(scn_csv) if scn is not None else None
  if scn is not None and scn_csv is None:
    raise RuntimeError("`--scn` requires a CSV file. Place it at tools/sim/lib/Scenarios.csv or pass --scn_csv.")

  simulator_bridge = MetaDriveBridge(dual_camera, high_quality,
                                     scenario=SIM_MODE_TO_SCENARIO[mode], enable_hcc=False,
                                     scn=scn, scn_csv=scn_csv, output_csv=output_csv)
  simulator_process = simulator_bridge.run(queue)

  return queue, simulator_process, simulator_bridge

def main():
  _, simulator_process, _ = create_bridge(True, False)
  simulator_process.join()

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--logitech_wheel', action='store_true',
                      help='Use Logitech wheel/pedals via /dev/input/event* (Linux)')
  parser.add_argument('--wheel_device', default=None,
                      help='Optional event device path, e.g. /dev/input/event5')
  parser.add_argument('--wheel_hz', type=float, default=100.0,
                      help='Wheel command publish rate (Hz) when using --logitech_wheel')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--mode', default="default", choices=["default", "hc3"],
                      help='Simulation mode preset: default or hc3')
  parser.add_argument('--scn', type=int, default=None,
                      help='Replay lead trajectory from scenario column index in Scenarios.csv (e.g., 48)')
  parser.add_argument('--scn_csv', default=None,
                      help='Path to scenario CSV file (defaults to tools/sim/lib/Scenarios.csv if present)')
  parser.add_argument('--output_csv', default=None,
                      help='Path to write bridge telemetry CSV output')

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality,
                                                             mode=args.mode, scn=args.scn, scn_csv=args.scn_csv,
                                                             output_csv=args.output_csv)

  if args.logitech_wheel:
    from openpilot.tools.sim.lib.logitech_wheel_ctrl import logitech_wheel_poll_thread
    logitech_wheel_poll_thread(queue, device_path=args.wheel_device, publish_hz=args.wheel_hz)
  elif args.joystick:
    # start input poll for joystick
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

    wheel_poll_thread(queue)
  else:
    # start input poll for keyboard
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    keyboard_poll_thread(queue)

  simulator_bridge.shutdown()

  simulator_process.join()
