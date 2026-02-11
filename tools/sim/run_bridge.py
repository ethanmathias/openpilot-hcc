#!/usr/bin/env python3
import argparse

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

def create_bridge(dual_camera, high_quality, scenario, manual_no_lead=False):
  queue: Any = Queue()

  simulator_bridge = MetaDriveBridge(dual_camera, high_quality, scenario=scenario,
                                     force_engage_on_startup=not manual_no_lead)
  simulator_process = simulator_bridge.run(queue)

  return queue, simulator_process, simulator_bridge

def main():
  _, simulator_process, _ = create_bridge(True, False, "default", False)
  simulator_process.join()

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--scenario', default="default", choices=["default", "hccc_step"])
  parser.add_argument('--manual_no_lead', action='store_true',
                      help="Disable auto-engage and run without virtual lead injection")

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality,
                                                             args.scenario, args.manual_no_lead)

  if args.joystick:
    # start input poll for joystick
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

    wheel_poll_thread(queue)
  else:
    # start input poll for keyboard
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    keyboard_poll_thread(queue)

  simulator_bridge.shutdown()

  simulator_process.join()
