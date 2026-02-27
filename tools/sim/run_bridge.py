#!/usr/bin/env python3
import argparse

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

SIM_MODE_TO_SCENARIO = {
  "default": "default",
  "hc3": "hccc_step",
}


def create_bridge(dual_camera, high_quality, mode="default"):
  queue: Any = Queue()

  simulator_bridge = MetaDriveBridge(dual_camera, high_quality, scenario=SIM_MODE_TO_SCENARIO[mode], enable_hcc=False)
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

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality, mode=args.mode)

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
