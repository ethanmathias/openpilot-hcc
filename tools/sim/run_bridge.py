#!/usr/bin/env python3
import argparse
import os

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

SIM_MODE_TO_SCENARIO = {
  "default": "default",
  "hc3": "hccc_step",
  "hcc_hil": "hccc_step",
}


def _resolve_scn_csv_path(scn_csv: str | None) -> str | None:
  if scn_csv:
    return os.path.expanduser(scn_csv)

  sim_dir = os.path.dirname(os.path.abspath(__file__))
  candidate_paths = [
    os.path.join(sim_dir, "lib", "Scenarios.csv"),
    os.path.join(sim_dir, "bridge", "lib", "Scenarios.csv"),
  ]
  for path in candidate_paths:
    if os.path.isfile(path):
      return path
  return None


def create_bridge(dual_camera, high_quality, mode="default", scn=None, scn_csv=None, output_csv=None, output_graph=None,
                  lead_prefix=None, devices_toml=None, raw_yuv=False, with_lead=False):
  queue: Any = Queue()

  if mode == "hcc_hil":
    # HIL mode owns its own per-role OPENPILOT_PREFIX contexts; don't set one
    # on the bridge process itself.
    from openpilot.tools.sim.bridge.metadrive.metadrive_hil_bridge import MetaDriveHILBridge
    from openpilot.tools.sim.hil.device_config import load as load_devices, warn_on_mismatch
    if devices_toml is None:
      sim_dir = os.path.dirname(os.path.abspath(__file__))
      devices_toml = os.path.join(sim_dir, "hil", "devices.toml")
    cfg = load_devices(devices_toml)
    warn_on_mismatch(cfg)

    lead_dev = cfg.lead if with_lead else None
    simulator_bridge = MetaDriveHILBridge(dual_camera, high_quality, ego_device=cfg.ego, lead_device=lead_dev,
                                          raw_yuv=raw_yuv, scenario=SIM_MODE_TO_SCENARIO[mode],
                                          scn=scn, scn_csv=scn_csv, output_csv=output_csv, output_graph=output_graph)
    simulator_process = simulator_bridge.run(queue)
    return queue, simulator_process, simulator_bridge

  # hc3 mode and dual-sim V2V both expect openpilot running under the 'hccego'
  # prefix (launch_openpilot_ego.sh exports OPENPILOT_PREFIX=hccego).  Set it
  # here so the bridge's cereal sockets land in the same namespace.
  if (mode == "hc3" or lead_prefix is not None) and os.getenv('OPENPILOT_PREFIX') is None:
    os.environ['OPENPILOT_PREFIX'] = 'hccego'

  if scn is not None and scn < 1:
    raise RuntimeError("`--scn` must be >= 1.")

  scn_csv = _resolve_scn_csv_path(scn_csv) if scn is not None else None
  if scn is not None and scn_csv is None:
    raise RuntimeError("`--scn` requires a CSV file. Place it at tools/sim/lib/Scenarios.csv or pass --scn_csv.")

  simulator_bridge = MetaDriveBridge(dual_camera, high_quality,
                                     scenario=SIM_MODE_TO_SCENARIO[mode], enable_hcc=False,
                                     scn=scn, scn_csv=scn_csv, output_csv=output_csv, output_graph=output_graph,
                                     lead_sim_prefix=lead_prefix)
  simulator_process = simulator_bridge.run(queue)

  return queue, simulator_process, simulator_bridge

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--keyboard', action='store_true',
                      help='Force keyboard input instead of auto-preferring a Logitech wheel')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--logitech_wheel', action='store_true',
                      help='Use Logitech wheel/pedals via /dev/input/event* (Linux); now the default unless --keyboard or --joystick is used')
  parser.add_argument('--wheel_device', default=None,
                      help='Optional event device path, e.g. /dev/input/event5')
  parser.add_argument('--wheel_hz', type=float, default=100.0,
                      help='Wheel command publish rate (Hz) when using --logitech_wheel')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--mode', default="default", choices=["default", "hc3", "hcc_hil"],
                      help='Simulation mode preset: default, hc3, or hcc_hil (hardware-in-the-loop with Comma 3X devices)')
  parser.add_argument('--devices_toml', default=None,
                      help='Path to tools/sim/hil/devices.toml (hcc_hil mode only)')
  parser.add_argument('--raw_yuv', action='store_true',
                      help='Bring-up fallback: ship NV12 frames to devices instead of H.264 (hcc_hil mode only)')
  parser.add_argument('--lead', action='store_true',
                      help='Enable the lead Comma 3X device (two-vehicle HIL; hcc_hil mode only)')
  parser.add_argument('--scn', type=int, default=None,
                      help='Replay lead trajectory from scenario column index in Scenarios.csv (e.g., 48)')
  parser.add_argument('--scn_csv', default=None,
                      help='Path to scenario CSV file (defaults to tools/sim/lib/Scenarios.csv if present)')
  parser.add_argument('--output_csv', default=None,
                      help='Path to write bridge telemetry CSV output')
  parser.add_argument('--output_graph', default=None,
                      help='Path to write a PNG speed plot (defaults next to the CSV output)')
  parser.add_argument('--lead_prefix', default=None,
                      help='Optional second OPENPILOT_PREFIX to feed shared-world lead-state CAN into for dual-sim V2V testing')

  return parser.parse_args(add_args)


def run_input_poll(queue, *, use_keyboard: bool = False, use_joystick: bool = False,
                   require_wheel: bool = False, wheel_device: str | None = None,
                   wheel_hz: float = 100.0) -> None:
  """Start the blocking input-poll thread. Tries Logitech wheel first unless
  keyboard or joystick is explicitly requested. Falls back to keyboard if the
  wheel is unavailable and not explicitly required."""
  if use_joystick:
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(queue)
    return

  if use_keyboard:
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(queue)
    return

  # Default: try wheel, fall back to keyboard
  try:
    from openpilot.tools.sim.lib.logitech_wheel_ctrl import logitech_wheel_poll_thread
    logitech_wheel_poll_thread(queue, device_path=wheel_device, publish_hz=wheel_hz)
  except RuntimeError as err:
    if require_wheel:
      raise
    print(f"[logitech] {err}")
    print("[logitech] Falling back to keyboard input. Use --keyboard to skip wheel autodetect.")
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(queue)


if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality,
                                                             mode=args.mode, scn=args.scn, scn_csv=args.scn_csv,
                                                             output_csv=args.output_csv, output_graph=args.output_graph,
                                                             lead_prefix=args.lead_prefix,
                                                             devices_toml=args.devices_toml, raw_yuv=args.raw_yuv,
                                                             with_lead=args.lead)

  run_input_poll(queue, use_keyboard=args.keyboard, use_joystick=args.joystick,
                 require_wheel=args.logitech_wheel, wheel_device=args.wheel_device,
                 wheel_hz=args.wheel_hz)

  simulator_bridge.shutdown()

  simulator_process.join()
