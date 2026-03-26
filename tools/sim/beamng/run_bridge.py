#!/usr/bin/env python3
import argparse
import os

from multiprocessing import Queue

from openpilot.tools.sim.beamng.bridge import BRIDGE_MODE_OPENPILOT, BRIDGE_MODE_REFERENCE, BeamNGBridge
from openpilot.tools.sim.beamng.world import BeamNGConfig


def _resolve_scn_csv_path(scn_csv: str | None) -> str | None:
  if scn_csv:
    return os.path.expanduser(scn_csv)

  repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
  candidates = [
    os.path.join(repo_root, "tools", "sim", "lib", "Scenarios.csv"),
    os.path.join(repo_root, "hccintegration", "scn", "Scenarios.csv"),
  ]
  for candidate in candidates:
    if os.path.isfile(candidate):
      return candidate
  return None


def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description="Standalone BeamNG bridge for openpilot.")
  parser.add_argument("--keyboard", action="store_true",
                      help="Force keyboard input instead of auto-preferring a Logitech wheel")
  parser.add_argument("--joystick", action="store_true")
  parser.add_argument("--logitech_wheel", action="store_true",
                      help="Use Logitech wheel/pedals via /dev/input/event*")
  parser.add_argument("--wheel_device", default=None)
  parser.add_argument("--wheel_hz", type=float, default=100.0)
  parser.add_argument("--dual_camera", action="store_true")
  parser.add_argument("--high_quality", action="store_true")
  parser.add_argument("--mode", default="default", choices=["default", "hc3"],
                      help="Use openpilot default longitudinal mapping or HC3-like longitudinal mapping in openpilot mode")
  parser.add_argument("--bridge_mode", default=BRIDGE_MODE_OPENPILOT, choices=[BRIDGE_MODE_OPENPILOT, BRIDGE_MODE_REFERENCE])
  parser.add_argument("--hc3_variant", default="new", choices=["original", "new"])
  parser.add_argument("--hc3_controller_path", default=None)
  parser.add_argument("--beamng_host", default="localhost")
  parser.add_argument("--beamng_port", type=int, default=64256)
  parser.add_argument("--beamng_home", default=None)
  parser.add_argument("--beamng_map", default="west_coast_usa")
  parser.add_argument("--beamng_scenario", default="OpenpilotBeamNG")
  parser.add_argument("--beamng_vehicle", default="etk800")
  parser.add_argument("--beamng_lead_vehicle", default=None)
  parser.add_argument("--scn", type=int, default=None,
                      help="Replay lead trajectory from scenario column index in Scenarios.csv")
  parser.add_argument("--scn_csv", default=None)
  parser.add_argument("--output_csv", default=None)
  return parser.parse_args(add_args)


def create_bridge(args):
  scenario_csv = _resolve_scn_csv_path(args.scn_csv) if args.scn is not None else None
  if args.scn is not None and scenario_csv is None:
    raise RuntimeError("`--scn` requires a scenario CSV file")

  config = BeamNGConfig(
    host=args.beamng_host,
    port=args.beamng_port,
    home=args.beamng_home,
    map_name=args.beamng_map,
    scenario_name=args.beamng_scenario,
    ego_vehicle_model=args.beamng_vehicle,
    lead_vehicle_model=args.beamng_lead_vehicle or args.beamng_vehicle,
    step_s=0.01,
    scenario_index=args.scn,
    scenario_csv=scenario_csv,
  )
  return BeamNGBridge(
    dual_camera=args.dual_camera,
    high_quality=args.high_quality,
    enable_hcc=(args.mode == "hc3"),
    bridge_mode=args.bridge_mode,
    hc3_variant=args.hc3_variant,
    hc3_controller_path=args.hc3_controller_path,
    beamng_config=config,
    output_csv=args.output_csv,
  )


def main():
  args = parse_args()
  queue = Queue()
  bridge = create_bridge(args)
  process = bridge.run(queue)

  use_logitech_wheel = not args.keyboard and not args.joystick
  if args.logitech_wheel:
    use_logitech_wheel = True

  if use_logitech_wheel:
    try:
      from openpilot.tools.sim.lib.logitech_wheel_ctrl import logitech_wheel_poll_thread
      logitech_wheel_poll_thread(queue, device_path=args.wheel_device, publish_hz=args.wheel_hz)
    except RuntimeError as err:
      if args.logitech_wheel:
        raise
      print(f"[logitech] {err}")
      print("[logitech] Falling back to keyboard input.")
      from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
      keyboard_poll_thread(queue)
  elif args.joystick:
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(queue)
  else:
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(queue)

  bridge.shutdown()
  process.join()


if __name__ == "__main__":
  main()
