"""The `hcc` command.

Everything defaults from config/devices.toml, so a configured setup needs no
flags. Every default can still be overridden per invocation.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from openpilot.tools.hcc.common import RUNS_DIR, load_settings
from openpilot.tools.hcc.fieldtest.preflight import find_lead, preflight
from openpilot.tools.hcc.fieldtest.remote import CheckFailure
from openpilot.tools.hcc.fieldtest.session import RunOptions, abort, collect, run

DESCRIPTION = """\
HCC field test driver. Run from the laptop, joined to the ego's hotspot.

  hcc preflight                 are both devices ready?
  hcc find-lead                 what can the ego's hotspot see?
  hcc run                       start a logged run (Ctrl-C ends and collects)
  hcc run --pedal               ... with the pedal rig as the manual input
  hcc run --bench               ... with no car, using a stand-in ego
  hcc abort                     kill the remote processes now, collect nothing
  hcc collect <run_id>          re-pull artifacts for a run that ended badly
  hcc plot <run_id>             link-health statistics and plots
  hcc pedal                     just the pedal publisher, nothing else

Hosts, users, repo paths, ports and device ids come from
tools/hcc/config/devices.toml.
"""


def _add_lead_host(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--lead_host", default=None,
                      help="Override the lead address from devices.toml")


def build_parser() -> argparse.ArgumentParser:
  settings = load_settings()
  parser = argparse.ArgumentParser(prog="hcc", description=DESCRIPTION,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest="command", required=True)

  p_pre = sub.add_parser("preflight", aliases=["check"], help="Pre-run checks")
  _add_lead_host(p_pre)
  p_pre.add_argument("--pedal", action="store_true", help="Also check the pedal stream")

  sub.add_parser("find-lead", help="List devices on the ego's hotspot")

  p_run = sub.add_parser("run", help="Run one scenario end to end")
  _add_lead_host(p_run)
  p_run.add_argument("--scn", "--scenario", type=int, default=settings.scenario, dest="scenario",
                     help=f"Scenario column (default {settings.scenario})")
  p_run.add_argument("--duration", type=float, default=None, help="Cap the run length in seconds")
  p_run.add_argument("--start_delay", type=float, default=5.0,
                     help="Seconds at the initial speed before the profile moves")
  p_run.add_argument("--end", choices=("hold", "stop"), default="stop",
                     help="After the profile: hold the final speed, or stop publishing")
  p_run.add_argument("--loop", action="store_true", help="Loop the profile until Ctrl-C")
  p_run.add_argument("--speed_scale", type=float, default=1.0, help="Scale the whole profile")
  p_run.add_argument("--max_speed_mph", type=float, default=settings.max_speed_mph,
                     help=f"Cap the profile's top speed (default {settings.max_speed_mph})")
  p_run.add_argument("--monitor_hz", type=float, default=settings.monitor_hz)
  p_run.add_argument("--notes", default="", help="Saved in metadata.json (conditions, location, ...)")
  p_run.add_argument("--skip_checks", action="store_true", help="Skip the preflight")
  p_run.add_argument("--bench", action="store_true",
                     help="No-car bench run: start a stand-in ego subscriber on the ego so the "
                          "relay has somewhere to forward. NEVER use with a real car.")
  p_run.add_argument("--pedal", action="store_true",
                     help="Also start the pedal publisher on this laptop")
  p_run.add_argument("--pedal_source", choices=("joystick", "synthetic"), default="joystick")

  p_abort = sub.add_parser("abort", help="Kill the remote processes now")
  _add_lead_host(p_abort)

  p_collect = sub.add_parser("collect", help="Re-pull artifacts for a past run")
  _add_lead_host(p_collect)
  p_collect.add_argument("run_id", help="e.g. run_20260612_153000_scn48")

  p_plot = sub.add_parser("plot", help="Statistics and plots for a run")
  p_plot.add_argument("run", help="A run id under tools/hcc/runs/, or a path to a relay CSV")
  p_plot.add_argument("--no_plot", action="store_true", help="Statistics only")

  p_pedal = sub.add_parser("pedal", help="Run only the pedal publisher")
  p_pedal.add_argument("--source", choices=("joystick", "synthetic"), default="joystick")
  p_pedal.add_argument("--list-axes", action="store_true", help="Print live axis values and exit")
  p_pedal.add_argument("--identify", action="store_true",
                       help="Work out which axis is throttle and which is brake, then exit")
  p_pedal.add_argument("rest", nargs=argparse.REMAINDER,
                       help="Further flags passed through to publishers/pedal.py")

  return parser


def _plot(args) -> int:
  from openpilot.tools.hcc.analysis import plot_run

  target = Path(args.run).expanduser()
  if target.is_file():
    argv = [str(target)]
  else:
    run_dir = target if target.is_dir() else RUNS_DIR / args.run
    relay_csv = run_dir / "relay.csv"
    if not relay_csv.is_file():
      print(f"no relay.csv in {run_dir}", file=sys.stderr)
      return 1
    argv = [str(relay_csv), "--out", str(run_dir / "plot.png")]
    for name, flag in (("sent.csv", "--sent"), ("pedal_sent.csv", "--pedal_sent")):
      if (run_dir / name).is_file():
        argv += [flag, str(run_dir / name)]
  if args.no_plot:
    argv.append("--no_plot")
  return plot_run.main(argv)


def _pedal(args) -> int:
  from openpilot.tools.hcc.publishers import pedal

  argv = ["--source", args.source]
  if args.identify:
    argv.append("--identify")
  if args.list_axes:
    argv.append("--list-axes")
  argv += [a for a in (args.rest or []) if a != "--"]
  return pedal.main(argv)


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  settings = load_settings()
  lead_host = getattr(args, "lead_host", None) or settings.lead.host

  try:
    if args.command in ("preflight", "check"):
      return 1 if preflight(settings, lead_host, check_pedal=args.pedal) > 0 else 0
    if args.command == "find-lead":
      return find_lead(settings)
    if args.command == "run":
      options = RunOptions(
        scenario=args.scenario, duration=args.duration, start_delay=args.start_delay,
        end=args.end, loop=args.loop, speed_scale=args.speed_scale,
        max_speed_mph=args.max_speed_mph, monitor_hz=args.monitor_hz, notes=args.notes,
        skip_checks=args.skip_checks, bench=args.bench, pedal=args.pedal,
        pedal_source=args.pedal_source, lead_host=lead_host,
      )
      return run(settings, options)
    if args.command == "abort":
      return abort(settings, lead_host)
    if args.command == "collect":
      return collect(settings, lead_host, args.run_id)
    if args.command == "plot":
      return _plot(args)
    if args.command == "pedal":
      return _pedal(args)
  except CheckFailure as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 1
  except subprocess.TimeoutExpired as exc:
    print(f"error: SSH timed out: {exc}", file=sys.stderr)
    return 1
  return 1


if __name__ == "__main__":
  raise SystemExit(main())
