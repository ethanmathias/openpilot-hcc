"""Run lifecycle: start a scenario, wait, stop everything, collect artifacts.

Everything a run produces lands in one folder under tools/hcc/runs/<run_id>/,
because the 2026-07-11 session showed that artifacts scattered across two
devices and a laptop are artifacts you cannot reconcile afterwards.
"""
from __future__ import annotations

import contextlib
import csv
import io
import json
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.tools.hcc.common import REPO_ROOT, RUNS_DIR, Settings
from openpilot.tools.hcc.fieldtest.remote import (
  CheckFailure,
  launch_cmd,
  remote_python,
  scp_from,
  ssh,
  ssh_ok,
  ssh_launch,
)
from openpilot.tools.hcc.fieldtest.preflight import preflight

# Remote script paths, relative to the openpilot checkout on a device.
VIRTUAL_LEAD = "tools/hcc/publishers/virtual_lead.py"
MONITOR = "tools/hcc/device/monitor.py"
BENCH_EGO = "tools/hcc/device/bench_ego.py"

# Patterns for `pkill -f` when tearing a run down.
REMOTE_PROCESSES = ("virtual_lead.py", "monitor.py", "bench_ego.py")

# Relay rows are pulled with margin either side of the run so a slightly
# mismatched clock cannot clip the window's edges.
COLLECT_MARGIN_US = 5_000_000


@dataclass
class RunOptions:
  scenario: int
  duration: float | None = None
  start_delay: float = 5.0
  end: str = "stop"
  loop: bool = False
  speed_scale: float = 1.0
  max_speed_mph: float | None = None
  monitor_hz: float = 10.0
  notes: str = ""
  skip_checks: bool = False
  bench: bool = False
  pedal: bool = False
  pedal_source: str = "joystick"
  lead_host: str | None = None
  extra: dict = field(default_factory=dict)


def _kill_remote(user: str, host: str, pattern: str) -> None:
  with contextlib.suppress(Exception):
    ssh(user, host, f"pkill -f {shlex.quote(pattern)}", timeout=10)


def abort(settings: Settings, lead_host: str) -> int:
  print("killing remote publishers and monitors...")
  _kill_remote(settings.lead.user, lead_host, "virtual_lead.py")
  for pattern in ("monitor.py", "bench_ego.py"):
    _kill_remote(settings.ego.user, settings.ego.host, pattern)
  print("done — the ego stops receiving V2V within ~300 ms and stops commanding accel.")
  print("NOTE: a local pedal publisher is not killed by this; stop it in its own terminal.")
  return 0


def _virtual_lead_flags(options: RunOptions, remote_sent_csv: str) -> str:
  flags = f"--scn {options.scenario} --end {options.end} --log_csv {remote_sent_csv}"
  if options.start_delay:
    flags += f" --start_delay {options.start_delay}"
  if options.duration:
    flags += f" --duration {options.duration}"
  if options.loop:
    flags += " --loop"
  if options.speed_scale != 1.0:
    flags += f" --speed_scale {options.speed_scale}"
  if options.max_speed_mph is not None:
    flags += f" --max_speed_mph {options.max_speed_mph}"
  return flags


def _start_pedal_publisher(settings: Settings, options: RunOptions, run_dir: Path) -> subprocess.Popen | None:
  """Start the pedal publisher locally -- it reads a rig attached to THIS laptop.

  Unlike the lead, this is not an SSH launch. Its log and sent-packet CSV go
  straight into the run folder.
  """
  log_path = run_dir / "pedal.log"
  cmd = [sys.executable, "tools/hcc/publishers/pedal.py",
         "--source", options.pedal_source,
         "--relay_host", settings.relay_host,
         "--relay_port", str(settings.relay_port),
         "--device_id", settings.pedal_device_id,
         "--log_csv", str(run_dir / "pedal_sent.csv")]
  log_file = log_path.open("w")
  proc = subprocess.Popen(cmd, cwd=str(REPO_ROOT), stdout=log_file, stderr=subprocess.STDOUT)
  time.sleep(1.5)
  if proc.poll() is not None:
    print(f"  WARNING: pedal publisher died at launch — see {log_path}")
    print("  " + (log_path.read_text().strip().splitlines() or ["(no output)"])[-1])
    return None
  print(f"laptop: pedal publisher started (pid {proc.pid}, source={options.pedal_source})")
  return proc


def run(settings: Settings, options: RunOptions) -> int:
  lead_host = options.lead_host or settings.lead.host
  ego, lead = settings.ego, settings.lead
  log_dir = settings.remote_log_dir

  if not options.skip_checks:
    if preflight(settings, lead_host, check_pedal=options.pedal) > 0:
      print("\npreflight failed — fix the FAILs above, or re-run with --skip_checks "
            "if you know better.")
      return 1
    print()

  run_id = time.strftime("run_%Y%m%d_%H%M%S") + f"_scn{options.scenario}"
  run_dir = RUNS_DIR / run_id
  run_dir.mkdir(parents=True, exist_ok=True)
  print(f"run id:  {run_id}")
  print(f"run dir: {run_dir}")

  remote = {
    "sent_csv": f"{log_dir}/sent_{run_id}.csv",
    "vl_log": f"{log_dir}/vl_{run_id}.log",
    "monitor_csv": f"{log_dir}/monitor_{run_id}.csv",
    "monitor_log": f"{log_dir}/monitor_{run_id}.log",
    "bench_log": f"{log_dir}/bench_{run_id}.log",
  }

  pedal_proc = _start_pedal_publisher(settings, options, run_dir) if options.pedal else None

  if options.bench:
    # No car: the real ego subscriber lives inside controlsd and never starts
    # offroad, so the relay would have no ego peer to forward to.
    ego_py = remote_python(ego.user, ego.host, ego.repo)
    bench_pid = ssh_launch(ego.user, ego.host,
                           launch_cmd(ego.repo, log_dir, ego_py, BENCH_EGO, remote["bench_log"]),
                           timeout=30)
    print(f"ego: BENCH subscriber started (pid {bench_pid}) — never use --bench with a real car")

  relay_remote = ssh(ego.user, ego.host,
                     f"ls -t {log_dir}/relay_*.csv 2>/dev/null | head -1").stdout.strip()
  if not relay_remote:
    print("WARNING: no relay CSV on the ego — is relay logging active? Continuing without it.")

  # Run bounds on the EGO's clock, because relay rows are stamped with it.
  t_start_us = int(ssh_ok(ego.user, ego.host, "date +%s%N")) // 1000

  ego_py = remote_python(ego.user, ego.host, ego.repo)
  monitor_flags = f"{MONITOR} --log_csv {remote['monitor_csv']} --hz {options.monitor_hz}"
  if relay_remote:
    monitor_flags += f" --relay_csv {relay_remote}"
  monitor_pid = ssh_launch(ego.user, ego.host,
                           launch_cmd(ego.repo, log_dir, ego_py, monitor_flags, remote["monitor_log"]),
                           timeout=30)
  print(f"ego: monitor started (pid {monitor_pid}) -> {remote['monitor_csv']}")

  vl_flags = _virtual_lead_flags(options, remote["sent_csv"])
  lead_py = remote_python(lead.user, lead_host, lead.repo)
  try:
    vl_pid = ssh_launch(lead.user, lead_host,
                        launch_cmd(lead.repo, log_dir, lead_py, f"{VIRTUAL_LEAD} {vl_flags}", remote["vl_log"]),
                        timeout=30)
  except (CheckFailure, subprocess.TimeoutExpired):
    _teardown(settings, lead_host, pedal_proc)
    raise
  print(f"lead: virtual_lead started (pid {vl_pid}): {vl_flags}")
  print("\n>>> scenario is LIVE — engage now. Ctrl-C ends the run and collects data. <<<\n")

  stop_requested = False

  def _request_stop(signum, frame):
    nonlocal stop_requested
    stop_requested = True

  old_term = signal.signal(signal.SIGTERM, _request_stop)
  try:
    while not stop_requested:
      time.sleep(3.0)
      if pedal_proc is not None and pedal_proc.poll() is not None:
        print("  WARNING: the pedal publisher exited — the ego is now denying positive accel.")
        pedal_proc = None
      alive = ssh(lead.user, lead_host, f"kill -0 {vl_pid} 2>/dev/null").returncode == 0
      tail = ssh(lead.user, lead_host, f"tail -1 {remote['vl_log']}").stdout.strip()
      if tail:
        print(f"  lead: {tail}")
      if not alive:
        print("virtual_lead finished on its own.")
        break
  except KeyboardInterrupt:
    print("\nrun stopped by user.")
  finally:
    signal.signal(signal.SIGTERM, old_term)
    _teardown(settings, lead_host, pedal_proc)

  t_end_us = int(ssh_ok(ego.user, ego.host, "date +%s%N")) // 1000
  print(f"run duration: {(t_end_us - t_start_us) / 1e6:.1f} s — collecting artifacts...")

  collected = collect_artifacts(settings, lead_host, run_dir, relay_remote,
                                t_start_us, t_end_us, remote)
  if options.bench:
    collected["bench_ego.log"] = scp_from(ego.user, ego.host, remote["bench_log"],
                                          run_dir / "bench_ego.log")

  metadata = {
    "run_id": run_id,
    "scenario": options.scenario,
    "ego_host": ego.host,
    "lead_host": lead_host,
    "start_ego_wall_time_us": t_start_us,
    "end_ego_wall_time_us": t_end_us,
    "duration_s": round((t_end_us - t_start_us) / 1e6, 2),
    "virtual_lead_flags": vl_flags,
    "monitor_hz": options.monitor_hz,
    "relay_csv_on_ego": relay_remote,
    "bench_mode": options.bench,
    "pedal_mode": options.pedal,
    "pedal_source": options.pedal_source if options.pedal else None,
    "collected": collected,
    "notes": options.notes,
  }
  (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")

  analyze(run_dir)
  print(f"\nrun complete — everything is in {run_dir}")
  return 0


def _teardown(settings: Settings, lead_host: str, pedal_proc: subprocess.Popen | None) -> None:
  _kill_remote(settings.lead.user, lead_host, "virtual_lead.py")
  for pattern in ("monitor.py", "bench_ego.py"):
    _kill_remote(settings.ego.user, settings.ego.host, pattern)
  if pedal_proc is not None and pedal_proc.poll() is None:
    pedal_proc.terminate()
    with contextlib.suppress(subprocess.TimeoutExpired):
      pedal_proc.wait(timeout=5)


def extract_relay_slice(full_csv: Path, out_csv: Path, t_start_us: int, t_end_us: int) -> int:
  """Copy relay rows within [t_start_us, t_end_us] (ego-clock us). Returns row count."""
  count = 0
  with open(full_csv, newline="") as src, open(out_csv, "w", newline="") as dst:
    reader = csv.reader(src)
    writer = csv.writer(dst)
    header = next(reader, None)
    if header:
      writer.writerow(header)
    for row in reader:
      try:
        t = int(row[0])
      except (ValueError, IndexError):
        continue
      if t_start_us <= t <= t_end_us:
        writer.writerow(row)
        count += 1
  return count


def collect_artifacts(settings: Settings, lead_host: str, run_dir: Path, relay_remote: str,
                      t_start_us: int, t_end_us: int, remote: dict[str, str]) -> dict[str, bool]:
  ego, lead = settings.ego, settings.lead
  collected: dict[str, bool] = {
    "sent.csv": scp_from(lead.user, lead_host, remote["sent_csv"], run_dir / "sent.csv"),
    "virtual_lead.log": scp_from(lead.user, lead_host, remote["vl_log"], run_dir / "virtual_lead.log"),
    "ego_monitor.csv": scp_from(ego.user, ego.host, remote["monitor_csv"], run_dir / "ego_monitor.csv"),
  }

  if relay_remote:
    full = run_dir / "relay_full.csv"
    if scp_from(ego.user, ego.host, relay_remote, full):
      rows = extract_relay_slice(full, run_dir / "relay.csv",
                                 t_start_us - COLLECT_MARGIN_US, t_end_us + COLLECT_MARGIN_US)
      full.unlink()
      collected["relay.csv"] = rows > 0
      print(f"  relay.csv: {rows} rows in the run window")
    else:
      collected["relay.csv"] = False

  for name, ok in collected.items():
    if not ok:
      print(f"  WARNING: failed to collect {name}")
  return collected


def analyze(run_dir: Path) -> None:
  """Stats and a plot from the collected relay slice. Never fails the run."""
  relay_csv = run_dir / "relay.csv"
  if not relay_csv.is_file():
    return
  from openpilot.tools.hcc.analysis import plot_run
  argv = [str(relay_csv), "--out", str(run_dir / "plot.png")]
  if (run_dir / "sent.csv").is_file():
    argv += ["--sent", str(run_dir / "sent.csv")]
  buffer = io.StringIO()
  try:
    with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
      plot_run.main(argv)
  except (SystemExit, Exception) as exc:  # noqa: BLE001 — analysis is best-effort
    buffer.write(f"\nanalysis incomplete: {exc}\n")
  stats = buffer.getvalue()
  (run_dir / "stats.txt").write_text(stats)
  print(stats.rstrip())


def collect(settings: Settings, lead_host: str, run_id: str) -> int:
  """Re-pull device logs for a run that ended uncleanly."""
  run_dir = RUNS_DIR / run_id
  meta_path = run_dir / "metadata.json"
  if meta_path.is_file():
    meta = json.loads(meta_path.read_text())
    t_start_us, t_end_us = meta["start_ego_wall_time_us"], meta["end_ego_wall_time_us"]
  else:
    run_dir.mkdir(parents=True, exist_ok=True)
    t_start_us, t_end_us = 0, 2**63 - 1
    print("no metadata.json — pulling the full relay log instead of a time slice")

  log_dir = settings.remote_log_dir
  relay_remote = ssh(settings.ego.user, settings.ego.host,
                     f"ls -t {log_dir}/relay_*.csv 2>/dev/null | head -1").stdout.strip()
  collected = collect_artifacts(settings, lead_host, run_dir, relay_remote, t_start_us, t_end_us, {
    "sent_csv": f"{log_dir}/sent_{run_id}.csv",
    "vl_log": f"{log_dir}/vl_{run_id}.log",
    "monitor_csv": f"{log_dir}/monitor_{run_id}.csv",
  })
  analyze(run_dir)
  print(f"collected into {run_dir}: {collected}")
  return 0
