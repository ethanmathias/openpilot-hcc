#!/usr/bin/env python3
"""PC-side orchestrator for real-world (in-car) HC3 tests.

Runs on your laptop, joined to the ego device's WiFi hotspot. Everything on
the devices is started and stopped over SSH, and every artifact from a run is
pulled back into one folder under tools/real_world_testing/runs/<run_id>/ so
the run can be reviewed later.

Subcommands:

    check       Preflight: SSH, relay service, params, clock skew, scenario CSV
    find-lead   Discover the lead device's IP on the ego's hotspot
    run         Start a scenario run, wait for it to finish, collect everything
    abort       Emergency: kill virtual_lead/monitor on the devices
    collect     Re-pull device logs for a run that ended uncleanly

Typical session:

    python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.34
    python3 tools/real_world_testing/field_test.py run --scn 48 --duration 60 \
        --lead_host 10.42.0.34 --notes "parking lot, dry, first ramp test"

See tools/real_world_testing/README.md for the full procedure.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import select
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

RUNS_DIR = Path(__file__).resolve().parent / "runs"
DEFAULT_EGO_HOST = "10.42.0.1"
DEFAULT_USER = "comma"
DEFAULT_REMOTE_DIR = "/data/openpilot"
REMOTE_LOG_DIR = "/data/hcc_v2v_logs"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new"]
CLOCK_SKEW_WARN_MS = 150.0   # subscriber/relay reject packets beyond 500 ms skew
CLOCK_SKEW_FAIL_MS = 400.0


class CheckFailure(Exception):
  pass


def ssh(user: str, host: str, command: str, timeout: float = 20.0) -> subprocess.CompletedProcess:
  return subprocess.run(["ssh", *SSH_OPTS, f"{user}@{host}", command],
                        capture_output=True, text=True, timeout=timeout)


def ssh_ok(user: str, host: str, command: str, timeout: float = 20.0) -> str:
  result = ssh(user, host, command, timeout)
  if result.returncode != 0:
    raise CheckFailure(f"ssh {host} `{command}` failed: {result.stderr.strip() or result.stdout.strip()}")
  return result.stdout.strip()


def launch_cmd(remote_dir: str, py: str, script_and_flags: str, log_path: str) -> str:
  """Remote command that starts a nohup'd background python and prints its
  PID — or DEAD if it didn't survive its first second (e.g. the log
  redirection failed, which the shell reports only on stderr while still
  forking a child whose PID looks legitimate)."""
  return (f"cd {remote_dir} && mkdir -p {REMOTE_LOG_DIR} && "
          f"{{ nohup env PYTHONPATH={remote_dir} {py} {script_and_flags} "
          f"> {log_path} 2>&1 < /dev/null & pid=$!; sleep 1; "
          f"kill -0 $pid 2>/dev/null && echo $pid || echo DEAD; }}")


def ssh_launch(user: str, host: str, command: str, timeout: float = 30.0) -> str:
  """Start a remote background process via launch_cmd(); return its PID.

  On some devices sshd holds the session open until the nohup'd child
  exits even with every fd redirected, so subprocess.run() blocks for the
  child's whole lifetime (this is what timed out bench takes 2 and 3 — the
  launch itself succeeded). Read the one PID/DEAD line ourselves and tear
  down the local ssh client; the remote process is under nohup and keeps
  running."""
  proc = subprocess.Popen(["ssh", *SSH_OPTS, f"{user}@{host}", command],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
  try:
    ready, _, _ = select.select([proc.stdout], [], [], timeout)
    if not ready:
      raise CheckFailure(f"ssh {host}: no PID within {timeout:.0f}s from `{command}`")
    line = proc.stdout.readline().strip()
  finally:
    proc.kill()
    proc.wait()
  if not line.isdigit():
    err = (proc.stderr.read() or "").strip()
    raise CheckFailure(f"ssh {host}: process died at launch ({err or line!r}): `{command}`")
  return line


def scp_from(user: str, host: str, remote_path: str, local_path: Path) -> bool:
  result = subprocess.run(["scp", *SSH_OPTS, f"{user}@{host}:{remote_path}", str(local_path)],
                          capture_output=True, text=True, timeout=120)
  return result.returncode == 0


_REMOTE_PY_CACHE: dict[str, str] = {}


def remote_python(user: str, host: str, remote_dir: str) -> str:
  """Resolve the python that has openpilot's deps on a device.

  Non-interactive SSH sessions don't run the login profile, so the AGNOS
  venv (and any repo venv) is not on PATH — find it explicitly."""
  if host not in _REMOTE_PY_CACHE:
    probe = (f"for p in {remote_dir}/.venv/bin/python3 /usr/local/venv/bin/python3; do "
             f"[ -x \"$p\" ] && echo \"$p\" && exit 0; done; echo python3")
    out = ssh(user, host, probe).stdout.strip().splitlines()
    _REMOTE_PY_CACHE[host] = out[-1] if out else "python3"
  return _REMOTE_PY_CACHE[host]


def remote_py_prefix(user: str, host: str, remote_dir: str) -> str:
  """`cd <repo> && PYTHONPATH=<repo> <venv-python>` — prefix for remote python commands."""
  py = remote_python(user, host, remote_dir)
  return f"cd {remote_dir} && PYTHONPATH={remote_dir} {py}"


def device_time_us(user: str, host: str) -> tuple[float, float]:
  """Return (device_wall_time_us, pc_mid_wall_time_us) sampled around one SSH round trip."""
  t0 = time.time_ns() / 1000.0
  out = ssh_ok(user, host, "date +%s%N")
  t1 = time.time_ns() / 1000.0
  return int(out) / 1000.0, (t0 + t1) / 2.0


def measure_clock_skew_ms(user: str, ego: str, lead: str) -> float:
  """Lead-minus-ego clock offset in ms (positive: lead clock is ahead)."""
  ego_dev, ego_pc = device_time_us(user, ego)
  lead_dev, lead_pc = device_time_us(user, lead)
  ego_offset = ego_dev - ego_pc
  lead_offset = lead_dev - lead_pc
  return (lead_offset - ego_offset) / 1000.0


# ---------------------------------------------------------------- preflight

def preflight(args) -> int:
  """Run every check; print PASS/WARN/FAIL lines with fixes. Returns count of FAILs."""
  failures = 0
  warnings = 0

  def report(status: str, message: str) -> None:
    nonlocal failures, warnings
    if status == "FAIL":
      failures += 1
    elif status == "WARN":
      warnings += 1
    print(f"  [{status:4}] {message}")

  print(f"Preflight: ego={args.ego_host} lead={args.lead_host} user={args.user}")

  for role, host in (("ego", args.ego_host), ("lead", args.lead_host)):
    try:
      ssh_ok(args.user, host, "true", timeout=10)
      report("PASS", f"{role}: SSH reachable at {host}")
    except (CheckFailure, subprocess.TimeoutExpired) as exc:
      report("FAIL", f"{role}: SSH to {host} failed ({exc}) — is the PC on the ego's hotspot? Try `find-lead` for the lead IP.")
      return failures  # nothing else is checkable

  # Relay service on ego
  result = ssh(args.user, args.ego_host, "systemctl is-active hcc-v2v-relay")
  if result.stdout.strip() == "active":
    report("PASS", "ego: hcc-v2v-relay service active")
  else:
    report("FAIL", "ego: relay not running — sudo tools/hcc_v2v/scripts/setup_v2v_network.sh ego (on the ego)")

  # Log dir writable as the ssh user on both devices. The relay runs as root
  # and creates /data/hcc_v2v_logs root-owned if it gets there first — then
  # every comma-user launch dies instantly on the log redirection (bench take 4).
  for role, host in (("ego", args.ego_host), ("lead", args.lead_host)):
    result = ssh(args.user, host,
                 f"mkdir -p {REMOTE_LOG_DIR} 2>/dev/null; touch {REMOTE_LOG_DIR}/.wtest && rm {REMOTE_LOG_DIR}/.wtest")
    report("PASS" if result.returncode == 0 else "FAIL",
           f"{role}: {REMOTE_LOG_DIR} writable as {args.user}" if result.returncode == 0
           else f"{role}: {REMOTE_LOG_DIR} not writable as {args.user} — "
                f"ssh in and `sudo chown {args.user}:{args.user} {REMOTE_LOG_DIR}`")

  # Params on ego
  param_py = ("from openpilot.common.params import Params; p = Params()\n"
              "ks = ['EnableHCCC', 'AlphaLongitudinalEnabled', 'HCCV2VEnabled', 'HCCV2VOnly']\n"
              "print(','.join(k + '=' + str(int(bool(p.get_bool(k)))) for k in ks))\n"
              "v = p.get('HCCV2VRelayHost')\n"
              "v = v.decode() if isinstance(v, (bytes, bytearray)) else (v or '')\n"
              "print('HCCV2VRelayHost=' + v.strip())\n")
  param_script = f"{remote_py_prefix(args.user, args.ego_host, args.remote_dir)} -c {shlex.quote(param_py)}"
  try:
    out = ssh_ok(args.user, args.ego_host, param_script, timeout=30)
    values = dict(item.split("=", 1) for line in out.splitlines() for item in line.split(",") if "=" in item)
    for key, expected in (("EnableHCCC", "1"), ("AlphaLongitudinalEnabled", "1"),
                          ("HCCV2VEnabled", "1"), ("HCCV2VOnly", "1")):
      if values.get(key) == expected:
        report("PASS", f"ego: param {key}={expected}")
      else:
        report("FAIL", f"ego: param {key}={values.get(key)!r}, expected {expected} — see README one-time setup")
    relay_host = values.get("HCCV2VRelayHost", "")
    if relay_host in ("127.0.0.1", "localhost"):
      report("PASS", f"ego: HCCV2VRelayHost={relay_host} (on-device relay)")
    else:
      report("WARN", f"ego: HCCV2VRelayHost={relay_host!r} — expected 127.0.0.1 for in-car testing")
  except (CheckFailure, subprocess.TimeoutExpired) as exc:
    report("FAIL", f"ego: could not read params ({exc})")

  # Lead: relay host param, scenario CSV, route to ego
  try:
    lead_py = ("from openpilot.common.params import Params\n"
               "v = Params().get('HCCV2VRelayHost')\n"
               "print((v.decode() if isinstance(v, (bytes, bytearray)) else (v or '')).strip())\n")
    out = ssh_ok(args.user, args.lead_host,
                 f"{remote_py_prefix(args.user, args.lead_host, args.remote_dir)} -c {shlex.quote(lead_py)}", timeout=30)
    if out == args.ego_host:
      report("PASS", f"lead: HCCV2VRelayHost={out}")
    else:
      report("WARN", f"lead: HCCV2VRelayHost={out!r} != ego host {args.ego_host} — virtual_lead will use it unless you pass --relay flags")
  except (CheckFailure, subprocess.TimeoutExpired) as exc:
    report("WARN", f"lead: could not read HCCV2VRelayHost ({exc})")

  result = ssh(args.user, args.lead_host, f"test -f {args.remote_dir}/tools/sim/lib/Scenarios.csv")
  report("PASS" if result.returncode == 0 else "FAIL",
         "lead: Scenarios.csv present" if result.returncode == 0 else "lead: Scenarios.csv missing — git pull on the lead device")

  result = ssh(args.user, args.lead_host, f"ping -c 1 -W 2 {args.ego_host}")
  report("PASS" if result.returncode == 0 else "FAIL",
         f"lead: can reach ego at {args.ego_host}" if result.returncode == 0
         else "lead: cannot ping the ego — is the lead on the hotspot? sudo tools/hcc_v2v/scripts/setup_v2v_network.sh lead")

  # Clock skew: the V2V subscriber rejects packets whose sender timestamp is
  # >500 ms from the ego's clock, silently. The lead has no internet on the
  # hotspot, so its clock can drift between test days.
  try:
    skew_ms = measure_clock_skew_ms(args.user, args.ego_host, args.lead_host)
    if abs(skew_ms) >= CLOCK_SKEW_FAIL_MS:
      report("FAIL", f"lead-ego clock skew {skew_ms:+.0f} ms (limit 500) — V2V packets WILL be rejected. "
                     "Give the lead internet briefly (NTP), or set its clock from the ego: "
                     "ssh into the lead and `sudo date -s \"$(ssh comma@10.42.0.1 date)\"`")
    elif abs(skew_ms) >= CLOCK_SKEW_WARN_MS:
      report("WARN", f"lead-ego clock skew {skew_ms:+.0f} ms — works now, but close to the 500 ms V2V limit")
    else:
      report("PASS", f"lead-ego clock skew {skew_ms:+.0f} ms")
  except (CheckFailure, subprocess.TimeoutExpired, ValueError) as exc:
    report("WARN", f"could not measure clock skew ({exc})")

  print(f"Preflight done: {failures} failure(s), {warnings} warning(s)")
  return failures


# ---------------------------------------------------------------- find-lead

def find_lead(args) -> int:
  """List DHCP clients on the ego's hotspot (the lead should be one of them)."""
  try:
    leases = ssh(args.user, args.ego_host,
                 "cat /var/lib/NetworkManager/dnsmasq-wlan0.leases 2>/dev/null; ip neigh show dev wlan0 2>/dev/null")
  except subprocess.TimeoutExpired:
    print(f"could not reach the ego at {args.ego_host}", file=sys.stderr)
    return 1
  print("Devices seen on the ego's hotspot:")
  print(leases.stdout.strip() or "  (none — has the lead joined? Your PC should appear here too.)")
  print("\nThe lead is usually the entry with hostname comma-<serial>. Pass it as --lead_host.")
  return 0


# ---------------------------------------------------------------- run

def _kill_remote(user: str, host: str, pattern: str) -> None:
  with contextlib.suppress(Exception):
    ssh(user, host, f"pkill -f {shlex.quote(pattern)}", timeout=10)


def abort(args) -> int:
  print("killing virtual_lead on the lead and hcc_monitor/bench_ego on the ego...")
  _kill_remote(args.user, args.lead_host, "virtual_lead.py")
  _kill_remote(args.user, args.ego_host, "hcc_monitor.py")
  _kill_remote(args.user, args.ego_host, "bench_ego.py")
  print("done — the ego stops receiving V2V within ~100 ms and stops commanding accel.")
  return 0


def _extract_relay_slice(full_csv: Path, out_csv: Path, t_start_us: int, t_end_us: int) -> int:
  """Copy relay rows within [t_start_us, t_end_us] (ego-clock µs). Returns row count."""
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


def run(args) -> int:
  if not args.skip_checks:
    if preflight(args) > 0:
      print("\npreflight failed — fix the FAILs above, or re-run with --skip_checks if you know better.")
      return 1
    print()

  run_id = time.strftime("run_%Y%m%d_%H%M%S") + f"_scn{args.scn}"
  run_dir = RUNS_DIR / run_id
  run_dir.mkdir(parents=True, exist_ok=True)
  print(f"run id: {run_id}")
  print(f"run dir: {run_dir}")

  remote_sent_csv = f"{REMOTE_LOG_DIR}/sent_{run_id}.csv"
  remote_vl_log = f"{REMOTE_LOG_DIR}/vl_{run_id}.log"
  remote_monitor_csv = f"{REMOTE_LOG_DIR}/monitor_{run_id}.csv"
  remote_monitor_log = f"{REMOTE_LOG_DIR}/monitor_{run_id}.log"
  remote_bench_log = f"{REMOTE_LOG_DIR}/bench_{run_id}.log"

  if args.bench:
    # No car: the real ego subscriber (inside controlsd) is not running, so
    # register a stand-in ego with the relay to exercise the forwarding path.
    ego_py = remote_python(args.user, args.ego_host, args.remote_dir)
    bench_cmd = launch_cmd(args.remote_dir, ego_py, "tools/hcc_v2v/bench_ego.py", remote_bench_log)
    bench_pid = ssh_launch(args.user, args.ego_host, bench_cmd, timeout=30)
    print(f"ego: BENCH ego subscriber started (pid {bench_pid}) — do not use --bench with a real car")

  # Which relay CSV is live right now (newest per-boot file on the ego)?
  relay_remote = ssh(args.user, args.ego_host, f"ls -t {REMOTE_LOG_DIR}/relay_*.csv 2>/dev/null | head -1").stdout.strip()
  if not relay_remote:
    print("WARNING: no relay CSV on the ego — relay logging not active? Continuing without it.")

  # Run start timestamp on the EGO's clock (relay rows are stamped with it).
  t_start_us = int(ssh_ok(args.user, args.ego_host, "date +%s%N")) // 1000

  # Start the ego-side monitor (records engaged/vEgo/accel contributions).
  ego_py = remote_python(args.user, args.ego_host, args.remote_dir)
  monitor_cmd = launch_cmd(args.remote_dir, ego_py,
                           f"tools/hcc_v2v/scripts/hcc_monitor.py --log_csv {remote_monitor_csv} --hz {args.monitor_hz}",
                           remote_monitor_log)
  monitor_pid = ssh_launch(args.user, args.ego_host, monitor_cmd, timeout=30)
  print(f"ego: monitor started (pid {monitor_pid}) -> {remote_monitor_csv}")

  # Start the virtual lead. nohup survives SSH drops; we watch its PID.
  vl_flags = f"--scn {args.scn} --end {args.end} --log_csv {remote_sent_csv}"
  if args.start_delay:
    vl_flags += f" --start_delay {args.start_delay}"
  if args.duration:
    vl_flags += f" --duration {args.duration}"
  if args.loop:
    vl_flags += " --loop"
  if args.speed_scale != 1.0:
    vl_flags += f" --speed_scale {args.speed_scale}"
  if args.max_speed_mph is not None:
    vl_flags += f" --max_speed_mph {args.max_speed_mph}"
  lead_py = remote_python(args.user, args.lead_host, args.remote_dir)
  vl_cmd = launch_cmd(args.remote_dir, lead_py, f"tools/hcc_v2v/virtual_lead.py {vl_flags}", remote_vl_log)
  try:
    vl_pid = ssh_launch(args.user, args.lead_host, vl_cmd, timeout=30)
  except (CheckFailure, subprocess.TimeoutExpired):
    _kill_remote(args.user, args.ego_host, "hcc_monitor.py")
    _kill_remote(args.user, args.ego_host, "bench_ego.py")
    raise
  print(f"lead: virtual_lead started (pid {vl_pid}): scn={args.scn} end={args.end} "
        f"start_delay={args.start_delay}s duration={args.duration or 'profile length'}"
        + (f" max_speed={args.max_speed_mph}mph" if args.max_speed_mph is not None else "")
        + (f" speed_scale={args.speed_scale}" if args.speed_scale != 1.0 else ""))
  print("\n>>> scenario is LIVE — drive/engage now. Ctrl-C ends the run and collects data. <<<\n")

  # Wait for the lead process to exit (duration/end=stop) or Ctrl-C/SIGTERM.
  stop_requested = False

  def _request_stop(signum, frame):
    nonlocal stop_requested
    stop_requested = True

  old_term = signal.signal(signal.SIGTERM, _request_stop)
  try:
    while not stop_requested:
      time.sleep(3.0)
      alive = ssh(args.user, args.lead_host, f"kill -0 {vl_pid} 2>/dev/null").returncode == 0
      tail = ssh(args.user, args.lead_host, f"tail -1 {remote_vl_log}").stdout.strip()
      if tail:
        print(f"  lead: {tail}")
      if not alive:
        print("virtual_lead finished on its own.")
        break
  except KeyboardInterrupt:
    print("\nrun stopped by user.")
  finally:
    signal.signal(signal.SIGTERM, old_term)
    _kill_remote(args.user, args.lead_host, "virtual_lead.py")
    _kill_remote(args.user, args.ego_host, "hcc_monitor.py")
    _kill_remote(args.user, args.ego_host, "bench_ego.py")

  t_end_us = int(ssh_ok(args.user, args.ego_host, "date +%s%N")) // 1000
  print(f"run duration: {(t_end_us - t_start_us) / 1e6:.1f} s — collecting artifacts...")

  collected = _collect_artifacts(args, run_dir, relay_remote, t_start_us, t_end_us,
                                 remote_sent_csv, remote_vl_log, remote_monitor_csv)
  if args.bench:
    collected["bench_ego.log"] = scp_from(args.user, args.ego_host, remote_bench_log, run_dir / "bench_ego.log")

  metadata = {
    "run_id": run_id,
    "scenario": args.scn,
    "ego_host": args.ego_host,
    "lead_host": args.lead_host,
    "start_ego_wall_time_us": t_start_us,
    "end_ego_wall_time_us": t_end_us,
    "duration_s": round((t_end_us - t_start_us) / 1e6, 2),
    "virtual_lead_flags": vl_flags,
    "monitor_hz": args.monitor_hz,
    "relay_csv_on_ego": relay_remote,
    "bench_mode": args.bench,
    "collected": collected,
    "notes": args.notes,
  }
  (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")

  _analyze(run_dir)
  print(f"\nrun complete — everything is in {run_dir}")
  return 0


def _collect_artifacts(args, run_dir: Path, relay_remote: str,
                       t_start_us: int, t_end_us: int,
                       remote_sent_csv: str, remote_vl_log: str, remote_monitor_csv: str) -> dict[str, bool]:
  margin_us = 5_000_000
  collected: dict[str, bool] = {}

  collected["sent.csv"] = scp_from(args.user, args.lead_host, remote_sent_csv, run_dir / "sent.csv")
  collected["virtual_lead.log"] = scp_from(args.user, args.lead_host, remote_vl_log, run_dir / "virtual_lead.log")
  collected["ego_monitor.csv"] = scp_from(args.user, args.ego_host, remote_monitor_csv, run_dir / "ego_monitor.csv")

  if relay_remote:
    full = run_dir / "relay_full.csv"
    if scp_from(args.user, args.ego_host, relay_remote, full):
      rows = _extract_relay_slice(full, run_dir / "relay.csv", t_start_us - margin_us, t_end_us + margin_us)
      full.unlink()
      collected["relay.csv"] = rows > 0
      print(f"  relay.csv: {rows} rows in the run window")
    else:
      collected["relay.csv"] = False

  for name, ok in collected.items():
    if not ok:
      print(f"  WARNING: failed to collect {name}")
  return collected


def _analyze(run_dir: Path) -> None:
  """Stats + plot from the collected relay slice; never fails the run."""
  relay_csv = run_dir / "relay.csv"
  if not relay_csv.is_file():
    return
  from tools.hcc_v2v import plot_v2v_run
  argv = [str(relay_csv), "--out", str(run_dir / "plot.png")]
  if (run_dir / "sent.csv").is_file():
    argv += ["--sent", str(run_dir / "sent.csv")]
  buffer = io.StringIO()
  try:
    with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
      plot_v2v_run.main(argv)
  except (SystemExit, Exception) as exc:  # noqa: BLE001 — analysis is best-effort
    buffer.write(f"\nanalysis incomplete: {exc}\n")
  stats = buffer.getvalue()
  (run_dir / "stats.txt").write_text(stats)
  print(stats.rstrip())


# ---------------------------------------------------------------- collect

def collect(args) -> int:
  """Re-pull device logs for a run that ended uncleanly (PC crash, etc.)."""
  run_dir = RUNS_DIR / args.run_id
  meta_path = run_dir / "metadata.json"
  if meta_path.is_file():
    meta = json.loads(meta_path.read_text())
    t_start_us, t_end_us = meta["start_ego_wall_time_us"], meta["end_ego_wall_time_us"]
  else:
    run_dir.mkdir(parents=True, exist_ok=True)
    t_start_us, t_end_us = 0, 2**63 - 1
    print("no metadata.json — pulling full relay log instead of a time slice")

  relay_remote = ssh(args.user, args.ego_host, f"ls -t {REMOTE_LOG_DIR}/relay_*.csv 2>/dev/null | head -1").stdout.strip()
  collected = _collect_artifacts(args, run_dir, relay_remote, t_start_us, t_end_us,
                                 f"{REMOTE_LOG_DIR}/sent_{args.run_id}.csv",
                                 f"{REMOTE_LOG_DIR}/vl_{args.run_id}.log",
                                 f"{REMOTE_LOG_DIR}/monitor_{args.run_id}.csv")
  _analyze(run_dir)
  print(f"collected into {run_dir}: {collected}")
  return 0


# ---------------------------------------------------------------- main

def _add_common(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--ego_host", default=DEFAULT_EGO_HOST, help=f"Ego device IP (default {DEFAULT_EGO_HOST}, its hotspot address)")
  parser.add_argument("--lead_host", default=None, help="Lead device IP on the hotspot (see find-lead)")
  parser.add_argument("--user", default=DEFAULT_USER)
  parser.add_argument("--remote_dir", default=DEFAULT_REMOTE_DIR, help="openpilot checkout on the devices")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest="command", required=True)

  p_check = sub.add_parser("check", help="Preflight checks")
  _add_common(p_check)

  p_find = sub.add_parser("find-lead", help="Discover the lead's hotspot IP")
  _add_common(p_find)

  p_run = sub.add_parser("run", help="Run one scenario end-to-end")
  _add_common(p_run)
  p_run.add_argument("--scn", type=int, required=True, help="Scenario column (e.g. 48)")
  p_run.add_argument("--duration", type=float, default=None, help="Cap the run length in seconds")
  p_run.add_argument("--start_delay", type=float, default=5.0, help="Seconds at initial speed before the profile moves (default 5)")
  p_run.add_argument("--end", choices=("hold", "stop"), default="stop", help="After the profile (default stop: ego disengages via staleness)")
  p_run.add_argument("--loop", action="store_true", help="Loop the profile until Ctrl-C")
  p_run.add_argument("--speed_scale", type=float, default=1.0,
                     help="Scale the whole speed profile (and its accelerations) by this factor")
  p_run.add_argument("--max_speed_mph", type=float, default=None,
                     help="Scale the profile down so its top speed is at most this many mph (e.g. 20 for a first run)")
  p_run.add_argument("--monitor_hz", type=float, default=10.0, help="Ego response sample rate")
  p_run.add_argument("--notes", default="", help="Free-form run notes saved in metadata.json (conditions, location, ...)")
  p_run.add_argument("--skip_checks", action="store_true", help="Skip the preflight")
  p_run.add_argument("--bench", action="store_true",
                     help="No-car bench test: start a stand-in ego subscriber on the ego device "
                          "so the relay forwards packets even though openpilot is offroad. NEVER use in a car.")

  p_abort = sub.add_parser("abort", help="Kill the remote processes now")
  _add_common(p_abort)

  p_collect = sub.add_parser("collect", help="Re-pull artifacts for a past run id")
  _add_common(p_collect)
  p_collect.add_argument("run_id", help="e.g. run_20260612_153000_scn48")

  args = parser.parse_args(argv)
  if args.command != "find-lead" and args.lead_host is None:
    parser.error("--lead_host is required (use find-lead to discover it)")

  try:
    return {"check": lambda a: 1 if preflight(a) > 0 else 0,
            "find-lead": find_lead,
            "run": run,
            "abort": abort,
            "collect": collect}[args.command](args)
  except CheckFailure as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 1
  except subprocess.TimeoutExpired as exc:
    print(f"error: SSH timed out: {exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
