"""SSH plumbing for driving the two comma devices from the laptop.

Every quirk in here was paid for on a bench or field day; the comments say
which, because they all look like over-engineering until they bite again.
"""
from __future__ import annotations

import select
import subprocess
import time
from pathlib import Path

SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new"]


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


def launch_cmd(remote_dir: str, log_dir: str, py: str, script_and_flags: str, log_path: str) -> str:
  """Remote command that starts a nohup'd background python and prints its PID.

  Prints DEAD instead if the process did not survive its first second -- which
  is what a failed log redirection looks like, since the shell reports that on
  stderr while still forking a child whose PID looks legitimate (bench take 4,
  caused by a root-owned log directory).
  """
  return (f"cd {remote_dir} && mkdir -p {log_dir} && "
          f"{{ nohup env PYTHONPATH={remote_dir} {py} {script_and_flags} "
          f"> {log_path} 2>&1 < /dev/null & pid=$!; sleep 1; "
          f"kill -0 $pid 2>/dev/null && echo $pid || echo DEAD; }}")


def ssh_launch(user: str, host: str, command: str, timeout: float = 30.0) -> str:
  """Start a remote background process via launch_cmd(); return its PID.

  Reads the one PID/DEAD line and then tears down the local ssh client
  ourselves. On some devices sshd holds the session open until the nohup'd
  child exits even with every fd redirected, so subprocess.run() would block
  for the child's whole lifetime -- this is what timed out bench takes 2 and 3,
  where the launch itself had actually succeeded. The remote process is under
  nohup and keeps running.
  """
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


def remote_python(user: str, host: str, remote_dir: str, override: str = "") -> str:
  """Resolve the python to use on a device.

  An explicit `override` from devices.toml wins and costs no round trip: a
  Raspberry Pi lead just has `python3`, and it needs no openpilot deps because
  the deployed publisher is pure stdlib.

  Without one, probe. A comma device needs that, because non-interactive SSH
  does not run the login profile, so neither the AGNOS venv nor a repo venv is
  on PATH. Probing explicitly is what fixed the zmq import failures on the
  first bench attempt.
  """
  if override:
    return override
  if host not in _REMOTE_PY_CACHE:
    probe = (f"for p in {remote_dir}/.venv/bin/python3 /usr/local/venv/bin/python3; do "
             f"[ -x \"$p\" ] && echo \"$p\" && exit 0; done; echo python3")
    out = ssh(user, host, probe).stdout.strip().splitlines()
    _REMOTE_PY_CACHE[host] = out[-1] if out else "python3"
  return _REMOTE_PY_CACHE[host]


def remote_py_prefix(user: str, host: str, remote_dir: str, override: str = "") -> str:
  """`cd <repo> && PYTHONPATH=<repo> <python>` for remote python commands.

  PYTHONPATH matters: running a script by path leaves `openpilot` unimportable.
  """
  py = remote_python(user, host, remote_dir, override)
  return f"cd {remote_dir} && PYTHONPATH={remote_dir} {py}"


def device_time_us(user: str, host: str) -> tuple[float, float]:
  """(device_wall_time_us, pc_mid_wall_time_us) sampled around one SSH round trip."""
  t0 = time.time_ns() / 1000.0
  out = ssh_ok(user, host, "date +%s%N")
  t1 = time.time_ns() / 1000.0
  return int(out) / 1000.0, (t0 + t1) / 2.0


def measure_clock_skew_ms(user: str, ego_host: str, other_host: str) -> float:
  """Other-minus-ego clock offset in ms (positive: the other clock is ahead).

  Matters because the subscriber and relay silently reject packets whose sender
  timestamp is more than 500 ms from the ego's clock. The lead has no internet
  on the test hotspot, so its clock drifts between test days -- and a laptop
  that has been on NTP all morning is the same trap from the other direction.
  """
  ego_dev, ego_pc = device_time_us(user, ego_host)
  other_dev, other_pc = device_time_us(user, other_host)
  return ((other_dev - other_pc) - (ego_dev - ego_pc)) / 1000.0
