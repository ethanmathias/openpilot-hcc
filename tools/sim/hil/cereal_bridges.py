"""Spawn cereal/messaging/bridge subprocesses to shuttle msgq↔ZMQ between
the PC and one Comma 3X device over RNDIS.

For each device the PC runs a pair:
  * msgq→zmq:  ./bridge                         (publishes the PC's
                                                 prefix-namespaced msgq services
                                                 over ZMQ for the device to subscribe)
  * zmq→msgq:  ./bridge <device_ip> <whitelist> (pulls the device's published
                                                 services back into PC msgq)

The corresponding pair is spawned on the device by
tools/sim/hil/scripts/launch_device.sh — see that file for the device side.

OPEN ISSUE (M4 — multi-device): the bridge binary binds ZMQ ports per-service
without a configurable bind interface, so running two msgq→zmq bridges on
the same PC will collide on ports. For two devices this needs either Linux
network namespaces, an msgq_to_zmq patch to honor a bind-IP arg, or two
separate PC-side Python processes confined by netns. M3 (single ego)
sidesteps this; M4 must address it before second-device bring-up.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
from dataclasses import dataclass


# Services the PC needs to read FROM the device. Everything else flows PC→device
# in the catch-all msgq→zmq direction.
SERVICES_FROM_DEVICE = [
  "carControl",
  "carState",
  "controlsState",
  "selfdriveState",
  "carParams",
  "radarState",
]


_STOP_TIMEOUT_SECS = 2.0


@dataclass
class BridgePair:
  prefix: str
  device_ip: str
  procs: list[subprocess.Popen]
  _log_files: list = None  # opened log file handles; closed on stop()

  def stop(self) -> None:
    for p in self.procs:
      try:
        p.terminate()
      except ProcessLookupError:
        pass
    for p in self.procs:
      try:
        p.wait(timeout=_STOP_TIMEOUT_SECS)
      except subprocess.TimeoutExpired:
        p.kill()
    if self._log_files:
      for f in self._log_files:
        f.close()
      self._log_files = None


def _bridge_binary() -> str:
  here = os.path.dirname(os.path.abspath(__file__))
  return os.path.normpath(os.path.join(here, "..", "..", "..", "cereal", "messaging", "bridge"))


def spawn(prefix: str, device_ip: str, log_dir: str = "/tmp") -> BridgePair:
  """Spawn the (msgq→zmq, zmq→msgq) pair for one device."""
  binary = _bridge_binary()
  if not os.path.exists(binary):
    raise FileNotFoundError(f"bridge binary not found at {binary}; build cereal first")

  env = os.environ.copy()
  env["OPENPILOT_PREFIX"] = prefix

  whitelist = " ".join(SERVICES_FROM_DEVICE)
  log_to_dev = open(os.path.join(log_dir, f"hil_bridge_{prefix}_msgq_to_zmq.log"), "ab")
  log_from_dev = open(os.path.join(log_dir, f"hil_bridge_{prefix}_zmq_to_msgq.log"), "ab")

  procs = [
    subprocess.Popen([binary], env=env, stdout=log_to_dev, stderr=subprocess.STDOUT),
    subprocess.Popen([binary, device_ip, whitelist], env=env, stdout=log_from_dev, stderr=subprocess.STDOUT),
  ]
  return BridgePair(prefix=prefix, device_ip=device_ip, procs=procs,
                    _log_files=[log_to_dev, log_from_dev])


def main() -> int:
  import argparse
  parser = argparse.ArgumentParser(description="Spawn cereal-bridge subprocesses for one device.")
  parser.add_argument("--prefix", required=True, help="OPENPILOT_PREFIX (e.g., hccego, hcclead)")
  parser.add_argument("--device", required=True, help="Device RNDIS IP")
  args = parser.parse_args()

  pair = spawn(args.prefix, args.device)
  print(f"[cereal_bridges] spawned {len(pair.procs)} processes for prefix={args.prefix} device={args.device}")

  def _shutdown(*_):
    pair.stop()
    sys.exit(0)
  signal.signal(signal.SIGINT, _shutdown)
  signal.signal(signal.SIGTERM, _shutdown)

  for p in pair.procs:
    p.wait()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
