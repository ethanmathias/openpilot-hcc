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

MULTI-DEVICE PORT COLLISION: the stock bridge binary binds tcp://*:<port>
per service, so two msgq→zmq bridges on the same PC collide and the second
one exits at startup. Fix: apply tools/sim/hil/patches/zmq_bind_address.patch
to the msgq submodule on the PC and rebuild; the patched ZMQPubSocket honors
ZMQ_BIND_ADDRESS, which spawn() sets to the PC-side RNDIS IP of each device
(devices.toml `pc_ip`). Each device then connects to its own PC_IP and only
sees its own bridge. Single-device (ego-only) runs work unpatched.
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
  _log_files: list | None = None  # opened log file handles; closed on stop()

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


def spawn(prefix: str, device_ip: str, bind_ip: str | None = None, log_dir: str = "/tmp") -> BridgePair:
  """Spawn the (msgq→zmq, zmq→msgq) pair for one device.

  bind_ip is the PC-side RNDIS IP for this device. With the patched msgq it
  scopes the msgq→zmq bridge's ZMQ binds to that interface so a second
  device's bridge can coexist (see module docstring). Ignored by stock msgq.
  """
  binary = _bridge_binary()
  if not os.path.exists(binary):
    raise FileNotFoundError(f"bridge binary not found at {binary}; build cereal first")

  env = os.environ.copy()
  env["OPENPILOT_PREFIX"] = prefix
  if bind_ip is not None:
    env["ZMQ_BIND_ADDRESS"] = bind_ip

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
  parser.add_argument("--bind_ip", default=None, help="PC-side RNDIS IP to scope the ZMQ binds to (needs patched msgq)")
  args = parser.parse_args()

  pair = spawn(args.prefix, args.device, bind_ip=args.bind_ip)
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
