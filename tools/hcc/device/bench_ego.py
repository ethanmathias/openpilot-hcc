#!/usr/bin/env python3
"""Stand-in ego subscriber for bench tests, with no car.

Registers with the relay as the ego and reports what it receives, so the full
publisher -> relay -> ego path can be exercised with the ego device on a desk.
Offroad, the real subscriber inside controlsd never starts, so without this
the relay has no ego peer and drops everything. `hcc run --bench` starts it.

It now counts BOTH streams, so the pedal rig can be bench-tested the same way
the lead was.

NEVER run this while openpilot is onroad: the relay tracks one ego peer, and
this would steal the registration from the real subscriber -- silently taking
the real controller off the air.
"""
from __future__ import annotations

import argparse
import time

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  RECV_BUFFER_BYTES,
  ROLE_EGO,
  TYPE_DATA,
  TYPE_PEDAL,
  V2VHelloPacket,
  decode_message,
  encode_hello_packet,
  open_udp_socket,
)
from openpilot.tools.hcc.common import load_settings


def main() -> None:
  settings = load_settings()
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--relay_host", default="127.0.0.1",
                      help="The relay runs on this device, so loopback by default")
  parser.add_argument("--relay_port", type=int, default=settings.relay_port)
  parser.add_argument("--device_id", default="bench-ego")
  args = parser.parse_args()

  sock = open_udp_socket()
  hello = encode_hello_packet(V2VHelloPacket(device_id=args.device_id, role=ROLE_EGO))
  relay = (args.relay_host, args.relay_port)

  counts = {TYPE_DATA: 0, TYPE_PEDAL: 0}
  latest: dict[str, dict] = {}
  last_hello = 0.0
  last_print = 0.0

  print(f"[bench_ego] registering as {args.device_id!r} with {relay[0]}:{relay[1]}", flush=True)
  while True:
    now = time.monotonic()
    if now - last_hello >= 1.0:
      sock.sendto(hello, relay)
      last_hello = now

    try:
      payload, _ = sock.recvfrom(RECV_BUFFER_BYTES)
      decoded = decode_message(payload)
      packet_type = str(decoded.get("type", TYPE_DATA))
      if packet_type in counts:
        counts[packet_type] += 1
        latest[packet_type] = decoded
    except TimeoutError:
      pass
    except (ValueError, UnicodeDecodeError):
      continue

    if now - last_print >= 1.0:
      last_print = now
      if not latest:
        print("[bench_ego] registered, waiting for forwarded packets...", flush=True)
        continue
      parts = []
      lead = latest.get(TYPE_DATA)
      if lead is not None:
        parts.append(f"lead: n={counts[TYPE_DATA]} seq={lead.get('seq')} "
                     f"v={float(lead.get('v_lead', 0.0)):.2f} a={float(lead.get('a_lead', 0.0)):+.2f}")
      pedal = latest.get(TYPE_PEDAL)
      if pedal is not None:
        parts.append(f"pedal: n={counts[TYPE_PEDAL]} seq={pedal.get('seq')} "
                     f"thr={float(pedal.get('throttle', 0.0)):.2f} brk={float(pedal.get('brake', 0.0)):.2f}")
      print("[bench_ego] " + "   ".join(parts), flush=True)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
