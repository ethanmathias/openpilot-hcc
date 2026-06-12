#!/usr/bin/env python3
"""Stand-in ego V2V subscriber for bench tests (no car).

Registers with the relay as the ego and prints what it receives, so the full
lead → relay → ego forwarding path can be exercised with the ego device
sitting on a desk (offroad, where the real subscriber inside controlsd never
starts). field_test.py runs this on the ego when given --bench.

NEVER run this while openpilot is onroad — the relay tracks one ego peer,
and this would steal the registration from the real subscriber.
"""
import argparse
import json
import socket
import time


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--relay_host", default="127.0.0.1")
  parser.add_argument("--relay_port", type=int, default=19090)
  parser.add_argument("--device_id", default="bench-ego")
  args = parser.parse_args()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.settimeout(0.2)
  hello = json.dumps({"type": "hello", "device_id": args.device_id, "role": "ego"}).encode()

  received = 0
  last_packet = None
  last_hello = 0.0
  last_print = 0.0
  while True:
    now = time.monotonic()
    if now - last_hello >= 1.0:
      sock.sendto(hello, (args.relay_host, args.relay_port))
      last_hello = now
    try:
      payload, _ = sock.recvfrom(4096)
      decoded = json.loads(payload)
      if decoded.get("type", "data") == "data":
        last_packet = decoded
        received += 1
    except TimeoutError:
      pass
    except (ValueError, UnicodeDecodeError):
      continue
    if now - last_print >= 1.0:
      last_print = now
      if last_packet is not None:
        print(f"[bench_ego] received={received} last: seq={last_packet.get('seq')} "
              f"v={float(last_packet.get('v_lead', 0.0)):.2f} m/s a={float(last_packet.get('a_lead', 0.0)):+.2f} m/s²",
              flush=True)
      else:
        print("[bench_ego] registered, waiting for forwarded packets...", flush=True)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
