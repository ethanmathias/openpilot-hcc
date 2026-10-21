"""Shared send side for the V2V publishers.

One socket for the whole run: the relay pins each peer's (host, port), so
recreating the socket mid-run would change the port and get every subsequent
packet dropped as sender_address_mismatch. The hello is re-sent every
HELLO_INTERVAL_S so a relay restart re-learns this peer automatically.
"""
from __future__ import annotations

import socket
import time

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  HELLO_INTERVAL_S,
  V2VHelloPacket,
  encode_hello_packet,
)


class Publisher:
  role: str = ""

  def __init__(self, relay_host: str, relay_port: int, device_id: str):
    assert self.role, "subclass must set role"
    self.relay = (relay_host, int(relay_port))
    self.device_id = device_id
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.seq = 0
    self._last_hello = 0.0
    self._hello_payload = encode_hello_packet(V2VHelloPacket(device_id=device_id, role=self.role))

  def maybe_hello(self) -> None:
    now = time.monotonic()
    if now - self._last_hello >= HELLO_INTERVAL_S:
      self.sock.sendto(self._hello_payload, self.relay)
      self._last_hello = now

  def _send(self, payload: bytes) -> None:
    self.sock.sendto(payload, self.relay)
    self.seq += 1

  def close(self) -> None:
    self.sock.close()
