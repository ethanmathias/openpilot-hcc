"""UDP endpoints: the subscribers that run inside controlsd and the publishers
that feed the relay.

A subscriber owns a background thread, re-announces itself to the relay once a
second so the relay always knows where to forward, and never blocks the caller:
controlsd only ever reads snapshot(), which is a lock-protected struct copy.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from collections.abc import Callable

from openpilot.selfdrive.controls.lib.hcc.transport.buffer import (
  V2VLeadBuffer,
  V2VPacketBuffer,
  V2VPedalBuffer,
)
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  HELLO_INTERVAL_S,
  RECV_BUFFER_BYTES,
  V2VConfig,
  V2VHelloPacket,
  V2VLeadPacket,
  V2VLeadSignal,
  V2VPedalPacket,
  V2VPedalSignal,
  encode_data_packet,
  encode_hello_packet,
  encode_pedal_packet,
  open_udp_socket,
  parse_data_packet,
  parse_pedal_packet,
  source_address_matches,
  wall_time_us,
)

PARSE_ERRORS = (ValueError, KeyError, TypeError, json.JSONDecodeError)


class _V2VSubscriber:
  """Receive loop shared by the lead and pedal subscribers."""

  def __init__(self, config: V2VConfig, buffer: V2VPacketBuffer, parse: Callable[[bytes], object], thread_name: str):
    self.config = config
    self.buffer = buffer
    self._parse = parse
    self._thread_name = thread_name
    self._stop_event = threading.Event()
    self._thread: threading.Thread | None = None
    self._socket: socket.socket | None = None
    self._last_hello_monotonic = 0.0
    self._transport_ok = True
    self._last_error = ""

  def start(self) -> None:
    if self._thread is not None or not self.config.enabled:
      return
    try:
      self._socket = open_udp_socket()
      self._transport_ok = True
      self._last_error = ""
      self._send_hello(force=True)
    except OSError as exc:
      self._transport_ok = False
      self._last_error = str(exc)
      return
    self._thread = threading.Thread(target=self._run, name=self._thread_name, daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop_event.set()
    if self._thread is not None:
      self._thread.join(timeout=1.0)
    self._thread = None
    if self._socket is not None:
      self._socket.close()
      self._socket = None

  def transport_ok(self) -> bool:
    return self._transport_ok

  def last_error(self) -> str:
    return self._last_error

  def _send_hello(self, force: bool = False) -> None:
    if self._socket is None:
      return
    now_monotonic_s = time.monotonic()
    if not force and (now_monotonic_s - self._last_hello_monotonic) < HELLO_INTERVAL_S:
      return
    self._last_hello_monotonic = now_monotonic_s
    payload = encode_hello_packet(V2VHelloPacket(device_id=self.config.device_id, role=self.config.role))
    self._socket.sendto(payload, (self.config.relay_host, self.config.relay_port))

  def _run(self) -> None:
    assert self._socket is not None
    while not self._stop_event.is_set():
      try:
        self._send_hello()
        payload, source_address = self._socket.recvfrom(RECV_BUFFER_BYTES)
      except TimeoutError:
        continue
      except OSError as exc:
        self._transport_ok = False
        self._last_error = str(exc)
        break

      try:
        packet = self._parse(payload)
      except PARSE_ERRORS:
        continue

      source_address_valid = source_address_matches(self.config, source_address)
      self.buffer.ingest(packet, source_address_valid=source_address_valid)


class V2VLeadSubscriber(_V2VSubscriber):
  def __init__(self, config: V2VConfig):
    super().__init__(config, V2VLeadBuffer(config.stale_threshold_ms, config.max_clock_skew_ms),
                     parse_data_packet, "hcc-v2v-lead")

  def snapshot(self) -> V2VLeadSignal:
    return self.buffer.snapshot()


class V2VPedalSubscriber(_V2VSubscriber):
  def __init__(self, config: V2VConfig):
    super().__init__(config, V2VPedalBuffer(config.stale_threshold_ms, config.max_clock_skew_ms),
                     parse_pedal_packet, "hcc-v2v-pedal")

  def snapshot(self) -> V2VPedalSignal:
    return self.buffer.snapshot()


class _V2VPublisher:
  """Send side shared by the lead and pedal publishers."""

  def __init__(self, config: V2VConfig):
    self.config = config
    self._socket: socket.socket | None = None
    self._seq = 0

  def start(self) -> None:
    if self._socket is not None or not self.config.enabled:
      return
    self._socket = open_udp_socket()
    hello_packet = encode_hello_packet(V2VHelloPacket(device_id=self.config.device_id, role=self.config.role))
    self._socket.sendto(hello_packet, (self.config.relay_host, self.config.relay_port))

  def close(self) -> None:
    if self._socket is not None:
      self._socket.close()
      self._socket = None

  def _ready(self) -> bool:
    if not self.config.enabled:
      return False
    if self._socket is None:
      self.start()
    return self._socket is not None

  def _send(self, payload: bytes) -> None:
    assert self._socket is not None
    self._socket.sendto(payload, (self.config.relay_host, self.config.relay_port))
    self._seq += 1


class V2VPublisher(_V2VPublisher):
  def publish(self, a_lead: float, v_lead: float) -> V2VLeadPacket | None:
    if not self._ready():
      return None
    packet = V2VLeadPacket(
      device_id=self.config.device_id,
      timestamp_us=wall_time_us(),
      a_lead=float(a_lead),
      v_lead=float(v_lead),
      seq=self._seq,
    )
    self._send(encode_data_packet(packet))
    return packet


class V2VPedalPublisher(_V2VPublisher):
  def publish(self, throttle: float, brake: float,
              throttle_raw: float = 0.0, brake_raw: float = 0.0) -> V2VPedalPacket | None:
    if not self._ready():
      return None
    packet = V2VPedalPacket(
      device_id=self.config.device_id,
      timestamp_us=wall_time_us(),
      throttle=float(throttle),
      brake=float(brake),
      seq=self._seq,
      throttle_raw=float(throttle_raw),
      brake_raw=float(brake_raw),
    )
    self._send(encode_pedal_packet(packet))
    return packet
