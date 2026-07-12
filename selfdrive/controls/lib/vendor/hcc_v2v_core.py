from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

DEFAULT_RELAY_HOST = "127.0.0.1"
DEFAULT_RELAY_PORT = 19090
DEFAULT_SEND_HZ = 50.0
# Field-calibrated (2026-07-11): the subscriber thread inside controlsd gets
# starved ~100 ms about once a second by OS scheduling, so a 100 ms threshold
# flapped once per second on a perfectly healthy 50 Hz stream. 300 ms clears
# the observed jitter with margin while still zeroing the command within a
# third of a second of a true dropout. Overridable via HCC_V2V_STALE_THRESHOLD_MS.
DEFAULT_STALE_THRESHOLD_MS = 300.0
DEFAULT_SENDER_ADDRESS_POLICY = "relay_host"
MAX_TIMESTAMP_SKEW_MS = 500.0
HELLO_INTERVAL_S = 1.0
RECV_BUFFER_BYTES = 4096
# A seq that regresses by more than this is a publisher restart (every
# publisher session numbers from 0), not a replayed packet: adopt the new
# session. Regressions within the window are dropped as replays/reorders.
SESSION_RESET_SEQ_GAP = 50

ROLE_LEAD = "lead"
ROLE_EGO = "ego"
TYPE_DATA = "data"
TYPE_HELLO = "hello"


@dataclass(frozen=True)
class V2VConfig:
  role: str
  enabled: bool
  relay_host: str
  relay_port: int
  device_id: str
  send_hz: float = DEFAULT_SEND_HZ
  stale_threshold_ms: float = DEFAULT_STALE_THRESHOLD_MS
  max_clock_skew_ms: float = MAX_TIMESTAMP_SKEW_MS
  sender_address_policy: str = DEFAULT_SENDER_ADDRESS_POLICY
  expected_sender_host: str | None = None
  expected_sender_port: int | None = None


@dataclass(frozen=True)
class V2VLeadPacket:
  device_id: str
  timestamp_us: int
  a_lead: float
  v_lead: float
  seq: int


@dataclass(frozen=True)
class V2VHelloPacket:
  device_id: str
  role: str


@dataclass(frozen=True)
class V2VLeadSignal:
  status: bool
  lead_speed_mps: float = 0.0
  lead_accel_mps2: float = 0.0
  seq: int = -1
  source_device_id: str = ""
  local_receive_valid: bool = False
  sender_timestamp_valid: bool = False
  source_address_valid: bool = False
  receive_age_ms: float = float("inf")


@dataclass(frozen=True)
class ReceivedLeadPacket:
  packet: V2VLeadPacket
  local_recv_monotonic_ns: int
  local_recv_wall_time_us: int
  sender_timestamp_valid: bool
  source_address_valid: bool


def monotonic_time_ns() -> int:
  return time.monotonic_ns()


def wall_time_us() -> int:
  return time.time_ns() // 1000


def _as_float(raw: Any, field_name: str) -> float:
  value = float(raw)
  if not value == value or value in (float("inf"), float("-inf")):
    raise ValueError(f"{field_name} must be finite")
  return value


def _compact_json(payload: dict[str, Any]) -> bytes:
  return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def encode_data_packet(packet: V2VLeadPacket) -> bytes:
  return _compact_json({
    "type": TYPE_DATA,
    "device_id": packet.device_id,
    "timestamp_us": int(packet.timestamp_us),
    "a_lead": float(packet.a_lead),
    "v_lead": float(packet.v_lead),
    "seq": int(packet.seq),
  })


def encode_hello_packet(packet: V2VHelloPacket) -> bytes:
  return _compact_json({
    "type": TYPE_HELLO,
    "device_id": packet.device_id,
    "role": packet.role,
  })


def decode_message(raw_payload: bytes) -> dict[str, Any]:
  decoded = json.loads(raw_payload.decode("utf-8"))
  if not isinstance(decoded, dict):
    raise ValueError("payload must decode to an object")
  return decoded


def parse_hello_packet(raw_payload: bytes) -> V2VHelloPacket:
  decoded = decode_message(raw_payload)
  if decoded.get("type") != TYPE_HELLO:
    raise ValueError("not a hello packet")
  role = str(decoded["role"])
  if role not in (ROLE_LEAD, ROLE_EGO):
    raise ValueError("invalid role")
  device_id = str(decoded["device_id"]).strip()
  if len(device_id) == 0:
    raise ValueError("device_id cannot be empty")
  return V2VHelloPacket(device_id=device_id, role=role)


def parse_data_packet(raw_payload: bytes) -> V2VLeadPacket:
  decoded = decode_message(raw_payload)
  if decoded.get("type", TYPE_DATA) != TYPE_DATA:
    raise ValueError("not a data packet")

  device_id = str(decoded["device_id"]).strip()
  if len(device_id) == 0:
    raise ValueError("device_id cannot be empty")

  packet = V2VLeadPacket(
    device_id=device_id,
    timestamp_us=int(decoded["timestamp_us"]),
    a_lead=_as_float(decoded["a_lead"], "a_lead"),
    v_lead=_as_float(decoded["v_lead"], "v_lead"),
    seq=int(decoded["seq"]),
  )
  if packet.seq < 0:
    raise ValueError("seq must be >= 0")
  if packet.timestamp_us <= 0:
    raise ValueError("timestamp_us must be > 0")
  return packet


def is_timestamp_plausible(timestamp_us: int,
                           now_wall_time_us: int | None = None,
                           max_clock_skew_ms: float = MAX_TIMESTAMP_SKEW_MS) -> bool:
  current_wall_time_us = wall_time_us() if now_wall_time_us is None else int(now_wall_time_us)
  skew_us = abs(current_wall_time_us - int(timestamp_us))
  return skew_us <= int(max_clock_skew_ms * 1000.0)


def packet_receive_age_ms(local_recv_monotonic_ns: int,
                          now_monotonic_ns: int | None = None) -> float:
  current_monotonic_ns = monotonic_time_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
  return max(0.0, (current_monotonic_ns - int(local_recv_monotonic_ns)) / 1e6)


def _resolve_host_candidates(host: str) -> set[str]:
  candidates = {host}
  try:
    for info in socket.getaddrinfo(host, None, proto=socket.IPPROTO_UDP):
      sockaddr = info[4]
      if len(sockaddr) >= 1:
        candidates.add(str(sockaddr[0]))
  except OSError:
    pass
  return candidates


def source_address_matches(config: V2VConfig, source_address: tuple[str, int]) -> bool:
  if config.sender_address_policy == "any":
    return True
  expected_host = config.expected_sender_host or config.relay_host
  expected_port = int(config.expected_sender_port or config.relay_port)
  actual_host, actual_port = str(source_address[0]), int(source_address[1])
  if actual_port != expected_port:
    return False
  return actual_host in _resolve_host_candidates(expected_host)


def open_udp_socket(bind_host: str = "0.0.0.0", bind_port: int = 0, timeout_s: float = 0.2) -> socket.socket:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.settimeout(timeout_s)
  sock.bind((bind_host, bind_port))
  return sock


class V2VLeadBuffer:
  def __init__(self, stale_threshold_ms: float = DEFAULT_STALE_THRESHOLD_MS, max_clock_skew_ms: float = MAX_TIMESTAMP_SKEW_MS):
    self.stale_threshold_ms = stale_threshold_ms
    self.max_clock_skew_ms = max_clock_skew_ms
    self._latest: ReceivedLeadPacket | None = None
    self._last_seq = -1
    self._lock = threading.Lock()

  def ingest(self,
             packet: V2VLeadPacket,
             recv_monotonic_ns: int | None = None,
             recv_wall_time_us: int | None = None,
             source_address_valid: bool = True) -> bool:
    received_monotonic_ns = monotonic_time_ns() if recv_monotonic_ns is None else int(recv_monotonic_ns)
    received_wall_time_us = wall_time_us() if recv_wall_time_us is None else int(recv_wall_time_us)
    sender_timestamp_valid = is_timestamp_plausible(packet.timestamp_us, received_wall_time_us, self.max_clock_skew_ms)
    if not sender_timestamp_valid or not source_address_valid:
      return False

    with self._lock:
      if packet.seq <= self._last_seq and (self._last_seq - packet.seq) <= SESSION_RESET_SEQ_GAP:
        # Duplicate or nearby replay within the current publisher session.
        return False
      # Forward progress, or a large backward jump = the publisher restarted
      # (a new session always numbers from 0) — adopt the new session.
      self._last_seq = packet.seq
      self._latest = ReceivedLeadPacket(
        packet=packet,
        local_recv_monotonic_ns=received_monotonic_ns,
        local_recv_wall_time_us=received_wall_time_us,
        sender_timestamp_valid=True,
        source_address_valid=True,
      )
    return True

  def snapshot(self, now_monotonic_ns: int | None = None) -> V2VLeadSignal:
    current_monotonic_ns = monotonic_time_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
    with self._lock:
      latest = self._latest

    if latest is None:
      return V2VLeadSignal(status=False)

    receive_age_ms = packet_receive_age_ms(latest.local_recv_monotonic_ns, current_monotonic_ns)
    local_receive_valid = receive_age_ms <= self.stale_threshold_ms

    return V2VLeadSignal(
      status=bool(local_receive_valid and latest.sender_timestamp_valid and latest.source_address_valid),
      lead_speed_mps=float(latest.packet.v_lead),
      lead_accel_mps2=float(latest.packet.a_lead),
      seq=int(latest.packet.seq),
      source_device_id=latest.packet.device_id,
      local_receive_valid=local_receive_valid,
      sender_timestamp_valid=latest.sender_timestamp_valid,
      source_address_valid=latest.source_address_valid,
      receive_age_ms=receive_age_ms,
    )


class V2VLeadSubscriber:
  def __init__(self, config: V2VConfig):
    self.config = config
    self.buffer = V2VLeadBuffer(config.stale_threshold_ms, config.max_clock_skew_ms)
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
    self._thread = threading.Thread(target=self._run, name="hcc-v2v-subscriber", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop_event.set()
    if self._thread is not None:
      self._thread.join(timeout=1.0)
    self._thread = None
    if self._socket is not None:
      self._socket.close()
      self._socket = None

  def snapshot(self) -> V2VLeadSignal:
    return self.buffer.snapshot()

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
        packet = parse_data_packet(payload)
      except (ValueError, KeyError, TypeError, json.JSONDecodeError):
        continue

      source_address_valid = source_address_matches(self.config, source_address)
      self.buffer.ingest(packet, source_address_valid=source_address_valid)


class V2VPublisher:
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

  def publish(self, a_lead: float, v_lead: float) -> V2VLeadPacket | None:
    if not self.config.enabled:
      return None
    if self._socket is None:
      self.start()
    if self._socket is None:
      return None

    packet = V2VLeadPacket(
      device_id=self.config.device_id,
      timestamp_us=wall_time_us(),
      a_lead=float(a_lead),
      v_lead=float(v_lead),
      seq=self._seq,
    )
    self._seq += 1
    self._socket.sendto(encode_data_packet(packet), (self.config.relay_host, self.config.relay_port))
    return packet

