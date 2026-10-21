"""Wire format and shared helpers for the HCC V2V link.

Two kinds of payload travel over the same UDP relay:

  lead   -- speed/acceleration of the vehicle ahead, from the lead device or
            tools/hcc/publishers/virtual_lead.py
  pedal  -- normalized throttle/brake from the pedal rig, published by a
            laptop running tools/hcc/publishers/pedal.py

Both are JSON datagrams carrying a device id, a wall-clock timestamp and a
monotonically increasing sequence number, so the staleness, clock-skew and
session-reset logic in buffer.py applies to either without knowing its shape.
"""
from __future__ import annotations

import json
import socket
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

# The pedal stream gets a tighter threshold than the lead: 300 ms of stale lead
# data is small against the 1.5 s headway the feedforward is tuned around, but
# 300 ms of stale pedal is a long time to hold a human's last brake command.
DEFAULT_PEDAL_STALE_THRESHOLD_MS = 150.0

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
ROLE_PEDAL = "pedal"
ROLES = (ROLE_LEAD, ROLE_EGO, ROLE_PEDAL)

TYPE_DATA = "data"
TYPE_HELLO = "hello"
TYPE_PEDAL = "pedal"


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
class V2VHelloPacket:
  device_id: str
  role: str


@dataclass(frozen=True)
class V2VLeadPacket:
  device_id: str
  timestamp_us: int
  a_lead: float
  v_lead: float
  seq: int


@dataclass(frozen=True)
class V2VPedalPacket:
  """Normalized pedal input. throttle/brake are fractions in [0, 1].

  The raw axis counts travel alongside the normalized values so a calibration
  mistake can be corrected in post rather than invalidating a run.
  """
  device_id: str
  timestamp_us: int
  throttle: float
  brake: float
  seq: int
  throttle_raw: float = 0.0
  brake_raw: float = 0.0


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
class V2VPedalSignal:
  status: bool
  throttle: float = 0.0
  brake: float = 0.0
  throttle_raw: float = 0.0
  brake_raw: float = 0.0
  seq: int = -1
  source_device_id: str = ""
  local_receive_valid: bool = False
  sender_timestamp_valid: bool = False
  source_address_valid: bool = False
  receive_age_ms: float = float("inf")


@dataclass(frozen=True)
class ReceivedPacket:
  """A packet of either shape, plus how and when it arrived locally."""
  packet: Any
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


def _as_fraction(raw: Any, field_name: str) -> float:
  value = _as_float(raw, field_name)
  if value < 0.0 or value > 1.0:
    raise ValueError(f"{field_name} must be in [0, 1]")
  return value


def _compact_json(payload: dict[str, Any]) -> bytes:
  return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _require_device_id(decoded: dict[str, Any]) -> str:
  device_id = str(decoded["device_id"]).strip()
  if len(device_id) == 0:
    raise ValueError("device_id cannot be empty")
  return device_id


def _require_seq_and_timestamp(seq: int, timestamp_us: int) -> None:
  if seq < 0:
    raise ValueError("seq must be >= 0")
  if timestamp_us <= 0:
    raise ValueError("timestamp_us must be > 0")


def encode_hello_packet(packet: V2VHelloPacket) -> bytes:
  return _compact_json({
    "type": TYPE_HELLO,
    "device_id": packet.device_id,
    "role": packet.role,
  })


def encode_data_packet(packet: V2VLeadPacket) -> bytes:
  return _compact_json({
    "type": TYPE_DATA,
    "device_id": packet.device_id,
    "timestamp_us": int(packet.timestamp_us),
    "a_lead": float(packet.a_lead),
    "v_lead": float(packet.v_lead),
    "seq": int(packet.seq),
  })


def encode_pedal_packet(packet: V2VPedalPacket) -> bytes:
  return _compact_json({
    "type": TYPE_PEDAL,
    "device_id": packet.device_id,
    "timestamp_us": int(packet.timestamp_us),
    "throttle": float(packet.throttle),
    "brake": float(packet.brake),
    "throttle_raw": float(packet.throttle_raw),
    "brake_raw": float(packet.brake_raw),
    "seq": int(packet.seq),
  })


def decode_message(raw_payload: bytes) -> dict[str, Any]:
  decoded = json.loads(raw_payload.decode("utf-8"))
  if not isinstance(decoded, dict):
    raise ValueError("payload must decode to an object")
  return decoded


def message_type(raw_payload: bytes) -> str:
  """The packet's type, defaulting to TYPE_DATA for payloads that omit it."""
  return str(decode_message(raw_payload).get("type", TYPE_DATA))


def parse_hello_packet(raw_payload: bytes) -> V2VHelloPacket:
  decoded = decode_message(raw_payload)
  if decoded.get("type") != TYPE_HELLO:
    raise ValueError("not a hello packet")
  role = str(decoded["role"])
  if role not in ROLES:
    raise ValueError("invalid role")
  return V2VHelloPacket(device_id=_require_device_id(decoded), role=role)


def parse_data_packet(raw_payload: bytes) -> V2VLeadPacket:
  decoded = decode_message(raw_payload)
  if decoded.get("type", TYPE_DATA) != TYPE_DATA:
    raise ValueError("not a data packet")

  packet = V2VLeadPacket(
    device_id=_require_device_id(decoded),
    timestamp_us=int(decoded["timestamp_us"]),
    a_lead=_as_float(decoded["a_lead"], "a_lead"),
    v_lead=_as_float(decoded["v_lead"], "v_lead"),
    seq=int(decoded["seq"]),
  )
  _require_seq_and_timestamp(packet.seq, packet.timestamp_us)
  return packet


def parse_pedal_packet(raw_payload: bytes) -> V2VPedalPacket:
  decoded = decode_message(raw_payload)
  if decoded.get("type") != TYPE_PEDAL:
    raise ValueError("not a pedal packet")

  packet = V2VPedalPacket(
    device_id=_require_device_id(decoded),
    timestamp_us=int(decoded["timestamp_us"]),
    throttle=_as_fraction(decoded["throttle"], "throttle"),
    brake=_as_fraction(decoded["brake"], "brake"),
    seq=int(decoded["seq"]),
    throttle_raw=_as_float(decoded.get("throttle_raw", 0.0), "throttle_raw"),
    brake_raw=_as_float(decoded.get("brake_raw", 0.0), "brake_raw"),
  )
  _require_seq_and_timestamp(packet.seq, packet.timestamp_us)
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
