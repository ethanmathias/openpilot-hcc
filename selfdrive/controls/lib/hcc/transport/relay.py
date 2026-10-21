"""The UDP relay that sits on the ego device and forwards between peers.

Peers announce themselves with a hello carrying their role, and the relay pins
each role to the address the hello arrived from. Three roles exist:

  lead   -> forwarded to ego
  pedal  -> forwarded to ego
  ego    -> the forwarding destination; sends only hellos

The lead path is the component with the measured 0% loss over 1750 packets, so
pedal support is added as a branch taken before any lead validation runs rather
than by generalizing that path. A pedal packet and a lead packet never touch
each other's code.
"""
from __future__ import annotations

import argparse
import csv
import socket
from dataclasses import dataclass
from pathlib import Path

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  RECV_BUFFER_BYTES,
  ROLE_EGO,
  ROLE_LEAD,
  ROLE_PEDAL,
  TYPE_DATA,
  TYPE_HELLO,
  TYPE_PEDAL,
  V2VLeadPacket,
  V2VPedalPacket,
  decode_message,
  is_timestamp_plausible,
  parse_data_packet,
  parse_hello_packet,
  parse_pedal_packet,
  wall_time_us,
)

RELAY_CSV_COLUMNS = [
  "recv_wall_time_us",
  "event_type",
  "role",
  "device_id",
  "source_host",
  "source_port",
  "dest_host",
  "dest_port",
  "seq",
  "a_lead",
  "v_lead",
  "throttle",
  "brake",
  "forwarded",
  "reason",
]


@dataclass(frozen=True)
class RegisteredPeer:
  device_id: str
  role: str
  address: tuple[str, int]


@dataclass(frozen=True)
class RelayAction:
  """A lead packet cleared for forwarding."""
  payload: bytes
  destination: tuple[str, int]
  device_id: str
  seq: int
  a_lead: float
  v_lead: float


@dataclass(frozen=True)
class PedalRelayAction:
  """A pedal packet cleared for forwarding."""
  payload: bytes
  destination: tuple[str, int]
  device_id: str
  seq: int
  throttle: float
  brake: float


class _SeqValidator:
  """Rejects replayed sequence numbers and implausible timestamps per device."""

  def __init__(self, max_clock_skew_ms: float = 500.0):
    self.max_clock_skew_ms = max_clock_skew_ms
    self._last_seq_by_device: dict[str, int] = {}

  def validate(self, packet: V2VLeadPacket | V2VPedalPacket) -> str | None:
    if not is_timestamp_plausible(packet.timestamp_us, max_clock_skew_ms=self.max_clock_skew_ms):
      return "timestamp_invalid"
    previous_seq = self._last_seq_by_device.get(packet.device_id)
    if previous_seq is not None and packet.seq <= previous_seq:
      return "duplicate_or_replayed_seq"
    self._last_seq_by_device[packet.device_id] = packet.seq
    return None


class RelayRouter:
  def __init__(self, enforce_stream_validation: bool = False, max_clock_skew_ms: float = 500.0):
    self._peers: dict[str, RegisteredPeer] = {}
    self.last_reason = "uninitialized"
    self._validator = _SeqValidator(max_clock_skew_ms) if enforce_stream_validation else None

  @property
  def lead_peer(self) -> RegisteredPeer | None:
    return self._peers.get(ROLE_LEAD)

  @property
  def ego_peer(self) -> RegisteredPeer | None:
    return self._peers.get(ROLE_EGO)

  @property
  def pedal_peer(self) -> RegisteredPeer | None:
    return self._peers.get(ROLE_PEDAL)

  def handle_datagram(self, payload: bytes, source_address: tuple[str, int]):
    """Route one datagram. Returns (event_type, role, device_id, action|None)."""
    decoded = decode_message(payload)
    packet_type = str(decoded.get("type", TYPE_DATA))

    if packet_type == TYPE_HELLO:
      hello = parse_hello_packet(payload)
      self._peers[hello.role] = RegisteredPeer(hello.device_id, hello.role, (source_address[0], int(source_address[1])))
      self.last_reason = "registered_peer"
      return (TYPE_HELLO, hello.role, hello.device_id, None)

    # Pedal branch, taken before the lead path is consulted at all.
    if packet_type == TYPE_PEDAL:
      return self._handle_pedal(payload, source_address)

    return self._handle_lead(payload, source_address)

  def _reject(self, event_type: str, role: str, device_id: str, reason: str):
    self.last_reason = reason
    return (event_type, role, device_id, None)

  def _check_peer(self, peer: RegisteredPeer | None, packet, source_address: tuple[str, int], missing: str) -> str | None:
    """Shared peer checks: registered, device id pinned, address pinned."""
    if peer is None:
      return missing
    if packet.device_id != peer.device_id:
      return "device_id_mismatch"
    if (source_address[0], int(source_address[1])) != peer.address:
      return "sender_address_mismatch"
    return None

  def _handle_pedal(self, payload: bytes, source_address: tuple[str, int]):
    packet = parse_pedal_packet(payload)
    ego_peer = self.ego_peer
    if ego_peer is None:
      return self._reject(TYPE_PEDAL, ROLE_PEDAL, packet.device_id, "missing_ego_peer")

    failure = self._check_peer(self.pedal_peer, packet, source_address, "missing_pedal_peer")
    if failure is not None:
      return self._reject(TYPE_PEDAL, ROLE_PEDAL, packet.device_id, failure)

    if self._validator is not None:
      failure = self._validator.validate(packet)
      if failure is not None:
        return self._reject(TYPE_PEDAL, ROLE_PEDAL, packet.device_id, failure)

    action = PedalRelayAction(
      payload=payload,
      destination=ego_peer.address,
      device_id=packet.device_id,
      seq=packet.seq,
      throttle=packet.throttle,
      brake=packet.brake,
    )
    self.last_reason = "forwarded"
    return (TYPE_PEDAL, ROLE_PEDAL, packet.device_id, action)

  def _handle_lead(self, payload: bytes, source_address: tuple[str, int]):
    packet = parse_data_packet(payload)
    ego_peer = self.ego_peer
    if self.lead_peer is None:
      return self._reject(TYPE_DATA, ROLE_LEAD, packet.device_id, "missing_lead_peer")
    if ego_peer is None:
      return self._reject(TYPE_DATA, ROLE_LEAD, packet.device_id, "missing_ego_peer")

    failure = self._check_peer(self.lead_peer, packet, source_address, "missing_lead_peer")
    if failure is not None:
      return self._reject(TYPE_DATA, ROLE_LEAD, packet.device_id, failure)

    if self._validator is not None:
      failure = self._validator.validate(packet)
      if failure is not None:
        return self._reject(TYPE_DATA, ROLE_LEAD, packet.device_id, failure)

    action = RelayAction(
      payload=payload,
      destination=ego_peer.address,
      device_id=packet.device_id,
      seq=packet.seq,
      a_lead=packet.a_lead,
      v_lead=packet.v_lead,
    )
    self.last_reason = "forwarded"
    return (TYPE_DATA, ROLE_LEAD, packet.device_id, action)


class UDPRelayServer:
  def __init__(self, host: str, port: int, log_csv_path: str | None = None, enforce_stream_validation: bool = False):
    self.host = host
    self.port = port
    self.router = RelayRouter(enforce_stream_validation=enforce_stream_validation)
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.socket.bind((host, port))
    self.socket.settimeout(0.2)
    self._log_file = None
    self._csv_writer = None

    if log_csv_path is not None:
      log_path = Path(log_csv_path).expanduser()
      log_path.parent.mkdir(parents=True, exist_ok=True)
      self._log_file = log_path.open("w", newline="")
      self._csv_writer = csv.writer(self._log_file)
      self._csv_writer.writerow(RELAY_CSV_COLUMNS)
      self._log_file.flush()

  def close(self) -> None:
    self.socket.close()
    if self._log_file is not None:
      self._log_file.close()
      self._log_file = None

  def _log_event(self, event_type: str, role: str, device_id: str,
                 source_address: tuple[str, int], action) -> None:
    if self._csv_writer is None:
      return
    is_pedal = isinstance(action, PedalRelayAction)
    is_lead = isinstance(action, RelayAction)
    self._csv_writer.writerow([
      wall_time_us(),
      event_type,
      role,
      device_id,
      source_address[0],
      int(source_address[1]),
      "" if action is None else action.destination[0],
      "" if action is None else int(action.destination[1]),
      "" if action is None else int(action.seq),
      float(action.a_lead) if is_lead else "",
      float(action.v_lead) if is_lead else "",
      float(action.throttle) if is_pedal else "",
      float(action.brake) if is_pedal else "",
      bool(action is not None),
      self.router.last_reason,
    ])
    self._log_file.flush()

  def serve_forever(self) -> None:
    while True:
      try:
        payload, address = self.socket.recvfrom(RECV_BUFFER_BYTES)
      except TimeoutError:
        continue
      try:
        event_type, role, device_id, action = self.router.handle_datagram(payload, address)
      except (ValueError, KeyError, TypeError) as exc:
        # A malformed datagram must never take the relay down mid-run.
        self.router.last_reason = f"malformed:{type(exc).__name__}"
        self._log_event("malformed", "?", "", address, None)
        continue
      if action is not None:
        self.socket.sendto(action.payload, action.destination)
      self._log_event(event_type, role, device_id, address, action)


def _build_arg_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="HCC UDP relay server.")
  parser.add_argument("--host", default="0.0.0.0", help="Host interface to bind the relay socket to.")
  parser.add_argument("--port", type=int, default=19090, help="UDP port to bind.")
  parser.add_argument("--log_csv", default=None, help="Optional CSV log output path.")
  parser.add_argument("--enforce_stream_validation", action="store_true",
                      help="Reject replayed sequence numbers and implausible packet timestamps.")
  return parser


def main() -> None:
  args = _build_arg_parser().parse_args()
  server = UDPRelayServer(args.host, args.port, args.log_csv, args.enforce_stream_validation)
  try:
    server.serve_forever()
  finally:
    server.close()


if __name__ == "__main__":
  main()
