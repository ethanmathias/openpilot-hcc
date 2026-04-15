from __future__ import annotations

import argparse
import csv
import socket
from dataclasses import dataclass
from pathlib import Path

from openpilot.selfdrive.controls.lib.vendor.hcc_v2v_core import (
  RECV_BUFFER_BYTES,
  ROLE_EGO,
  ROLE_LEAD,
  TYPE_DATA,
  TYPE_HELLO,
  V2VLeadPacket,
  decode_message,
  is_timestamp_plausible,
  parse_data_packet,
  parse_hello_packet,
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
  payload: bytes
  destination: tuple[str, int]
  device_id: str
  seq: int
  a_lead: float
  v_lead: float


class _LeadPacketValidator:
  def __init__(self, max_clock_skew_ms: float = 500.0):
    self.max_clock_skew_ms = max_clock_skew_ms
    self._last_seq_by_device: dict[str, int] = {}

  def validate(self, packet: V2VLeadPacket) -> str | None:
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
    self._validator = _LeadPacketValidator(max_clock_skew_ms) if enforce_stream_validation else None

  @property
  def lead_peer(self) -> RegisteredPeer | None:
    return self._peers.get(ROLE_LEAD)

  @property
  def ego_peer(self) -> RegisteredPeer | None:
    return self._peers.get(ROLE_EGO)

  def handle_datagram(self, payload: bytes, source_address: tuple[str, int]) -> tuple[str, str, str, RelayAction | None]:
    decoded = decode_message(payload)
    packet_type = str(decoded.get("type", TYPE_DATA))
    if packet_type == TYPE_HELLO:
      hello = parse_hello_packet(payload)
      self._peers[hello.role] = RegisteredPeer(hello.device_id, hello.role, (source_address[0], int(source_address[1])))
      self.last_reason = "registered_peer"
      return (TYPE_HELLO, hello.role, hello.device_id, None)

    packet = parse_data_packet(payload)
    lead_peer = self.lead_peer
    ego_peer = self.ego_peer
    if lead_peer is None:
      self.last_reason = "missing_lead_peer"
      return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)
    if ego_peer is None:
      self.last_reason = "missing_ego_peer"
      return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)
    if packet.device_id != lead_peer.device_id:
      self.last_reason = "device_id_mismatch"
      return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)
    if (source_address[0], int(source_address[1])) != lead_peer.address:
      self.last_reason = "sender_address_mismatch"
      return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)
    if self._validator is not None:
      failure = self._validator.validate(packet)
      if failure is not None:
        self.last_reason = failure
        return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)

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

  def close(self) -> None:
    self.socket.close()
    if self._log_file is not None:
      self._log_file.close()
      self._log_file = None

  def _log_event(self, event_type: str, role: str, device_id: str, source_address: tuple[str, int], action: RelayAction | None) -> None:
    if self._csv_writer is None:
      return
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
      "" if action is None else float(action.a_lead),
      "" if action is None else float(action.v_lead),
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
      event_type, role, device_id, action = self.router.handle_datagram(payload, address)
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

