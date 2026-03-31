#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import socket
import time
from dataclasses import dataclass
from pathlib import Path

from openpilot.selfdrive.controls.lib.hcc_v2v import (
  RECV_BUFFER_BYTES,
  ROLE_EGO,
  ROLE_LEAD,
  TYPE_DATA,
  TYPE_HELLO,
  V2VHelloPacket,
  V2VLeadPacket,
  decode_message,
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


class RelayRouter:
  def __init__(self):
    self._peers: dict[str, RegisteredPeer] = {}

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
      return (TYPE_HELLO, hello.role, hello.device_id, None)

    packet = parse_data_packet(payload)
    lead_peer = self.lead_peer
    ego_peer = self.ego_peer
    if lead_peer is None or ego_peer is None or packet.device_id != lead_peer.device_id:
      return (TYPE_DATA, ROLE_LEAD, packet.device_id, None)

    action = RelayAction(
      payload=payload,
      destination=ego_peer.address,
      device_id=packet.device_id,
      seq=packet.seq,
      a_lead=packet.a_lead,
      v_lead=packet.v_lead,
    )
    return (TYPE_DATA, ROLE_LEAD, packet.device_id, action)


class UDPRelayServer:
  def __init__(self, host: str, port: int, log_csv_path: str | None = None):
    self.host = host
    self.port = port
    self.router = RelayRouter()
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

  def _log_event(self,
                 event_type: str,
                 role: str,
                 device_id: str,
                 source_address: tuple[str, int],
                 action: RelayAction | None) -> None:
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
  parser = argparse.ArgumentParser(description="Minimal HCC UDP relay server.")
  parser.add_argument("--host", default="0.0.0.0", help="Host interface to bind the relay socket to.")
  parser.add_argument("--port", type=int, default=19090, help="UDP port to bind.")
  parser.add_argument("--log_csv", default=None, help="Optional CSV log output path.")
  return parser


def main() -> None:
  args = _build_arg_parser().parse_args()
  server = UDPRelayServer(args.host, args.port, args.log_csv)
  try:
    server.serve_forever()
  finally:
    server.close()


if __name__ == "__main__":
  main()
