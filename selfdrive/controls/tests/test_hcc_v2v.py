from openpilot.selfdrive.controls.lib.hcc_v2v import (
  MAX_TIMESTAMP_SKEW_MS,
  ROLE_EGO,
  ROLE_LEAD,
  TYPE_DATA,
  TYPE_HELLO,
  V2VHelloPacket,
  V2VLeadBuffer,
  V2VLeadPacket,
  encode_data_packet,
  encode_hello_packet,
  is_timestamp_plausible,
  parse_data_packet,
  parse_hello_packet,
)
from openpilot.tools.hcc_v2v.relay_server import RelayRouter


def test_data_packet_round_trip():
  packet = V2VLeadPacket(device_id="lead-1", timestamp_us=1_743_000_000_000_000, a_lead=-1.25, v_lead=14.5, seq=42)
  parsed = parse_data_packet(encode_data_packet(packet))
  assert parsed == packet


def test_hello_packet_round_trip():
  packet = V2VHelloPacket(device_id="ego-1", role=ROLE_EGO)
  parsed = parse_hello_packet(encode_hello_packet(packet))
  assert parsed == packet


def test_timestamp_plausibility_window():
  now_us = 1_743_000_000_000_000
  assert is_timestamp_plausible(now_us, now_us)
  assert is_timestamp_plausible(now_us - int(MAX_TIMESTAMP_SKEW_MS * 1000), now_us)
  assert not is_timestamp_plausible(now_us - int((MAX_TIMESTAMP_SKEW_MS + 1) * 1000), now_us)


def test_lead_buffer_rejects_duplicate_and_stale_packets():
  buffer = V2VLeadBuffer(stale_threshold_ms=100.0, max_clock_skew_ms=500.0)
  base_wall_us = 1_743_000_000_000_000
  first = V2VLeadPacket(device_id="lead-1", timestamp_us=base_wall_us, a_lead=0.5, v_lead=12.0, seq=3)
  duplicate = V2VLeadPacket(device_id="lead-1", timestamp_us=base_wall_us, a_lead=0.7, v_lead=13.0, seq=3)

  assert buffer.ingest(first, recv_monotonic_ns=1_000_000_000, recv_wall_time_us=base_wall_us)
  assert not buffer.ingest(duplicate, recv_monotonic_ns=1_010_000_000, recv_wall_time_us=base_wall_us)

  fresh_signal = buffer.snapshot(now_monotonic_ns=1_050_000_000)
  assert fresh_signal.status is True
  assert fresh_signal.seq == 3
  assert abs(fresh_signal.lead_speed_mps - 12.0) < 1e-6

  stale_signal = buffer.snapshot(now_monotonic_ns=1_200_000_001)
  assert stale_signal.status is False
  assert stale_signal.local_receive_valid is False


def test_lead_buffer_rejects_implausible_sender_timestamp():
  buffer = V2VLeadBuffer(stale_threshold_ms=100.0, max_clock_skew_ms=500.0)
  packet = V2VLeadPacket(
    device_id="lead-1",
    timestamp_us=1_743_000_000_000_000,
    a_lead=0.1,
    v_lead=4.0,
    seq=1,
  )
  assert not buffer.ingest(packet, recv_monotonic_ns=10, recv_wall_time_us=packet.timestamp_us + 600_000)
  assert buffer.snapshot().status is False


def test_relay_router_registers_peers_and_forwards_lead_packets():
  router = RelayRouter()
  lead_hello = encode_hello_packet(V2VHelloPacket(device_id="lead-1", role=ROLE_LEAD))
  ego_hello = encode_hello_packet(V2VHelloPacket(device_id="ego-1", role=ROLE_EGO))
  lead_packet = encode_data_packet(V2VLeadPacket(device_id="lead-1", timestamp_us=1_743_000_000_000_000, a_lead=-0.8, v_lead=11.3, seq=8))

  event_type, role, device_id, action = router.handle_datagram(lead_hello, ("127.0.0.1", 20001))
  assert event_type == TYPE_HELLO
  assert role == ROLE_LEAD
  assert device_id == "lead-1"
  assert action is None

  event_type, role, device_id, action = router.handle_datagram(ego_hello, ("127.0.0.1", 20002))
  assert event_type == TYPE_HELLO
  assert role == ROLE_EGO
  assert device_id == "ego-1"
  assert action is None

  event_type, role, device_id, action = router.handle_datagram(lead_packet, ("127.0.0.1", 20001))
  assert event_type == TYPE_DATA
  assert role == ROLE_LEAD
  assert device_id == "lead-1"
  assert action is not None
  assert action.destination == ("127.0.0.1", 20002)
  assert action.seq == 8


def test_relay_router_does_not_forward_unregistered_sender():
  router = RelayRouter()
  router.handle_datagram(encode_hello_packet(V2VHelloPacket(device_id="ego-1", role=ROLE_EGO)), ("127.0.0.1", 20002))
  event_type, _, device_id, action = router.handle_datagram(
    encode_data_packet(V2VLeadPacket(device_id="lead-1", timestamp_us=1_743_000_000_000_000, a_lead=0.1, v_lead=8.0, seq=1)),
    ("127.0.0.1", 20001),
  )
  assert event_type == TYPE_DATA
  assert device_id == "lead-1"
  assert action is None
