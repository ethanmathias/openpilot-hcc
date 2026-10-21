"""Relay routing tests.

The relay is the one place that sees all three peers, so it is where a packet
from the wrong sender has to be stopped. These tests drive the router directly
with datagrams and addresses rather than opening sockets.

The lead path is the component measured at 0% loss over 1750 packets; the pedal
branch was added ahead of it specifically so the lead path stayed untouched.
Several tests below exist to prove the two do not interfere.
"""
import json

import pytest

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  ROLE_EGO,
  ROLE_LEAD,
  ROLE_PEDAL,
  TYPE_DATA,
  TYPE_HELLO,
  TYPE_PEDAL,
  V2VHelloPacket,
  V2VLeadPacket,
  V2VPedalPacket,
  encode_data_packet,
  encode_hello_packet,
  encode_pedal_packet,
  wall_time_us,
)
from openpilot.selfdrive.controls.lib.hcc.transport.relay import (
  RELAY_CSV_COLUMNS,
  PedalRelayAction,
  RelayAction,
  RelayRouter,
)

LEAD_ADDR = ("10.42.0.50", 40001)
EGO_ADDR = ("10.42.0.1", 40002)
PEDAL_ADDR = ("10.42.0.77", 40003)
ATTACKER_ADDR = ("10.42.0.99", 40004)


def _lead_payload(seq=1, v=20.0, a=0.5, device_id="lead-1"):
  return encode_data_packet(V2VLeadPacket(device_id=device_id, timestamp_us=wall_time_us(),
                                          a_lead=a, v_lead=v, seq=seq))


def _pedal_payload(seq=1, throttle=0.0, brake=0.0, device_id="hcc-pedals"):
  return encode_pedal_packet(V2VPedalPacket(device_id=device_id, timestamp_us=wall_time_us(),
                                            throttle=throttle, brake=brake, seq=seq))


def _hello(role, device_id):
  return encode_hello_packet(V2VHelloPacket(device_id=device_id, role=role))


def _router(register_lead=True, register_ego=True, register_pedal=False, **kwargs):
  router = RelayRouter(**kwargs)
  if register_lead:
    router.handle_datagram(_hello(ROLE_LEAD, "lead-1"), LEAD_ADDR)
  if register_ego:
    router.handle_datagram(_hello(ROLE_EGO, "hcc-ego"), EGO_ADDR)
  if register_pedal:
    router.handle_datagram(_hello(ROLE_PEDAL, "hcc-pedals"), PEDAL_ADDR)
  return router


# ---- registration --------------------------------------------------------

def test_hello_registers_each_role_at_its_source_address():
  router = _router(register_pedal=True)
  assert router.lead_peer.address == LEAD_ADDR
  assert router.ego_peer.address == EGO_ADDR
  assert router.pedal_peer.address == PEDAL_ADDR
  assert router.last_reason == "registered_peer"


def test_hello_returns_no_action():
  router = RelayRouter()
  event_type, role, device_id, action = router.handle_datagram(_hello(ROLE_LEAD, "lead-1"), LEAD_ADDR)
  assert (event_type, role, device_id) == (TYPE_HELLO, ROLE_LEAD, "lead-1")
  assert action is None


def test_re_hello_moves_a_peer_to_its_new_address():
  # Devices change address when the hotspot re-issues a lease mid-session.
  router = _router()
  new_addr = ("10.42.0.51", 40009)
  router.handle_datagram(_hello(ROLE_LEAD, "lead-1"), new_addr)
  assert router.lead_peer.address == new_addr
  _, _, _, action = router.handle_datagram(_lead_payload(seq=2), new_addr)
  assert action is not None


# ---- lead path -----------------------------------------------------------

def test_lead_packet_forwards_to_the_ego():
  router = _router()
  event_type, role, device_id, action = router.handle_datagram(_lead_payload(seq=7, v=15.0, a=-1.0), LEAD_ADDR)
  assert (event_type, role) == (TYPE_DATA, ROLE_LEAD)
  assert isinstance(action, RelayAction)
  assert action.destination == EGO_ADDR
  assert action.seq == 7
  assert action.v_lead == 15.0
  assert action.a_lead == -1.0
  assert router.last_reason == "forwarded"


def test_untyped_payload_still_routes_as_lead_data():
  router = _router()
  payload = json.dumps({"device_id": "lead-1", "timestamp_us": wall_time_us(),
                        "a_lead": 0.0, "v_lead": 10.0, "seq": 1}).encode()
  _, _, _, action = router.handle_datagram(payload, LEAD_ADDR)
  assert isinstance(action, RelayAction)


@pytest.mark.parametrize("kwargs,reason", [
  (dict(register_lead=False), "missing_lead_peer"),
  (dict(register_ego=False), "missing_ego_peer"),
])
def test_lead_packet_dropped_until_both_peers_are_known(kwargs, reason):
  router = _router(**kwargs)
  _, _, _, action = router.handle_datagram(_lead_payload(), LEAD_ADDR)
  assert action is None
  assert router.last_reason == reason


def test_lead_packet_from_the_wrong_address_is_dropped():
  router = _router()
  _, _, _, action = router.handle_datagram(_lead_payload(), ATTACKER_ADDR)
  assert action is None
  assert router.last_reason == "sender_address_mismatch"


def test_lead_packet_with_the_wrong_device_id_is_dropped():
  router = _router()
  _, _, _, action = router.handle_datagram(_lead_payload(device_id="someone-else"), LEAD_ADDR)
  assert action is None
  assert router.last_reason == "device_id_mismatch"


# ---- pedal path ----------------------------------------------------------

def test_pedal_packet_forwards_to_the_ego():
  router = _router(register_pedal=True)
  event_type, role, device_id, action = router.handle_datagram(
    _pedal_payload(seq=3, throttle=0.25, brake=0.5), PEDAL_ADDR)
  assert (event_type, role, device_id) == (TYPE_PEDAL, ROLE_PEDAL, "hcc-pedals")
  assert isinstance(action, PedalRelayAction)
  assert action.destination == EGO_ADDR
  assert action.throttle == 0.25
  assert action.brake == 0.5
  assert router.last_reason == "forwarded"


def test_pedal_packet_dropped_before_the_pedal_peer_says_hello():
  router = _router(register_pedal=False)
  _, _, _, action = router.handle_datagram(_pedal_payload(), PEDAL_ADDR)
  assert action is None
  assert router.last_reason == "missing_pedal_peer"


def test_pedal_packet_from_the_wrong_address_is_dropped():
  router = _router(register_pedal=True)
  _, _, _, action = router.handle_datagram(_pedal_payload(), ATTACKER_ADDR)
  assert action is None
  assert router.last_reason == "sender_address_mismatch"


def test_pedal_packet_with_the_wrong_device_id_is_dropped():
  router = _router(register_pedal=True)
  _, _, _, action = router.handle_datagram(_pedal_payload(device_id="not-the-rig"), PEDAL_ADDR)
  assert action is None
  assert router.last_reason == "device_id_mismatch"


def test_pedal_packet_needs_the_ego_but_not_the_lead():
  # Bench work runs the rig with no lead device present at all.
  router = _router(register_lead=False, register_ego=True, register_pedal=True)
  _, _, _, action = router.handle_datagram(_pedal_payload(), PEDAL_ADDR)
  assert isinstance(action, PedalRelayAction)


def test_pedal_packet_dropped_without_an_ego():
  router = _router(register_ego=False, register_pedal=True)
  _, _, _, action = router.handle_datagram(_pedal_payload(), PEDAL_ADDR)
  assert action is None
  assert router.last_reason == "missing_ego_peer"


# ---- the two paths must not interfere ------------------------------------

def test_a_pedal_publisher_does_not_disturb_the_lead_stream():
  # On hcc-ego the single-lead router dropped any non-lead device_id as
  # device_id_mismatch, so a pedal publisher on the same port broke nothing
  # but was itself discarded on every packet. Both must now flow.
  router = _router(register_pedal=True)
  for seq in range(1, 6):
    _, _, _, lead_action = router.handle_datagram(_lead_payload(seq=seq), LEAD_ADDR)
    _, _, _, pedal_action = router.handle_datagram(_pedal_payload(seq=seq), PEDAL_ADDR)
    assert isinstance(lead_action, RelayAction)
    assert isinstance(pedal_action, PedalRelayAction)


def test_pedal_payload_from_the_lead_address_is_not_accepted_as_lead():
  router = _router(register_pedal=True)
  _, _, _, action = router.handle_datagram(_pedal_payload(), LEAD_ADDR)
  assert action is None


def test_lead_payload_from_the_pedal_address_is_not_accepted():
  router = _router(register_pedal=True)
  _, _, _, action = router.handle_datagram(_lead_payload(), PEDAL_ADDR)
  assert action is None
  assert router.last_reason == "sender_address_mismatch"


# ---- optional stream validation -----------------------------------------

def test_stream_validation_rejects_replayed_seq_on_both_paths():
  router = _router(register_pedal=True, enforce_stream_validation=True)
  assert router.handle_datagram(_lead_payload(seq=5), LEAD_ADDR)[3] is not None
  assert router.handle_datagram(_lead_payload(seq=5), LEAD_ADDR)[3] is None
  assert router.last_reason == "duplicate_or_replayed_seq"

  assert router.handle_datagram(_pedal_payload(seq=5), PEDAL_ADDR)[3] is not None
  assert router.handle_datagram(_pedal_payload(seq=4), PEDAL_ADDR)[3] is None
  assert router.last_reason == "duplicate_or_replayed_seq"


def test_stream_validation_is_off_by_default():
  # The relay forwards and logs; the ego's buffer is what rejects replays in
  # normal operation, so duplicate suppression here is opt-in.
  router = _router()
  assert router.handle_datagram(_lead_payload(seq=5), LEAD_ADDR)[3] is not None
  assert router.handle_datagram(_lead_payload(seq=5), LEAD_ADDR)[3] is not None


def test_stream_validation_tracks_devices_independently():
  router = _router(register_pedal=True, enforce_stream_validation=True)
  router.handle_datagram(_lead_payload(seq=9), LEAD_ADDR)
  # Same seq, different device: must not be mistaken for a replay.
  assert router.handle_datagram(_pedal_payload(seq=9), PEDAL_ADDR)[3] is not None


# ---- CSV schema ----------------------------------------------------------

def test_csv_carries_both_lead_and_pedal_columns():
  for column in ("a_lead", "v_lead", "throttle", "brake", "reason", "forwarded"):
    assert column in RELAY_CSV_COLUMNS
