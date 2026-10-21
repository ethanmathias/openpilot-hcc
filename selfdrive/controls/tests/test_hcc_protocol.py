"""Wire-format tests: round trips, validation, and the checks that decide
whether a datagram is allowed to influence control at all."""
import json

import pytest

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  MAX_TIMESTAMP_SKEW_MS,
  ROLE_EGO,
  ROLE_LEAD,
  ROLE_PEDAL,
  TYPE_DATA,
  TYPE_HELLO,
  TYPE_PEDAL,
  V2VConfig,
  V2VHelloPacket,
  V2VLeadPacket,
  V2VPedalPacket,
  encode_data_packet,
  encode_hello_packet,
  encode_pedal_packet,
  is_timestamp_plausible,
  message_type,
  packet_receive_age_ms,
  parse_data_packet,
  parse_hello_packet,
  parse_pedal_packet,
  source_address_matches,
)

T0 = 1_743_000_000_000_000


def _config(**kwargs):
  base = dict(role=ROLE_EGO, enabled=True, relay_host="127.0.0.1", relay_port=19090, device_id="hcc-ego")
  base.update(kwargs)
  return V2VConfig(**base)


# ---- round trips ----------------------------------------------------------

def test_lead_packet_round_trip():
  packet = V2VLeadPacket(device_id="lead-1", timestamp_us=T0, a_lead=-1.25, v_lead=14.5, seq=42)
  assert parse_data_packet(encode_data_packet(packet)) == packet


def test_pedal_packet_round_trip():
  packet = V2VPedalPacket(device_id="hcc-pedals", timestamp_us=T0, throttle=0.25, brake=0.75,
                          seq=7, throttle_raw=1024.0, brake_raw=512.0)
  assert parse_pedal_packet(encode_pedal_packet(packet)) == packet


def test_hello_packet_round_trip():
  for role in (ROLE_LEAD, ROLE_EGO, ROLE_PEDAL):
    packet = V2VHelloPacket(device_id=f"dev-{role}", role=role)
    assert parse_hello_packet(encode_hello_packet(packet)) == packet


def test_pedal_raw_counts_are_optional():
  payload = json.dumps({"type": TYPE_PEDAL, "device_id": "p", "timestamp_us": T0,
                        "throttle": 0.5, "brake": 0.0, "seq": 1}).encode()
  parsed = parse_pedal_packet(payload)
  assert parsed.throttle_raw == 0.0 and parsed.brake_raw == 0.0


def test_message_type_defaults_to_data():
  # Lead publishers predating the pedal work omit "type"; they must still route.
  payload = json.dumps({"device_id": "l", "timestamp_us": T0, "a_lead": 0.0,
                        "v_lead": 10.0, "seq": 1}).encode()
  assert message_type(payload) == TYPE_DATA
  assert parse_data_packet(payload).v_lead == 10.0


def test_encoding_is_deterministic():
  # Sorted keys and no whitespace: byte-identical for identical content, which
  # is what makes the relay CSV and the publisher CSV comparable.
  packet = V2VLeadPacket(device_id="l", timestamp_us=T0, a_lead=1.0, v_lead=2.0, seq=3)
  assert encode_data_packet(packet) == encode_data_packet(packet)
  assert b" " not in encode_data_packet(packet)


# ---- the packet types must not be confusable ------------------------------

def test_parsers_reject_the_other_packet_type():
  lead = encode_data_packet(V2VLeadPacket(device_id="l", timestamp_us=T0, a_lead=0.0, v_lead=1.0, seq=1))
  pedal = encode_pedal_packet(V2VPedalPacket(device_id="p", timestamp_us=T0, throttle=0.0, brake=0.0, seq=1))
  hello = encode_hello_packet(V2VHelloPacket(device_id="d", role=ROLE_LEAD))

  with pytest.raises(ValueError):
    parse_pedal_packet(lead)
  with pytest.raises(ValueError):
    parse_data_packet(pedal)
  with pytest.raises(ValueError):
    parse_data_packet(hello)
  with pytest.raises(ValueError):
    parse_hello_packet(lead)


# ---- validation -----------------------------------------------------------

@pytest.mark.parametrize("field,value", [("a_lead", float("nan")), ("a_lead", float("inf")),
                                         ("v_lead", float("-inf"))])
def test_lead_packet_rejects_non_finite(field, value):
  payload = {"type": TYPE_DATA, "device_id": "l", "timestamp_us": T0, "a_lead": 0.0, "v_lead": 1.0, "seq": 1}
  payload[field] = value
  with pytest.raises(ValueError):
    parse_data_packet(json.dumps(payload).encode())


@pytest.mark.parametrize("throttle,brake", [(-0.1, 0.0), (1.1, 0.0), (0.0, -0.5), (0.0, 2.0)])
def test_pedal_packet_rejects_out_of_range_fractions(throttle, brake):
  # A pedal fraction outside [0, 1] means a calibration fault at the publisher.
  # Rejecting it on the wire keeps a miscalibrated rig from commanding accel.
  payload = {"type": TYPE_PEDAL, "device_id": "p", "timestamp_us": T0,
             "throttle": throttle, "brake": brake, "seq": 1}
  with pytest.raises(ValueError):
    parse_pedal_packet(json.dumps(payload).encode())


@pytest.mark.parametrize("parse,ptype", [(parse_data_packet, TYPE_DATA), (parse_pedal_packet, TYPE_PEDAL)])
def test_packets_reject_empty_device_id_negative_seq_and_bad_timestamp(parse, ptype):
  body = {"a_lead": 0.0, "v_lead": 1.0} if ptype == TYPE_DATA else {"throttle": 0.0, "brake": 0.0}
  for bad in ({"device_id": "   "}, {"device_id": ""}, {"seq": -1}, {"timestamp_us": 0}, {"timestamp_us": -5}):
    payload = {"type": ptype, "device_id": "d", "timestamp_us": T0, "seq": 1, **body, **bad}
    with pytest.raises(ValueError):
      parse(json.dumps(payload).encode())


def test_hello_rejects_unknown_role():
  payload = json.dumps({"type": TYPE_HELLO, "device_id": "d", "role": "attacker"}).encode()
  with pytest.raises(ValueError):
    parse_hello_packet(payload)


def test_non_object_payload_is_rejected():
  with pytest.raises(ValueError):
    parse_data_packet(b"[1, 2, 3]")


# ---- clock skew and staleness --------------------------------------------

def test_timestamp_plausibility_window():
  assert is_timestamp_plausible(T0, T0)
  edge_us = int(MAX_TIMESTAMP_SKEW_MS * 1000.0)
  assert is_timestamp_plausible(T0, T0 + edge_us)
  assert is_timestamp_plausible(T0, T0 - edge_us)
  assert not is_timestamp_plausible(T0, T0 + edge_us + 1)
  assert not is_timestamp_plausible(T0, T0 - edge_us - 1)


def test_receive_age_is_never_negative():
  # Guards against a monotonic clock reading backwards across threads.
  assert packet_receive_age_ms(1_000_000_000, 900_000_000) == 0.0
  assert packet_receive_age_ms(1_000_000_000, 1_100_000_000) == pytest.approx(100.0)


# ---- source address pinning ----------------------------------------------

def test_source_address_must_match_expected_host_and_port():
  config = _config(expected_sender_host="127.0.0.1", expected_sender_port=19090)
  assert source_address_matches(config, ("127.0.0.1", 19090))
  assert not source_address_matches(config, ("127.0.0.1", 19091))
  assert not source_address_matches(config, ("10.42.0.99", 19090))


def test_any_policy_accepts_all_sources():
  config = _config(sender_address_policy="any")
  assert source_address_matches(config, ("10.0.0.1", 1))
