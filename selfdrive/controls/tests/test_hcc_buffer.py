"""Buffer tests: staleness, replay rejection and publisher-session resets.

This is the component that produced 0% loss over 1750 packets on the bench, so
the behaviors below are regression locks, not aspirations. Every test drives
the clock explicitly rather than sleeping.
"""
import pytest

from openpilot.selfdrive.controls.lib.hcc.transport.buffer import V2VLeadBuffer, V2VPedalBuffer
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  DEFAULT_PEDAL_STALE_THRESHOLD_MS,
  DEFAULT_STALE_THRESHOLD_MS,
  SESSION_RESET_SEQ_GAP,
  V2VLeadPacket,
  V2VPedalPacket,
)

T0 = 1_743_000_000_000_000
NS = 1_000_000  # ns per ms


def _lead(seq, v=20.0, a=0.5, ts=T0, device_id="lead-1"):
  return V2VLeadPacket(device_id=device_id, timestamp_us=ts, a_lead=a, v_lead=v, seq=seq)


def _pedal(seq, throttle=0.0, brake=0.0, ts=T0):
  return V2VPedalPacket(device_id="hcc-pedals", timestamp_us=ts, throttle=throttle, brake=brake, seq=seq)


def _ingest(buf, packet, at_ms=0, **kwargs):
  return buf.ingest(packet, recv_monotonic_ns=at_ms * NS, recv_wall_time_us=packet.timestamp_us, **kwargs)


# ---- empty / fresh / stale -----------------------------------------------

def test_empty_buffer_is_not_valid():
  signal = V2VLeadBuffer().snapshot()
  assert not signal.status
  assert signal.seq == -1
  assert signal.receive_age_ms == float("inf")


def test_fresh_packet_is_valid_and_carries_its_values():
  buf = V2VLeadBuffer()
  assert _ingest(buf, _lead(1, v=14.5, a=-1.25), at_ms=0)
  signal = buf.snapshot(now_monotonic_ns=10 * NS)
  assert signal.status
  assert signal.lead_speed_mps == 14.5
  assert signal.lead_accel_mps2 == -1.25
  assert signal.seq == 1
  assert signal.source_device_id == "lead-1"
  assert signal.receive_age_ms == pytest.approx(10.0)


def test_staleness_boundary_is_inclusive():
  buf = V2VLeadBuffer()
  _ingest(buf, _lead(1), at_ms=0)
  at_threshold = int(DEFAULT_STALE_THRESHOLD_MS) * NS
  assert buf.snapshot(now_monotonic_ns=at_threshold).status
  assert not buf.snapshot(now_monotonic_ns=at_threshold + NS).status


def test_stale_signal_keeps_its_values_but_drops_status():
  # The controller must be able to tell "stale" from "never received".
  buf = V2VLeadBuffer()
  _ingest(buf, _lead(5, v=12.0), at_ms=0)
  signal = buf.snapshot(now_monotonic_ns=int(DEFAULT_STALE_THRESHOLD_MS + 100) * NS)
  assert not signal.status
  assert signal.seq == 5
  assert signal.lead_speed_mps == 12.0
  assert not signal.local_receive_valid


def test_stale_threshold_clears_the_once_per_second_starvation_flap():
  # Field-calibrated: ~100 ms of scheduling starvation about once a second
  # tripped a 100 ms threshold on a healthy 50 Hz stream (field bug 3).
  assert DEFAULT_STALE_THRESHOLD_MS >= 300.0


def test_pedal_buffer_is_stricter_than_the_lead_buffer():
  # 300 ms of stale lead is small against a 1.5 s headway; 300 ms of stale
  # pedal is a long time to hold a human's last brake command.
  assert DEFAULT_PEDAL_STALE_THRESHOLD_MS < DEFAULT_STALE_THRESHOLD_MS
  assert V2VPedalBuffer().stale_threshold_ms == DEFAULT_PEDAL_STALE_THRESHOLD_MS


# ---- replay / duplicate / reorder ----------------------------------------

def test_duplicate_and_replayed_seq_are_rejected():
  buf = V2VLeadBuffer()
  assert _ingest(buf, _lead(10), at_ms=0)
  assert not _ingest(buf, _lead(10), at_ms=1)   # exact duplicate
  assert not _ingest(buf, _lead(9), at_ms=2)    # reorder
  assert not _ingest(buf, _lead(1), at_ms=3)    # old replay
  assert buf.snapshot(now_monotonic_ns=0).seq == 10


def test_forward_progress_is_accepted_including_gaps():
  buf = V2VLeadBuffer()
  assert _ingest(buf, _lead(1), at_ms=0)
  assert _ingest(buf, _lead(2), at_ms=20)
  assert _ingest(buf, _lead(40), at_ms=40)  # dropped packets, still forward
  assert buf.snapshot(now_monotonic_ns=40 * NS).seq == 40


def test_rejected_packet_does_not_refresh_the_timestamp():
  # A replay must not be able to make a stale signal look fresh again.
  buf = V2VLeadBuffer()
  _ingest(buf, _lead(10), at_ms=0)
  _ingest(buf, _lead(10), at_ms=1000)
  assert not buf.snapshot(now_monotonic_ns=1000 * NS).status


# ---- publisher session resets --------------------------------------------

def test_large_backward_jump_adopts_the_new_publisher_session():
  # A restarted publisher numbers from 0 again; without this the ego would
  # ignore the new session forever.
  buf = V2VLeadBuffer()
  _ingest(buf, _lead(500), at_ms=0)
  assert _ingest(buf, _lead(0), at_ms=20)
  assert buf.snapshot(now_monotonic_ns=20 * NS).seq == 0


def test_session_reset_boundary():
  buf = V2VLeadBuffer()
  _ingest(buf, _lead(100), at_ms=0)
  # Exactly at the gap: still treated as a replay within the session.
  assert not _ingest(buf, _lead(100 - SESSION_RESET_SEQ_GAP), at_ms=10)
  # One further back: a new session.
  assert _ingest(buf, _lead(100 - SESSION_RESET_SEQ_GAP - 1), at_ms=20)


# ---- clock skew and address pinning gate ingestion -----------------------

def test_implausible_sender_timestamp_is_rejected():
  buf = V2VLeadBuffer()
  skewed = _lead(1, ts=T0)
  assert not buf.ingest(skewed, recv_monotonic_ns=0, recv_wall_time_us=T0 + 10_000_000)
  assert not buf.snapshot(now_monotonic_ns=0).status


def test_invalid_source_address_is_rejected():
  buf = V2VLeadBuffer()
  assert not _ingest(buf, _lead(1), at_ms=0, source_address_valid=False)
  assert not buf.snapshot(now_monotonic_ns=0).status


# ---- pedal buffer --------------------------------------------------------

def test_pedal_snapshot_carries_normalized_and_raw_values():
  buf = V2VPedalBuffer()
  packet = V2VPedalPacket(device_id="hcc-pedals", timestamp_us=T0, throttle=0.4, brake=0.6,
                          seq=3, throttle_raw=2048.0, brake_raw=1024.0)
  assert _ingest(buf, packet, at_ms=0)
  signal = buf.snapshot(now_monotonic_ns=0)
  assert signal.status
  assert signal.throttle == 0.4
  assert signal.brake == 0.6
  assert signal.throttle_raw == 2048.0
  assert signal.brake_raw == 1024.0


def test_pedal_buffer_shares_the_session_reset_behavior():
  buf = V2VPedalBuffer()
  _ingest(buf, _pedal(300), at_ms=0)
  assert _ingest(buf, _pedal(0), at_ms=20)  # laptop restarted the publisher
  assert buf.snapshot(now_monotonic_ns=20 * NS).seq == 0
