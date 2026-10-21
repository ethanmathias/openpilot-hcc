"""Staleness, clock-skew and session-reset handling for received V2V packets.

This is the component with the measured 0% loss over 1750 packets on the
2026-06-12 bench run, so the logic is kept intact and shared rather than
reimplemented per packet type: V2VPacketBuffer holds it, and the lead and
pedal buffers only differ in which signal dataclass they emit.
"""
from __future__ import annotations

import threading

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  DEFAULT_PEDAL_STALE_THRESHOLD_MS,
  DEFAULT_STALE_THRESHOLD_MS,
  MAX_TIMESTAMP_SKEW_MS,
  SESSION_RESET_SEQ_GAP,
  ReceivedPacket,
  V2VLeadSignal,
  V2VPedalSignal,
  is_timestamp_plausible,
  monotonic_time_ns,
  packet_receive_age_ms,
  wall_time_us,
)


class V2VPacketBuffer:
  """Holds the most recent accepted packet, whatever its shape.

  Accepts a packet only when its sender timestamp is plausible, its source
  address is allowed, and its seq represents forward progress -- or a backward
  jump large enough to mean the publisher restarted.
  """

  def __init__(self,
               stale_threshold_ms: float = DEFAULT_STALE_THRESHOLD_MS,
               max_clock_skew_ms: float = MAX_TIMESTAMP_SKEW_MS):
    self.stale_threshold_ms = stale_threshold_ms
    self.max_clock_skew_ms = max_clock_skew_ms
    self._latest: ReceivedPacket | None = None
    self._last_seq = -1
    self._lock = threading.Lock()

  def ingest(self,
             packet,
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
      # (a new session always numbers from 0) -- adopt the new session.
      self._last_seq = packet.seq
      self._latest = ReceivedPacket(
        packet=packet,
        local_recv_monotonic_ns=received_monotonic_ns,
        local_recv_wall_time_us=received_wall_time_us,
        sender_timestamp_valid=True,
        source_address_valid=True,
      )
    return True

  def latest(self, now_monotonic_ns: int | None = None) -> tuple[ReceivedPacket | None, float, bool]:
    """The newest accepted packet, its local age in ms, and whether it is fresh."""
    current_monotonic_ns = monotonic_time_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
    with self._lock:
      latest = self._latest

    if latest is None:
      return None, float("inf"), False

    receive_age_ms = packet_receive_age_ms(latest.local_recv_monotonic_ns, current_monotonic_ns)
    return latest, receive_age_ms, receive_age_ms <= self.stale_threshold_ms


class V2VLeadBuffer(V2VPacketBuffer):
  def snapshot(self, now_monotonic_ns: int | None = None) -> V2VLeadSignal:
    latest, receive_age_ms, fresh = self.latest(now_monotonic_ns)
    if latest is None:
      return V2VLeadSignal(status=False)

    return V2VLeadSignal(
      status=bool(fresh and latest.sender_timestamp_valid and latest.source_address_valid),
      lead_speed_mps=float(latest.packet.v_lead),
      lead_accel_mps2=float(latest.packet.a_lead),
      seq=int(latest.packet.seq),
      source_device_id=latest.packet.device_id,
      local_receive_valid=fresh,
      sender_timestamp_valid=latest.sender_timestamp_valid,
      source_address_valid=latest.source_address_valid,
      receive_age_ms=receive_age_ms,
    )


class V2VPedalBuffer(V2VPacketBuffer):
  def __init__(self,
               stale_threshold_ms: float = DEFAULT_PEDAL_STALE_THRESHOLD_MS,
               max_clock_skew_ms: float = MAX_TIMESTAMP_SKEW_MS):
    super().__init__(stale_threshold_ms, max_clock_skew_ms)

  def snapshot(self, now_monotonic_ns: int | None = None) -> V2VPedalSignal:
    latest, receive_age_ms, fresh = self.latest(now_monotonic_ns)
    if latest is None:
      return V2VPedalSignal(status=False)

    return V2VPedalSignal(
      status=bool(fresh and latest.sender_timestamp_valid and latest.source_address_valid),
      throttle=float(latest.packet.throttle),
      brake=float(latest.packet.brake),
      throttle_raw=float(latest.packet.throttle_raw),
      brake_raw=float(latest.packet.brake_raw),
      seq=int(latest.packet.seq),
      source_device_id=latest.packet.device_id,
      local_receive_valid=fresh,
      sender_timestamp_valid=latest.sender_timestamp_valid,
      source_address_valid=latest.source_address_valid,
      receive_age_ms=receive_age_ms,
    )
