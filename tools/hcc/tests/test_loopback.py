"""End-to-end loopback: publisher -> relay -> subscriber, over real sockets.

Everything else in the suite tests a component in isolation. This drives the
whole V2V path on localhost, which is what catches the mistakes isolated tests
miss: a hello that never registers, an address pinned to the wrong port, a
packet type the relay routes but the subscriber cannot parse.

It is also the harness the bench step uses. Measuring the publisher-to-blend
latency distribution is this test plus a timestamp difference.
"""
from __future__ import annotations

import socket
import threading
import time

import pytest

from openpilot.selfdrive.controls.lib.hcc.transport.endpoints import (
  V2VLeadSubscriber,
  V2VPedalSubscriber,
)
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import ROLE_EGO, V2VConfig
from openpilot.selfdrive.controls.lib.hcc.transport.relay import UDPRelayServer
from openpilot.tools.hcc.publishers.pedal import PedalPublisher
from openpilot.tools.hcc.publishers.virtual_lead import VirtualLeadPublisher

HOST = "127.0.0.1"


def _free_port() -> int:
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind((HOST, 0))
    return s.getsockname()[1]


class _Relay:
  """UDPRelayServer on a background thread, stopped by closing its socket."""

  def __init__(self, port: int):
    self.server = UDPRelayServer(HOST, port)
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def _run(self):
    try:
      self.server.serve_forever()
    except OSError:
      pass  # socket closed from stop()

  def stop(self):
    self.server.close()
    self._thread.join(timeout=1.0)


def _ego_config(port: int, stale_ms: float = 2000.0) -> V2VConfig:
  return V2VConfig(role=ROLE_EGO, enabled=True, relay_host=HOST, relay_port=port,
                   device_id="hcc-ego", stale_threshold_ms=stale_ms,
                   expected_sender_host=HOST, expected_sender_port=port)


def _wait_for(predicate, timeout_s: float = 3.0) -> bool:
  deadline = time.monotonic() + timeout_s
  while time.monotonic() < deadline:
    if predicate():
      return True
    time.sleep(0.02)
  return False


@pytest.fixture
def relay_port():
  port = _free_port()
  relay = _Relay(port)
  try:
    yield port
  finally:
    relay.stop()


def test_lead_packets_reach_the_ego_subscriber(relay_port):
  subscriber = V2VLeadSubscriber(_ego_config(relay_port))
  subscriber.start()
  publisher = VirtualLeadPublisher(HOST, relay_port, "hcc-lead")
  try:
    assert _wait_for(lambda: subscriber.snapshot().seq >= 0 or _publish_lead(publisher))
    signal = subscriber.snapshot()
    assert signal.status
    assert signal.lead_speed_mps == pytest.approx(14.5)
    assert signal.lead_accel_mps2 == pytest.approx(-1.25)
    assert signal.source_device_id == "hcc-lead"
  finally:
    publisher.close()
    subscriber.stop()


def _publish_lead(publisher) -> bool:
  publisher.maybe_hello()
  publisher.publish(14.5, -1.25)
  return False


def test_pedal_packets_reach_the_ego_subscriber(relay_port):
  # The ego cannot tell a pedal packet's origin from the wire; the relay pins
  # it. This proves the pedal branch routes and the subscriber parses it.
  subscriber = V2VPedalSubscriber(_ego_config(relay_port))
  subscriber.start()
  publisher = PedalPublisher(HOST, relay_port, "hcc-pedals")
  try:
    def pump():
      publisher.maybe_hello()
      publisher.publish(0.4, 0.6, 1234.0, 5678.0)
      return subscriber.snapshot().status

    assert _wait_for(pump), "no pedal packet arrived at the subscriber"
    signal = subscriber.snapshot()
    assert signal.throttle == pytest.approx(0.4)
    assert signal.brake == pytest.approx(0.6)
    assert signal.throttle_raw == pytest.approx(1234.0)
    assert signal.brake_raw == pytest.approx(5678.0)
    assert signal.source_device_id == "hcc-pedals"
  finally:
    publisher.close()
    subscriber.stop()


def test_lead_and_pedal_streams_coexist_on_one_relay(relay_port):
  # Three peers on one hotspot and one port is the Fall-26 configuration.
  lead_sub = V2VLeadSubscriber(_ego_config(relay_port))
  pedal_sub = V2VPedalSubscriber(_ego_config(relay_port))
  lead_sub.start()
  pedal_sub.start()
  lead_pub = VirtualLeadPublisher(HOST, relay_port, "hcc-lead")
  pedal_pub = PedalPublisher(HOST, relay_port, "hcc-pedals")
  try:
    def pump():
      lead_pub.maybe_hello()
      pedal_pub.maybe_hello()
      lead_pub.publish(20.0, 0.5)
      pedal_pub.publish(0.1, 0.9, 1.0, 2.0)
      return lead_sub.snapshot().status and pedal_sub.snapshot().status

    assert _wait_for(pump), "both streams did not become healthy"
    assert lead_sub.snapshot().lead_speed_mps == pytest.approx(20.0)
    assert pedal_sub.snapshot().brake == pytest.approx(0.9)
  finally:
    lead_pub.close()
    pedal_pub.close()
    lead_sub.stop()
    pedal_sub.stop()


def test_stream_goes_stale_when_the_publisher_stops(relay_port):
  # The ego must notice silence. This is what the blend's accel_max clamp
  # keys off, so it has to actually happen.
  subscriber = V2VPedalSubscriber(_ego_config(relay_port, stale_ms=100.0))
  subscriber.start()
  publisher = PedalPublisher(HOST, relay_port, "hcc-pedals")
  try:
    def pump():
      publisher.maybe_hello()
      publisher.publish(0.5, 0.0, 0.0, 0.0)
      return subscriber.snapshot().status

    assert _wait_for(pump), "stream never became healthy"
    publisher.close()  # stop publishing
    assert _wait_for(lambda: not subscriber.snapshot().status, timeout_s=2.0)
    stale = subscriber.snapshot()
    assert not stale.status
    assert stale.throttle == pytest.approx(0.5)  # last value retained for diagnosis
  finally:
    subscriber.stop()
