"""Blend tests: the human-in-the-loop term and its failure behavior.

The failure behavior is the part that matters most here. A stale lead stream is
safe by construction -- zero command, the car coasts. A stale *pedal* stream is
not: if the subject is holding the brake when the link drops, their brake
request vanishes and the controller command, possibly positive, resumes alone.
"""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.hcc.blend import (
  ROAD_PEDAL_BRAKE_GAIN,
  ROAD_PEDAL_GAS_GAIN,
  SOURCE_CARSTATE,
  SOURCE_NONE,
  SOURCE_PEDAL,
  ManualBlend,
  normalize_pedal,
  pedal_to_accel,
)
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import V2VPedalSignal

LIMITS = (-3.5, 2.0)


def _pedal(throttle=0.0, brake=0.0, status=True, age_ms=10.0, seq=5):
  return V2VPedalSignal(status=status, throttle=throttle, brake=brake, seq=seq, receive_age_ms=age_ms)


def _cs(gas=None, brake=None):
  fields = {}
  if gas is not None:
    fields["gas"] = gas
  if brake is not None:
    fields["brake"] = brake
  return SimpleNamespace(**fields)


# ---- normalization -------------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
  (0.0, 0.0), (0.5, 0.5), (1.0, 1.0),
  (50.0, 0.5), (100.0, 1.0),   # percent form
  (-0.2, 0.0), (150.0, 1.0),   # clipped
])
def test_normalize_accepts_fraction_or_percent(raw, expected):
  assert normalize_pedal(raw) == pytest.approx(expected)


def test_normalize_boundary_between_fraction_and_percent():
  # 1.0 is a full fraction, not 1%. Anything above it is read as percent.
  assert normalize_pedal(1.0) == 1.0
  assert normalize_pedal(1.5) == pytest.approx(0.015)


# ---- pedal to accel ------------------------------------------------------

def test_full_throttle_and_full_brake_hit_the_gains():
  assert pedal_to_accel(1.0, 0.0) == pytest.approx(ROAD_PEDAL_GAS_GAIN)
  assert pedal_to_accel(0.0, 1.0) == pytest.approx(-ROAD_PEDAL_BRAKE_GAIN)


def test_brake_authority_exceeds_throttle_authority():
  assert ROAD_PEDAL_BRAKE_GAIN > ROAD_PEDAL_GAS_GAIN


def test_both_pedals_cancel():
  assert pedal_to_accel(0.6, 0.6) == pytest.approx(0.0)


def test_brake_wins_when_pressed_harder():
  assert pedal_to_accel(0.3, 0.8) < 0.0


def test_neutral_is_exactly_zero():
  assert pedal_to_accel(0.0, 0.0) == 0.0


# ---- pedal source --------------------------------------------------------

def test_healthy_pedal_signal_drives_the_manual_term():
  blend = ManualBlend(pedal_enabled=True)
  manual, limits = blend.update(_cs(), _pedal(throttle=0.5), LIMITS)
  assert manual == pytest.approx(0.5 * ROAD_PEDAL_GAS_GAIN)
  assert blend.source == SOURCE_PEDAL
  assert blend.pedal_ok
  assert not blend.authority_clamped
  assert limits == LIMITS


def test_pedal_source_ignores_carstate_entirely():
  # The whole point: the car's own pedals leave the critical path.
  blend = ManualBlend(pedal_enabled=True)
  manual, _ = blend.update(_cs(gas=1.0, brake=0.0), _pedal(brake=1.0), LIMITS)
  assert manual == pytest.approx(-ROAD_PEDAL_BRAKE_GAIN)


# ---- the stale-link clamp ------------------------------------------------

def test_stale_pedal_zeroes_the_term_and_denies_positive_authority():
  blend = ManualBlend(pedal_enabled=True)
  manual, limits = blend.update(_cs(), _pedal(brake=1.0, status=False), LIMITS)
  assert manual == 0.0
  assert blend.authority_clamped
  assert limits[1] == 0.0        # no positive accel until the link recovers
  assert limits[0] == LIMITS[0]  # deceleration authority is untouched


def test_missing_pedal_signal_is_treated_as_stale():
  blend = ManualBlend(pedal_enabled=True)
  manual, limits = blend.update(_cs(), None, LIMITS)
  assert manual == 0.0
  assert blend.authority_clamped
  assert limits[1] == 0.0


def test_clamp_never_raises_an_already_negative_accel_max():
  # A planner already demanding deceleration must not be relaxed by the clamp.
  blend = ManualBlend(pedal_enabled=True)
  _, limits = blend.update(_cs(), None, (-3.5, -1.0))
  assert limits[1] == -1.0


def test_clamp_releases_when_the_link_recovers():
  blend = ManualBlend(pedal_enabled=True)
  blend.update(_cs(), None, LIMITS)
  assert blend.authority_clamped
  manual, limits = blend.update(_cs(), _pedal(throttle=1.0), LIMITS)
  assert not blend.authority_clamped
  assert limits == LIMITS
  assert manual == pytest.approx(ROAD_PEDAL_GAS_GAIN)


def test_clamp_applies_before_any_healthy_packet_arrives():
  # Denying positive authority until the rig is actually publishing is
  # intended: engaging with pedal mode on and no publisher should not creep.
  blend = ManualBlend(pedal_enabled=True)
  _, limits = blend.update(_cs(), V2VPedalSignal(status=False), LIMITS)
  assert limits[1] == 0.0


# ---- CarState fallback ---------------------------------------------------

def test_disabled_pedal_mode_falls_back_to_carstate():
  blend = ManualBlend(pedal_enabled=False)
  manual, limits = blend.update(_cs(gas=0.0, brake=1.0), None, LIMITS)
  assert manual == pytest.approx(-ROAD_PEDAL_BRAKE_GAIN)
  assert blend.source == SOURCE_CARSTATE
  assert limits == LIMITS


def test_carstate_fallback_never_clamps_authority():
  # No pedal stream is expected in this mode, so its absence is not a fault.
  blend = ManualBlend(pedal_enabled=False)
  _, limits = blend.update(_cs(), None, LIMITS)
  assert not blend.authority_clamped
  assert limits == LIMITS


def test_missing_carstate_pedal_fields_yield_zero():
  # This is the 2026-07-11 Tucson reality: CarState.gas is deprecated out of
  # the schema and Hyundai never sets CarState.brake, so the term was exactly
  # 0.0 in 100% of field samples. It must degrade to 0.0, not raise.
  blend = ManualBlend(pedal_enabled=False)
  manual, _ = blend.update(_cs(), None, LIMITS)
  assert manual == 0.0


# ---- bookkeeping ---------------------------------------------------------

def test_link_health_is_reported_for_the_monitor():
  blend = ManualBlend(pedal_enabled=True)
  blend.update(_cs(), _pedal(throttle=0.2, age_ms=42.0, seq=99), LIMITS)
  assert blend.pedal_age_ms == pytest.approx(42.0)
  assert blend.pedal_seq == 99


def test_reset_clears_state():
  blend = ManualBlend(pedal_enabled=True)
  blend.update(_cs(), _pedal(throttle=1.0), LIMITS)
  blend.reset()
  assert blend.manual_accel == 0.0
  assert blend.source == SOURCE_NONE
  assert not blend.pedal_ok
  assert not blend.authority_clamped
  assert blend.pedal_age_ms == float("inf")
