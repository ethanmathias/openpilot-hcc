"""Where pedal readings come from, and how raw axes become fractions.

The rig's host requirement is still an open question -- whether the SRP2
presents itself as a standalone USB HID device or expects a wheelbase as its
host is not stated on the product page, and it gates the hardware build. So
the input is behind a small interface with two backends:

  JoystickPedalSource   a real USB HID pedal set, through pygame.joystick
  SyntheticPedalSource  a repeatable waveform, no hardware at all

The synthetic source is not a toy. Steps 2-4 of the order of work -- bench the
publisher to the blend site, measure the latency distribution, verify the
staleness and session-reset paths, then confirm the term moves on a stationary
car -- all need a publisher on the wire and none of them need pedals. It also
makes the whole path testable before the rig arrives.

Calibration is per session, not once. A load cell's rest offset moves with
temperature and mount preload, so each run captures rest and a max-force
reference. The raw counts go on the wire alongside the fractions so a bad
calibration can be corrected in post instead of invalidating the run.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass
class AxisCalibration:
  """Maps one raw axis onto [0, 1]."""
  rest: float = 0.0
  full: float = 1.0

  def normalize(self, raw: float) -> float:
    span = self.full - self.rest
    if abs(span) < 1e-9:
      return 0.0
    return min(max((float(raw) - self.rest) / span, 0.0), 1.0)

  @property
  def is_plausible(self) -> bool:
    return abs(self.full - self.rest) > 1e-6


class PedalSource:
  """Returns raw (throttle, brake) axis values in whatever units it reads."""
  name = "base"

  def read(self) -> tuple[float, float]:
    raise NotImplementedError

  def close(self) -> None:
    pass


class SyntheticPedalSource(PedalSource):
  """A deterministic throttle/brake waveform for bench runs.

  Alternates a throttle ramp and a brake ramp on a fixed period so the
  resulting manual_accel term is recognizable in the monitor CSV without
  anyone touching a pedal. Never applies both at once.
  """
  name = "synthetic"

  def __init__(self, period_s: float = 20.0, amplitude: float = 0.6):
    self.period_s = float(period_s)
    self.amplitude = min(max(float(amplitude), 0.0), 1.0)
    self._t0 = time.monotonic()

  def read(self) -> tuple[float, float]:
    phase = ((time.monotonic() - self._t0) % self.period_s) / self.period_s
    # First half throttle, second half brake, each a smooth half-sine.
    if phase < 0.5:
      return self.amplitude * math.sin(phase * 2.0 * math.pi), 0.0
    return 0.0, self.amplitude * math.sin((phase - 0.5) * 2.0 * math.pi)


class JoystickPedalSource(PedalSource):
  """A real USB HID pedal set, read through pygame.joystick.

  pygame needs no driver install on macOS or Linux. Axis indices vary by rig,
  so they are flags -- run with --list-axes to see live values and pick them.
  """
  name = "joystick"

  def __init__(self, throttle_axis: int = 1, brake_axis: int = 2, device_index: int = 0):
    try:
      import pygame
    except ImportError as exc:
      raise RuntimeError(
        "pygame is required for the joystick pedal source. Install it with "
        "`uv pip install pygame`, or use --source synthetic for bench work."
      ) from exc

    self._pygame = pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
      raise RuntimeError("no USB HID joystick/pedal device found")

    self._js = pygame.joystick.Joystick(device_index)
    self._js.init()
    self.throttle_axis = int(throttle_axis)
    self.brake_axis = int(brake_axis)
    self.device_name = self._js.get_name()
    self.num_axes = self._js.get_numaxes()
    for axis in (self.throttle_axis, self.brake_axis):
      if axis >= self.num_axes:
        raise RuntimeError(f"axis {axis} out of range: {self.device_name!r} has {self.num_axes} axes")

  def read(self) -> tuple[float, float]:
    self._pygame.event.pump()
    return (self._js.get_axis(self.throttle_axis), self._js.get_axis(self.brake_axis))

  def all_axes(self) -> list[float]:
    self._pygame.event.pump()
    return [self._js.get_axis(i) for i in range(self.num_axes)]

  def close(self) -> None:
    self._pygame.joystick.quit()
    self._pygame.quit()


def identify_axes(source: "JoystickPedalSource", hold_s: float = 3.0,
                  prompt=print, settle_s: float = 1.0) -> dict:
  """Work out which axis is which by watching the subject press each pedal.

  A 3-pedal rig exposes several axes with no documented order, and guessing
  wrong is not obvious from the data afterwards -- a swapped throttle and
  brake still produces a plausible-looking trace. So the mapping is measured
  once, by deviation from rest, and written into devices.toml.

  Returns {"rest": [...], "throttle": idx, "brake": idx, "moved": {...}}.
  """
  n = source.num_axes

  def capture(seconds: float) -> list[float]:
    lo = [9e9] * n
    hi = [-9e9] * n
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
      values = source.all_axes()
      for i, v in enumerate(values):
        lo[i] = min(lo[i], v)
        hi[i] = max(hi[i], v)
      time.sleep(0.01)
    # the extreme furthest from rest is what a held pedal looks like
    return [lo, hi]

  prompt(f"Identifying axes on {source.device_name!r} ({n} axes).")
  prompt(f"  Feet OFF the pedals for {settle_s:.0f} s...")
  lo, hi = capture(settle_s)
  rest = [(lo[i] + hi[i]) / 2.0 for i in range(n)]
  prompt("  rest: " + "  ".join(f"[{i}] {v:+.3f}" for i, v in enumerate(rest)))

  result = {"rest": rest, "moved": {}}
  for name in ("throttle", "brake"):
    prompt(f"  Press and HOLD the {name.upper()} pedal to full travel ({hold_s:.0f} s)...")
    lo, hi = capture(hold_s)
    # deviation from rest, in whichever direction the axis actually travels
    deviation = [max(abs(lo[i] - rest[i]), abs(hi[i] - rest[i])) for i in range(n)]
    best = max(range(n), key=lambda i: deviation[i])
    result[name] = best
    result["moved"][name] = {i: round(deviation[i], 3) for i in range(n) if deviation[i] > 0.05}
    travel = hi[best] if abs(hi[best] - rest[best]) > abs(lo[best] - rest[best]) else lo[best]
    prompt(f"    -> axis {best}  (rest {rest[best]:+.3f} -> {travel:+.3f}, deviation {deviation[best]:.3f})")
    if deviation[best] < 0.1:
      prompt(f"    WARNING: nothing moved more than 0.1. Was the {name} pedal pressed?")

  if result["throttle"] == result["brake"]:
    prompt("  WARNING: throttle and brake resolved to the SAME axis. Re-run and "
           "press only one pedal at a time.")
  return result


def build_source(kind: str, throttle_axis: int = 1, brake_axis: int = 2,
                 period_s: float = 20.0, amplitude: float = 0.6) -> PedalSource:
  if kind == "synthetic":
    return SyntheticPedalSource(period_s=period_s, amplitude=amplitude)
  if kind == "joystick":
    return JoystickPedalSource(throttle_axis=throttle_axis, brake_axis=brake_axis)
  raise ValueError(f"unknown pedal source: {kind}")
