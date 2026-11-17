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


def _snapshot(source, seconds: float, poll_s: float = 0.02) -> list[float]:
  """Mean of each axis over a short window."""
  totals = [0.0] * source.num_axes
  count = 0
  deadline = time.monotonic() + seconds
  while time.monotonic() < deadline:
    for i, v in enumerate(source.all_axes()):
      totals[i] += v
    count += 1
    time.sleep(poll_s)
  count = max(count, 1)
  return [total / count for total in totals]


def _wait_for_rest(source, rest: list[float], tolerance: float, timeout_s: float,
                   poll_s: float = 0.02) -> bool:
  """Block until every axis is back near rest. True if it got there."""
  deadline = time.monotonic() + timeout_s
  while time.monotonic() < deadline:
    values = source.all_axes()
    if all(abs(v - rest[i]) <= tolerance for i, v in enumerate(values)):
      return True
    time.sleep(poll_s)
  return False


def _wait_for_press(source, rest: list[float], min_deviation: float, stable_s: float,
                    timeout_s: float, poll_s: float = 0.02):
  """Block until some axis is deviating from rest and holding steady.

  Waiting for the pedal to settle is what makes this reliable: a timed window
  catches the previous pedal on its way back up, which is exactly how the
  first version mis-resolved throttle and brake onto the same axis.

  Returns (axis, held_values) or (None, held_values) on timeout.
  """
  deadline = time.monotonic() + timeout_s
  steady_since = None
  steady_values: list[float] | None = None

  while time.monotonic() < deadline:
    values = source.all_axes()
    deviations = [abs(v - rest[i]) for i, v in enumerate(values)]
    peak = max(deviations)

    if peak >= min_deviation:
      moved = all(abs(values[i] - steady_values[i]) < 0.05 for i in range(len(values))) \
          if steady_values is not None else False
      if steady_values is not None and moved:
        if time.monotonic() - steady_since >= stable_s:
          return deviations.index(peak), values
      else:
        steady_values = list(values)
        steady_since = time.monotonic()
    else:
      steady_values = None
      steady_since = None
    time.sleep(poll_s)

  return None, (steady_values or source.all_axes())


def identify_axes(source, prompt=print, min_deviation: float = 0.3,
                  stable_s: float = 0.4, rest_tolerance: float = 0.15,
                  timeout_s: float = 30.0, settle_s: float = 0.8) -> dict:
  """Work out which axis is throttle and which is brake.

  A 3-pedal rig exposes several axes in no documented order, and guessing
  wrong is not visible in the data afterwards -- a swapped throttle and brake
  still produces a plausible trace, and the run is only found to be useless
  when the acceleration goes the wrong way.

  Each step waits for the rig to physically reach the expected state rather
  than sampling a fixed window, so releasing a pedal slowly cannot bleed into
  the next measurement.

  Returns {"rest": [...], "throttle": idx, "brake": idx, "held": {...},
           "deviations": {...}}.
  """
  n = source.num_axes
  prompt(f"Identifying axes on {source.device_name!r} ({n} axes).")
  prompt("  Feet OFF the pedals...")
  rest = _snapshot(source, settle_s)
  prompt("  rest: " + "  ".join(f"[{i}] {v:+.3f}" for i, v in enumerate(rest)))

  result: dict = {"rest": rest, "held": {}, "deviations": {}}
  for name in ("throttle", "brake"):
    if not _wait_for_rest(source, rest, rest_tolerance, timeout_s):
      prompt(f"  WARNING: pedals did not return to rest before the {name} step.")
    prompt(f"  Press and HOLD the {name.upper()} pedal, firmly, until it reads...")

    axis, values = _wait_for_press(source, rest, min_deviation, stable_s, timeout_s)
    deviations = {i: round(abs(values[i] - rest[i]), 3) for i in range(n)}
    result["deviations"][name] = deviations

    if axis is None:
      prompt(f"    nothing held steady within {timeout_s:.0f} s — was the {name} pressed?")
      result[name] = None
      continue

    result[name] = axis
    result["held"][name] = round(values[axis], 3)
    others = {i: d for i, d in deviations.items() if i != axis and d >= min_deviation}
    prompt(f"    -> axis {axis}  (rest {rest[axis]:+.3f} -> held {values[axis]:+.3f}, "
           f"deviation {deviations[axis]:.3f})")
    if others:
      prompt(f"       also moved: {others} — a 3-pedal rig can register the clutch")

  if result.get("throttle") is not None and result["throttle"] == result.get("brake"):
    prompt("  WARNING: throttle and brake resolved to the SAME axis. Release fully "
           "between steps and press only one pedal at a time.")
  return result


def build_source(kind: str, throttle_axis: int = 1, brake_axis: int = 2,
                 period_s: float = 20.0, amplitude: float = 0.6) -> PedalSource:
  if kind == "synthetic":
    return SyntheticPedalSource(period_s=period_s, amplitude=amplitude)
  if kind == "joystick":
    return JoystickPedalSource(throttle_axis=throttle_axis, brake_axis=brake_axis)
  raise ValueError(f"unknown pedal source: {kind}")
