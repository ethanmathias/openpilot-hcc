#!/usr/bin/env python3
import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import NoReturn

from openpilot.tools.sim.bridge.common import control_cmd_gen

# evdev is Linux-only. On macOS we fall back to pygame.joystick — see
# logitech_wheel_poll_thread() for dispatch.
if sys.platform.startswith("linux"):
  try:
    import evdev
    from evdev import ecodes
  except ImportError as e:
    raise RuntimeError("logitech_wheel_ctrl.py on Linux requires python-evdev") from e
else:
  evdev = None  # type: ignore
  ecodes = None  # type: ignore

DEFAULT_PUBLISH_HZ = 100.0
STEER_DEADZONE = 0.015
ENABLE_AUTOCENTER = False


def _clamp(v: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, v))

def _apply_deadzone(v: float, deadzone: float) -> float:
  if abs(v) <= deadzone:
    return 0.0
  return math.copysign((abs(v) - deadzone) / max(1.0 - deadzone, 1e-3), v)


def _name_score(name: str) -> int:
  n = name.lower()
  score = 0
  for marker in ("logitech", "g29", "g920", "g923", "driving force", "wheel"):
    if marker in n:
      score += 1
  return score


def _abs_codes(dev):
  try:
    return {code for code, _ in dev.capabilities(absinfo=True).get(ecodes.EV_ABS, [])}
  except Exception:
    return set()


def _device_score(dev) -> int:
  abs_codes = _abs_codes(dev)
  score = _name_score(dev.name) * 10

  if ecodes.ABS_X in abs_codes or ecodes.ABS_RX in abs_codes or ecodes.ABS_WHEEL in abs_codes:
    score += 8
  if ecodes.ABS_GAS in abs_codes or ecodes.ABS_Z in abs_codes:
    score += 5
  if ecodes.ABS_BRAKE in abs_codes or ecodes.ABS_RZ in abs_codes:
    score += 5
  if ecodes.EV_KEY in dev.capabilities():
    score += 1
  return score


def _find_logitech_device(device_path: str | None):
  try:
    if device_path is not None:
      dev = evdev.InputDevice(device_path)
      if ecodes.EV_ABS not in dev.capabilities():
        raise RuntimeError(f"{device_path} has no EV_ABS capabilities")
      return dev
  except PermissionError as e:
    raise RuntimeError(f"Permission denied opening {device_path}. Add your user to the input group or run with sudo.") from e

  devices = []
  for path in evdev.list_devices():
    try:
      dev = evdev.InputDevice(path)
      devices.append(dev)
    except PermissionError:
      continue

  candidates = [dev for dev in devices if ecodes.EV_ABS in dev.capabilities()]
  if not candidates:
    available = ", ".join(f"{d.path}:{d.name}" for d in devices) or "<none>"
    raise RuntimeError(f"No EV_ABS input device found. Available devices: {available}")

  candidates.sort(key=_device_score, reverse=True)
  if _device_score(candidates[0]) > 0:
    return candidates[0]
  if len(candidates) == 1:
    return candidates[0]

  available = ", ".join(f"{d.path}:{d.name}" for d in devices) or "<none>"
  raise RuntimeError(
    f"Could not identify Logitech wheel automatically. Pass --wheel_device /dev/input/eventX. Available devices: {available}"
  )


def _abs_map(dev):
  abs_caps = dev.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
  return {code: info for code, info in abs_caps}


def _pick_axis(abs_info, candidates):
  for code in candidates:
    if code in abs_info:
      return code
  return None


def _pick_steer_axis(abs_info):
  candidates = [c for c in (ecodes.ABS_X, ecodes.ABS_RX, ecodes.ABS_WHEEL) if c in abs_info]
  if not candidates:
    return None
  return max(candidates, key=lambda c: abs_info[c].max - abs_info[c].min)


def _pick_pedal_axes(abs_info):
  # Prefer explicit gas/brake axes, then the common Z/RZ pair.
  throttle_candidates = [ecodes.ABS_GAS, ecodes.ABS_Z, ecodes.ABS_RY, ecodes.ABS_Y]
  brake_candidates = [ecodes.ABS_BRAKE, ecodes.ABS_RZ, ecodes.ABS_Y, ecodes.ABS_RY]

  throttle_axis = _pick_axis(abs_info, throttle_candidates)
  brake_axis = _pick_axis(abs_info, brake_candidates)

  if throttle_axis is not None and brake_axis is not None and throttle_axis == brake_axis:
    for c in brake_candidates:
      if c in abs_info and c != throttle_axis:
        brake_axis = c
        break
    else:
      brake_axis = None

  return throttle_axis, brake_axis


@dataclass
class PedalNorm:
  minimum: int
  maximum: int
  rest: int
  inverted: bool

  @classmethod
  def from_absinfo(cls, info):
    center = (info.min + info.max) / 2
    return cls(info.min, info.max, info.value, info.value > center)

  def normalize(self, raw_value: int) -> float:
    if self.inverted:
      denom = max(self.rest - self.minimum, 1)
      out = (self.rest - raw_value) / denom
    else:
      denom = max(self.maximum - self.rest, 1)
      out = (raw_value - self.rest) / denom
    return _clamp(float(out), 0.0, 1.0)


class LogitechWheelState:
  def __init__(self):
    self.steer = 0.0
    self.throttle = 0.0
    self.brake = 0.0
    self.lock = threading.Lock()

  def set_steer(self, value: float):
    with self.lock:
      self.steer = _clamp(value, -1.0, 1.0)

  def set_throttle(self, value: float):
    with self.lock:
      self.throttle = _clamp(value, 0.0, 1.0)

  def set_brake(self, value: float):
    with self.lock:
      self.brake = _clamp(value, 0.0, 1.0)

  def snapshot(self):
    with self.lock:
      return self.steer, self.throttle, self.brake


def _publisher_loop(q, state: LogitechWheelState, hz: float, stop_event: threading.Event):
  dt = 1.0 / max(hz, 1.0)
  while not stop_event.is_set():
    steer, throttle, brake = state.snapshot()
    q.put(control_cmd_gen(f"steer_{steer:.5f}"))
    q.put(control_cmd_gen(f"throttle_{throttle:.5f}"))
    q.put(control_cmd_gen(f"brake_{brake:.5f}"))
    time.sleep(dt)


def logitech_wheel_poll_thread(q, device_path: str | None = None, publish_hz: float = DEFAULT_PUBLISH_HZ) -> NoReturn:
  if not sys.platform.startswith("linux"):
    return _pygame_wheel_poll_thread(q, publish_hz=publish_hz)
  return _evdev_wheel_poll_thread(q, device_path=device_path, publish_hz=publish_hz)


def _evdev_wheel_poll_thread(q, device_path: str | None = None, publish_hz: float = DEFAULT_PUBLISH_HZ) -> NoReturn:
  dev = _find_logitech_device(device_path)
  abs_info = _abs_map(dev)

  steer_axis = _pick_steer_axis(abs_info)
  throttle_axis, brake_axis = _pick_pedal_axes(abs_info)

  if steer_axis is None:
    raise RuntimeError(f"No steering axis found on {dev.path} ({dev.name})")
  if throttle_axis is None or brake_axis is None:
    raise RuntimeError(f"No throttle/brake axes found on {dev.path} ({dev.name})")

  steer_info = abs_info[steer_axis]
  throttle_norm = PedalNorm.from_absinfo(abs_info[throttle_axis])
  brake_norm = PedalNorm.from_absinfo(abs_info[brake_axis])

  print(f"[logitech] device: {dev.path} ({dev.name})")
  print(f"[logitech] axes: steer={steer_axis} throttle={throttle_axis} brake={brake_axis}")

  state = LogitechWheelState()
  stop_event = threading.Event()
  publisher = threading.Thread(target=_publisher_loop, args=(q, state, publish_hz, stop_event), daemon=True)
  publisher.start()

  if ENABLE_AUTOCENTER:
    try:
      dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 24000)
    except Exception:
      pass

  button_map = {
    ecodes.BTN_SOUTH: "cruise_down",
    ecodes.BTN_NORTH: "cruise_up",
    ecodes.BTN_EAST: "cruise_cancel",
    ecodes.BTN_START: "reset",
  }

  steer_center = (steer_info.min + steer_info.max) * 0.5
  steer_span = max((steer_info.max - steer_info.min) * 0.5, 1.0)

  try:
    for ev in dev.read_loop():
      if ev.type == ecodes.EV_ABS:
        if ev.code == steer_axis:
          steer = (ev.value - steer_center) / steer_span
          steer = _apply_deadzone(float(steer), STEER_DEADZONE)
          state.set_steer(steer)
        elif ev.code == throttle_axis:
          state.set_throttle(throttle_norm.normalize(ev.value))
        elif ev.code == brake_axis:
          state.set_brake(brake_norm.normalize(ev.value))

      elif ev.type == ecodes.EV_KEY and ev.value == 1:
        cmd = button_map.get(ev.code)
        if cmd is not None:
          q.put(control_cmd_gen(cmd))
  except PermissionError as e:
    raise RuntimeError(f"Permission denied reading {dev.path}. Add your user to the input group or run with sudo.") from e
  finally:
    stop_event.set()


def _pygame_wheel_poll_thread(q, publish_hz: float = DEFAULT_PUBLISH_HZ) -> NoReturn:
  """macOS / non-Linux fallback using pygame.joystick.

  Axis convention: pygame returns steer in [-1, 1] for axis 0; pedals in
  [-1, 1] for the Z/RZ pair (resting at -1, fully pressed at +1). We
  remap pedals to [0, 1] before publishing.
  """
  try:
    import pygame
  except ImportError as e:
    raise RuntimeError("pygame is required for wheel input on non-Linux hosts") from e

  pygame.init()
  pygame.joystick.init()
  if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick/wheel detected. Plug in the device and try again.")

  js = pygame.joystick.Joystick(0)
  js.init()
  print(f"[pygame-wheel] device: {js.get_name()} axes={js.get_numaxes()} buttons={js.get_numbuttons()}")

  state = LogitechWheelState()
  stop_event = threading.Event()
  publisher = threading.Thread(target=_publisher_loop, args=(q, state, publish_hz, stop_event), daemon=True)
  publisher.start()

  # Logitech G29/G920 on macOS: axis 0 = steer, axis 1 = throttle, axis 2 = brake.
  # Other wheels may differ; expose env vars for overrides.
  import os
  steer_idx = int(os.getenv("WHEEL_STEER_AXIS", "0"))
  throttle_idx = int(os.getenv("WHEEL_THROTTLE_AXIS", "1"))
  brake_idx = int(os.getenv("WHEEL_BRAKE_AXIS", "2"))

  def _pedal_norm(v: float) -> float:
    # pygame pedals rest at +1 and press to -1; flip and rescale to [0, 1].
    return _clamp((1.0 - v) * 0.5, 0.0, 1.0)

  poll_dt = 1.0 / 200.0
  try:
    while True:
      pygame.event.pump()
      steer_raw = js.get_axis(steer_idx) if js.get_numaxes() > steer_idx else 0.0
      throttle_raw = js.get_axis(throttle_idx) if js.get_numaxes() > throttle_idx else 1.0
      brake_raw = js.get_axis(brake_idx) if js.get_numaxes() > brake_idx else 1.0
      state.set_steer(_apply_deadzone(float(steer_raw), STEER_DEADZONE))
      state.set_throttle(_pedal_norm(float(throttle_raw)))
      state.set_brake(_pedal_norm(float(brake_raw)))
      time.sleep(poll_dt)
  finally:
    stop_event.set()


def main():
  parser = argparse.ArgumentParser(description="Logitech wheel/pedal input for openpilot sim bridge.")
  parser.add_argument("--device", default=None, help="Optional /dev/input/eventX path")
  parser.add_argument("--hz", type=float, default=DEFAULT_PUBLISH_HZ, help="Control publish rate")
  args = parser.parse_args()

  from multiprocessing import Queue
  q = Queue()
  logitech_wheel_poll_thread(q, device_path=args.device, publish_hz=args.hz)


if __name__ == "__main__":
  main()
