#!/usr/bin/env python3
import argparse
import threading
import time
from dataclasses import dataclass
from typing import NoReturn

from openpilot.tools.sim.bridge.common import control_cmd_gen

try:
  import evdev
  from evdev import ecodes
except ImportError as e:
  raise RuntimeError("logitech_wheel_ctrl.py requires python-evdev") from e


def _clamp(v: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, v))


def _name_score(name: str) -> int:
  n = name.lower()
  score = 0
  for marker in ("logitech", "g29", "g920", "g923", "driving force", "wheel"):
    if marker in n:
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

  candidates.sort(key=lambda d: _name_score(d.name), reverse=True)
  if _name_score(candidates[0].name) > 0:
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


def logitech_wheel_poll_thread(q, device_path: str | None = None, publish_hz: float = 25.0) -> NoReturn:
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

  # Optional force-feedback autcenter if supported.
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
          state.set_steer((ev.value - steer_center) / steer_span)
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


def main():
  parser = argparse.ArgumentParser(description="Logitech wheel/pedal input for openpilot sim bridge.")
  parser.add_argument("--device", default=None, help="Optional /dev/input/eventX path")
  parser.add_argument("--hz", type=float, default=25.0, help="Control publish rate")
  args = parser.parse_args()

  from multiprocessing import Queue
  q = Queue()
  logitech_wheel_poll_thread(q, device_path=args.device, publish_hz=args.hz)


if __name__ == "__main__":
  main()
