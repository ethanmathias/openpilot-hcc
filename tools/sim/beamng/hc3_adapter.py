import importlib.util
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np


def manual_longitudinal_command(driver_gas: float, driver_brake: float) -> float:
  return float(np.clip(float(driver_gas) - float(driver_brake), -1.0, 1.0))


def split_signed_command(command: float) -> tuple[float, float]:
  return float(np.clip(command, 0.0, 1.0)), float(np.clip(-command, 0.0, 1.0))


def accel_to_pedal_commands(accel_cmd: float, beamng_hccc_mode: bool) -> tuple[float, float]:
  if beamng_hccc_mode:
    return split_signed_command(accel_cmd)
  throttle = float(np.clip(accel_cmd / 1.6, 0.0, 1.0))
  brake = float(np.clip(-accel_cmd / 4.0, 0.0, 1.0))
  return throttle, brake


def _slew_limit(target: float, previous: float, rise_step: float, fall_step: float) -> float:
  if target >= previous:
    return float(min(target, previous + rise_step))
  return float(max(target, previous - fall_step))


def calibrated_hc3_pedal_commands(accel_cmd: float, prev_throttle: float, prev_brake: float) -> tuple[float, float]:
  if abs(accel_cmd) < 0.05:
    accel_cmd = 0.0

  target_throttle = float(np.clip(accel_cmd / 1.6, 0.0, 1.0))
  target_brake = float(np.clip(-accel_cmd, 0.0, 1.0))
  throttle = _slew_limit(target_throttle, prev_throttle, rise_step=0.04, fall_step=0.08)
  brake = _slew_limit(target_brake, prev_brake, rise_step=0.08, fall_step=0.10)
  return throttle, brake


def normalize_hc3_output(output: Any) -> float:
  if isinstance(output, (tuple, list)):
    if len(output) != 2:
      raise ValueError(f"Expected HC3 output of length 2, got {len(output)}")
    return float(output[0]) - float(output[1])
  return float(output)


def blend_manual_and_hc3(driver_gas: float, driver_brake: float, hc3_cmd: float) -> dict[str, float]:
  manual_cmd = manual_longitudinal_command(driver_gas, driver_brake)
  final_cmd = manual_cmd + float(hc3_cmd)
  throttle_out, brake_out = split_signed_command(final_cmd)
  return {
    "manual_cmd": float(manual_cmd),
    "hc3_cmd": float(hc3_cmd),
    "final_cmd": float(final_cmd),
    "throttle_out": float(throttle_out),
    "brake_out": float(brake_out),
  }


@dataclass
class HC3StepResult:
  manual_accel: float
  hccc_accel: float
  final_accel: float
  throttle_out: float
  brake_out: float


class HC3ControllerAdapter:
  def __init__(self, controller: Any):
    self.controller = controller

  def step(self) -> float:
    return normalize_hc3_output(self.controller.run_step())


def _repo_root() -> Path:
  return Path(__file__).resolve().parents[3]


def _load_module_from_path(module_path: Path) -> ModuleType:
  spec = importlib.util.spec_from_file_location(f"beamng_hc3_{module_path.stem}", module_path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"Failed to load HC3 controller module from {module_path}")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def resolve_hc3_controller_path(variant: str, controller_path: str | None = None) -> Path:
  if controller_path is not None:
    candidates = [Path(controller_path).expanduser()]
  else:
    integration_dir = _repo_root() / "hccintegration"
    if variant == "new":
      candidates = [integration_dir / "BEAMNGhCCC_controller_NEW.py"]
    elif variant == "original":
      candidates = [
        integration_dir / "hCCC_controller.py",
        integration_dir / "hCCC_controller_ICE.py",
      ]
    else:
      raise ValueError(f"Unsupported HC3 variant: {variant}")

  for candidate in candidates:
    if candidate.is_file():
      return candidate

  searched = ", ".join(str(path) for path in candidates)
  raise FileNotFoundError(f"Unable to find HC3 controller for variant '{variant}'. Searched: {searched}")


def create_hc3_controller(variant: str, ego_vehicle: Any, lead_vehicle: Any, dt: float = 0.1,
                          controller_path: str | None = None) -> HC3ControllerAdapter:
  module_path = resolve_hc3_controller_path(variant, controller_path)
  module = _load_module_from_path(module_path)
  controller_cls = getattr(module, "hCCC", None)
  if controller_cls is None:
    raise AttributeError(f"HC3 controller module does not define hCCC: {module_path}")
  controller = controller_cls(ego_vehicle, lead_vehicle, dt=dt)
  return HC3ControllerAdapter(controller)


class ReferenceLongitudinalBlender:
  def __init__(self, controller: HC3ControllerAdapter, update_dt: float = 0.1, sim_dt: float = 0.01):
    self.controller = controller
    self._held_hc3_cmd = 0.0
    self._update_steps = max(1, int(round(update_dt / sim_dt)))
    self._counter = 0

  def reset(self):
    self._held_hc3_cmd = 0.0
    self._counter = 0

  def step(self, driver_gas: float, driver_brake: float) -> HC3StepResult:
    if self._counter <= 0:
      self._held_hc3_cmd = float(self.controller.step())
      self._counter = self._update_steps - 1
    else:
      self._counter -= 1

    blended = blend_manual_and_hc3(driver_gas, driver_brake, self._held_hc3_cmd)
    return HC3StepResult(
      manual_accel=float(blended["manual_cmd"]),
      hccc_accel=float(blended["hc3_cmd"]),
      final_accel=float(blended["final_cmd"]),
      throttle_out=float(blended["throttle_out"]),
      brake_out=float(blended["brake_out"]),
    )
