"""Every param and environment variable the HCC experiment reads.

On hcc-ego this logic was spread across longcontrol.py, hcc_v2v.py and
car_specific.py, each with its own getattr/check_key/env fallback. It lives
here instead, so "what turns this on and where does it come from" has one
answer.

Precedence is always: environment variable (bench and sim overrides) > param
(what the device is configured with) > default (off).
"""
from __future__ import annotations

import os
from typing import TYPE_CHECKING, Any

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  DEFAULT_PEDAL_STALE_THRESHOLD_MS,
  DEFAULT_RELAY_HOST,
  DEFAULT_RELAY_PORT,
  DEFAULT_SEND_HZ,
  DEFAULT_SENDER_ADDRESS_POLICY,
  DEFAULT_STALE_THRESHOLD_MS,
  MAX_TIMESTAMP_SKEW_MS,
  ROLE_EGO,
  ROLE_LEAD,
  ROLE_PEDAL,
  V2VConfig,
)

if TYPE_CHECKING:
  from openpilot.common.params import Params

SIMULATION = os.environ.get("SIMULATION", "0") == "1"


def default_params() -> Params:
  """Params is a compiled extension, so it is imported on use rather than at
  module import. That keeps this module (and everything that reads config from
  an injected params object) importable and unit-testable without a built
  openpilot."""
  from openpilot.common.params import Params
  return Params()


def parse_bool_env(value: str | None) -> bool | None:
  if value is None:
    return None
  normalized = value.strip().lower()
  if normalized in ("1", "true", "yes", "on"):
    return True
  if normalized in ("0", "false", "no", "off"):
    return False
  return None


def _get_optional_param(params: Params, key: str) -> Any | None:
  try:
    return params.get(key)
  except Exception:
    return None


def read_bool_param(params: Params, key: str) -> bool | None:
  if _get_optional_param(params, key) is None:
    return None
  return params.get_bool(key)


def read_str_param(params: Params, key: str) -> str | None:
  raw_value = _get_optional_param(params, key)
  if raw_value is None:
    return None
  # Typed params can come back as int (e.g. HCCV2VRelayPort) or bytes.
  if isinstance(raw_value, (bytes, bytearray)):
    raw_value = raw_value.decode()
  stripped = str(raw_value).strip()
  return stripped if len(stripped) > 0 else None


def read_int_param(params: Params, key: str) -> int | None:
  raw_value = read_str_param(params, key)
  return None if raw_value is None else int(raw_value)


def _flag(params: Params, env_map, param_key: str, env_key: str, default: bool) -> bool:
  env_value = parse_bool_env(env_map.get(env_key))
  if env_value is not None:
    return env_value
  param_value = read_bool_param(params, param_key)
  return default if param_value is None else param_value


class HCCSettings:
  """The experiment's feature flags, read once per controller reset.

  Reads are cheap but not free and controlsd runs at 100 Hz, so callers hold
  one of these and refresh it rather than hitting Params per tick.
  """

  def __init__(self, params: Params | None = None, env: dict[str, str] | None = None, CP=None):
    self._params = default_params() if params is None else params
    self._env = os.environ if env is None else env
    self._CP = CP

  def hccc_enabled(self) -> bool:
    """Master switch for the hCCC longitudinal controller."""
    cp_flag = getattr(self._CP, "enableHCCC", None)
    if cp_flag is not None:
      return bool(cp_flag)
    env_value = parse_bool_env(self._env.get("HCC_ENABLE_HCCC"))
    if env_value is not None:
      return env_value
    param_value = read_bool_param(self._params, "EnableHCCC")
    # In simulation the toggle is implied; on the road it must be set explicitly.
    return SIMULATION if param_value is None else param_value

  def v2v_only(self) -> bool:
    """Take the lead exclusively from V2V, never from radar/vision."""
    return _flag(self._params, self._env, "HCCV2VOnly", "HCC_V2V_ONLY", False)

  def pedal_enabled(self) -> bool:
    """Source the human longitudinal input from the pedal rig over V2V."""
    return _flag(self._params, self._env, "HCCPedalEnabled", "HCC_PEDAL_ENABLED", False)


def load_v2v_config(role: str,
                    params: Params | None = None,
                    env: dict[str, str] | None = None,
                    default_enabled: bool = False,
                    default_device_id: str | None = None) -> V2VConfig:
  """Config for the lead stream (the ego subscribes, the lead publishes)."""
  if role not in (ROLE_LEAD, ROLE_EGO):
    raise ValueError(f"unsupported role: {role}")

  params_reader = default_params() if params is None else params
  env_map = os.environ if env is None else env
  fallback_device_id = default_device_id or f"hcc-{role}"

  enabled = read_bool_param(params_reader, "HCCV2VEnabled")
  env_enabled = parse_bool_env(env_map.get("HCC_V2V_ENABLED"))
  if env_enabled is not None:
    enabled = env_enabled
  if enabled is None:
    enabled = default_enabled

  relay_host = env_map.get("HCC_V2V_RELAY_HOST") or read_str_param(params_reader, "HCCV2VRelayHost") or DEFAULT_RELAY_HOST
  relay_port = int(env_map.get("HCC_V2V_RELAY_PORT") or read_int_param(params_reader, "HCCV2VRelayPort") or DEFAULT_RELAY_PORT)
  device_id = env_map.get("HCC_V2V_DEVICE_ID") or read_str_param(params_reader, "HCCV2VDeviceId") or fallback_device_id

  return V2VConfig(
    role=role,
    enabled=bool(enabled),
    relay_host=str(relay_host),
    relay_port=int(relay_port),
    device_id=str(device_id),
    send_hz=float(env_map.get("HCC_V2V_SEND_HZ") or DEFAULT_SEND_HZ),
    stale_threshold_ms=float(env_map.get("HCC_V2V_STALE_THRESHOLD_MS") or DEFAULT_STALE_THRESHOLD_MS),
    max_clock_skew_ms=float(env_map.get("HCC_V2V_MAX_CLOCK_SKEW_MS") or MAX_TIMESTAMP_SKEW_MS),
    sender_address_policy=str(env_map.get("HCC_V2V_SENDER_POLICY") or DEFAULT_SENDER_ADDRESS_POLICY),
    expected_sender_host=str(env_map.get("HCC_V2V_EXPECTED_SENDER_HOST") or relay_host),
    expected_sender_port=int(env_map.get("HCC_V2V_EXPECTED_SENDER_PORT") or relay_port),
  )


def load_pedal_config(params: Params | None = None,
                      env: dict[str, str] | None = None,
                      default_enabled: bool = False) -> V2VConfig:
  """Config for the pedal stream.

  Network settings are deliberately shared with the lead stream -- the pedal
  laptop is a third peer on the same relay and the same hotspot, so there is
  only ever one relay host/port to configure. Only the device id and the
  staleness threshold are pedal-specific.
  """
  params_reader = default_params() if params is None else params
  env_map = os.environ if env is None else env

  lead_side = load_v2v_config(ROLE_EGO, params_reader, env_map, default_enabled=True)

  enabled = parse_bool_env(env_map.get("HCC_PEDAL_ENABLED"))
  if enabled is None:
    param_value = read_bool_param(params_reader, "HCCPedalEnabled")
    enabled = default_enabled if param_value is None else param_value

  device_id = (env_map.get("HCC_PEDAL_DEVICE_ID")
               or read_str_param(params_reader, "HCCPedalDeviceId")
               or "hcc-pedals")
  stale_ms = (env_map.get("HCC_PEDAL_STALE_MS")
              or read_int_param(params_reader, "HCCPedalStaleMs")
              or DEFAULT_PEDAL_STALE_THRESHOLD_MS)

  return V2VConfig(
    role=ROLE_PEDAL,
    enabled=bool(enabled),
    relay_host=lead_side.relay_host,
    relay_port=lead_side.relay_port,
    device_id=str(device_id),
    send_hz=lead_side.send_hz,
    stale_threshold_ms=float(stale_ms),
    max_clock_skew_ms=lead_side.max_clock_skew_ms,
    sender_address_policy=lead_side.sender_address_policy,
    expected_sender_host=lead_side.expected_sender_host,
    expected_sender_port=lead_side.expected_sender_port,
  )
