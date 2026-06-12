from __future__ import annotations

import os

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.vendor.hcc_v2v_core import (
  DEFAULT_RELAY_HOST,
  DEFAULT_RELAY_PORT,
  DEFAULT_SEND_HZ,
  DEFAULT_SENDER_ADDRESS_POLICY,
  DEFAULT_STALE_THRESHOLD_MS,
  HELLO_INTERVAL_S,
  MAX_TIMESTAMP_SKEW_MS,
  RECV_BUFFER_BYTES,
  ROLE_EGO,
  ROLE_LEAD,
  TYPE_DATA,
  TYPE_HELLO,
  V2VConfig,
  V2VHelloPacket,
  V2VLeadBuffer,
  V2VLeadPacket,
  V2VLeadSignal,
  V2VLeadSubscriber,
  V2VPublisher,
  encode_data_packet,
  encode_hello_packet,
  is_timestamp_plausible,
  parse_data_packet,
  parse_hello_packet,
)


def _parse_bool_env(value: str | None) -> bool | None:
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


def _read_bool_param(params: Params, key: str) -> bool | None:
  raw_value = _get_optional_param(params, key)
  if raw_value is None:
    return None
  return params.get_bool(key)


def _read_str_param(params: Params, key: str) -> str | None:
  raw_value = _get_optional_param(params, key)
  if raw_value is None:
    return None
  # Typed params can come back as int (e.g. HCCV2VRelayPort) or bytes.
  if isinstance(raw_value, (bytes, bytearray)):
    raw_value = raw_value.decode()
  stripped = str(raw_value).strip()
  return stripped if len(stripped) > 0 else None


def _read_int_param(params: Params, key: str) -> int | None:
  raw_value = _read_str_param(params, key)
  if raw_value is None:
    return None
  return int(raw_value)


def load_v2v_config(role: str,
                    params: Params | None = None,
                    env: dict[str, str] | None = None,
                    default_enabled: bool = False,
                    default_device_id: str | None = None) -> V2VConfig:
  if role not in (ROLE_LEAD, ROLE_EGO):
    raise ValueError(f"unsupported role: {role}")

  params_reader = Params() if params is None else params
  env_map = os.environ if env is None else env
  fallback_device_id = default_device_id or f"hcc-{role}"

  enabled = _read_bool_param(params_reader, "HCCV2VEnabled")
  env_enabled = _parse_bool_env(env_map.get("HCC_V2V_ENABLED"))
  if env_enabled is not None:
    enabled = env_enabled
  if enabled is None:
    enabled = default_enabled

  relay_host = env_map.get("HCC_V2V_RELAY_HOST") or _read_str_param(params_reader, "HCCV2VRelayHost") or DEFAULT_RELAY_HOST
  relay_port = int(env_map.get("HCC_V2V_RELAY_PORT") or _read_int_param(params_reader, "HCCV2VRelayPort") or DEFAULT_RELAY_PORT)
  device_id = env_map.get("HCC_V2V_DEVICE_ID") or _read_str_param(params_reader, "HCCV2VDeviceId") or fallback_device_id
  sender_address_policy = env_map.get("HCC_V2V_SENDER_POLICY") or DEFAULT_SENDER_ADDRESS_POLICY
  expected_sender_host = env_map.get("HCC_V2V_EXPECTED_SENDER_HOST") or relay_host
  expected_sender_port = int(env_map.get("HCC_V2V_EXPECTED_SENDER_PORT") or relay_port)
  send_hz = float(env_map.get("HCC_V2V_SEND_HZ") or DEFAULT_SEND_HZ)
  stale_threshold_ms = float(env_map.get("HCC_V2V_STALE_THRESHOLD_MS") or DEFAULT_STALE_THRESHOLD_MS)
  max_clock_skew_ms = float(env_map.get("HCC_V2V_MAX_CLOCK_SKEW_MS") or MAX_TIMESTAMP_SKEW_MS)

  return V2VConfig(
    role=role,
    enabled=bool(enabled),
    relay_host=str(relay_host),
    relay_port=int(relay_port),
    device_id=str(device_id),
    send_hz=send_hz,
    stale_threshold_ms=stale_threshold_ms,
    max_clock_skew_ms=max_clock_skew_ms,
    sender_address_policy=str(sender_address_policy),
    expected_sender_host=str(expected_sender_host),
    expected_sender_port=int(expected_sender_port),
  )

