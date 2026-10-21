"""Config tests: where each setting comes from and which source wins.

Params is a compiled extension and this file must run without a built
openpilot, so every test injects a FakeParams. That injection is the reason
config.py imports Params on use rather than at module import.

Precedence is always: environment variable > param > default (off).
"""
import pytest

from openpilot.selfdrive.controls.lib.hcc.config import (
  HCCSettings,
  load_pedal_config,
  load_v2v_config,
  parse_bool_env,
  read_int_param,
  read_str_param,
)
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  DEFAULT_PEDAL_STALE_THRESHOLD_MS,
  DEFAULT_RELAY_HOST,
  DEFAULT_RELAY_PORT,
  DEFAULT_STALE_THRESHOLD_MS,
  ROLE_EGO,
  ROLE_LEAD,
  ROLE_PEDAL,
)


class FakeParams:
  """Mimics the bits of the Params API that config.py uses.

  Keys that were never written return None; keys outside the known set raise,
  the way Params raises UnknownKeyName.
  """

  KNOWN = {"EnableHCCC", "HCCV2VEnabled", "HCCV2VOnly", "HCCV2VRelayHost", "HCCV2VRelayPort",
           "HCCV2VDeviceId", "HCCPedalEnabled", "HCCPedalDeviceId", "HCCPedalStaleMs"}

  def __init__(self, values=None):
    self._values = dict(values or {})

  def get(self, key):
    if key not in self.KNOWN:
      raise KeyError(key)
    return self._values.get(key)

  def get_bool(self, key):
    raw = self.get(key)
    if isinstance(raw, bool):
      return raw
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode()
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


class CP:
  def __init__(self, enableHCCC=None):
    if enableHCCC is not None:
      self.enableHCCC = enableHCCC


# ---- env parsing ---------------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
  ("1", True), ("true", True), ("TRUE", True), ("yes", True), ("on", True), ("  on  ", True),
  ("0", False), ("false", False), ("no", False), ("off", False),
  (None, None), ("", None), ("maybe", None),
])
def test_parse_bool_env(raw, expected):
  assert parse_bool_env(raw) is expected


# ---- typed param reads ---------------------------------------------------

def test_int_typed_param_read_as_string():
  # Field bug: HCCV2VRelayPort is INT-typed, so Params returns an int and the
  # original str() path crashed load_v2v_config.
  params = FakeParams({"HCCV2VRelayPort": 19095})
  assert read_str_param(params, "HCCV2VRelayPort") == "19095"
  assert read_int_param(params, "HCCV2VRelayPort") == 19095


def test_bytes_valued_param_is_decoded():
  params = FakeParams({"HCCV2VRelayHost": b"10.42.0.1"})
  assert read_str_param(params, "HCCV2VRelayHost") == "10.42.0.1"


def test_blank_and_unset_params_read_as_none():
  params = FakeParams({"HCCV2VRelayHost": "   "})
  assert read_str_param(params, "HCCV2VRelayHost") is None
  assert read_str_param(FakeParams(), "HCCV2VRelayHost") is None


def test_unknown_key_does_not_raise():
  assert read_str_param(FakeParams(), "NotARealKey") is None


# ---- feature flags -------------------------------------------------------

def test_hccc_defaults_off_on_the_road():
  settings = HCCSettings(params=FakeParams(), env={}, CP=CP())
  assert settings.hccc_enabled() is False


def test_hccc_param_enables_it():
  settings = HCCSettings(params=FakeParams({"EnableHCCC": True}), env={}, CP=CP())
  assert settings.hccc_enabled() is True


def test_carparams_flag_outranks_the_param():
  # card.py may stamp the decision into CarParams; that wins so the whole
  # process agrees for the life of the drive.
  settings = HCCSettings(params=FakeParams({"EnableHCCC": True}), env={}, CP=CP(enableHCCC=False))
  assert settings.hccc_enabled() is False


def test_env_overrides_the_param_for_the_flags():
  params = FakeParams({"EnableHCCC": False, "HCCV2VOnly": False, "HCCPedalEnabled": False})
  settings = HCCSettings(params=params,
                         env={"HCC_ENABLE_HCCC": "1", "HCC_V2V_ONLY": "1", "HCC_PEDAL_ENABLED": "1"},
                         CP=CP())
  assert settings.hccc_enabled() is True
  assert settings.v2v_only() is True
  assert settings.pedal_enabled() is True


def test_flags_default_off_and_follow_their_params():
  assert HCCSettings(params=FakeParams(), env={}, CP=CP()).v2v_only() is False
  assert HCCSettings(params=FakeParams(), env={}, CP=CP()).pedal_enabled() is False
  assert HCCSettings(params=FakeParams({"HCCV2VOnly": True}), env={}, CP=CP()).v2v_only() is True
  assert HCCSettings(params=FakeParams({"HCCPedalEnabled": True}), env={}, CP=CP()).pedal_enabled() is True


def test_unparseable_env_falls_through_to_the_param():
  settings = HCCSettings(params=FakeParams({"HCCV2VOnly": True}), env={"HCC_V2V_ONLY": "banana"}, CP=CP())
  assert settings.v2v_only() is True


# ---- lead config ---------------------------------------------------------

def test_lead_config_defaults():
  config = load_v2v_config(ROLE_EGO, params=FakeParams(), env={})
  assert config.role == ROLE_EGO
  assert config.enabled is False
  assert config.relay_host == DEFAULT_RELAY_HOST
  assert config.relay_port == DEFAULT_RELAY_PORT
  assert config.device_id == "hcc-ego"
  assert config.stale_threshold_ms == DEFAULT_STALE_THRESHOLD_MS


def test_lead_config_reads_params():
  params = FakeParams({"HCCV2VEnabled": True, "HCCV2VRelayHost": "10.42.0.1",
                       "HCCV2VRelayPort": 19095, "HCCV2VDeviceId": "ego-7"})
  config = load_v2v_config(ROLE_EGO, params=params, env={})
  assert config.enabled is True
  assert config.relay_host == "10.42.0.1"
  assert config.relay_port == 19095
  assert config.device_id == "ego-7"


def test_lead_config_env_wins_over_params():
  params = FakeParams({"HCCV2VEnabled": False, "HCCV2VRelayHost": "10.42.0.1",
                       "HCCV2VRelayPort": 19095, "HCCV2VDeviceId": "ego-7"})
  env = {"HCC_V2V_ENABLED": "1", "HCC_V2V_RELAY_HOST": "127.0.0.1",
         "HCC_V2V_RELAY_PORT": "20000", "HCC_V2V_DEVICE_ID": "bench-ego",
         "HCC_V2V_STALE_THRESHOLD_MS": "120"}
  config = load_v2v_config(ROLE_EGO, params=params, env=env)
  assert config.enabled is True
  assert config.relay_host == "127.0.0.1"
  assert config.relay_port == 20000
  assert config.device_id == "bench-ego"
  assert config.stale_threshold_ms == 120.0


def test_lead_config_role_defaults_its_device_id():
  assert load_v2v_config(ROLE_LEAD, params=FakeParams(), env={}).device_id == "hcc-lead"


def test_lead_config_rejects_the_pedal_role():
  # The pedal stream has its own loader; passing ROLE_PEDAL here is a mistake.
  with pytest.raises(ValueError):
    load_v2v_config(ROLE_PEDAL, params=FakeParams(), env={})


def test_sender_address_defaults_to_the_relay():
  config = load_v2v_config(ROLE_EGO, params=FakeParams({"HCCV2VRelayHost": "10.42.0.1",
                                                        "HCCV2VRelayPort": 19095}), env={})
  assert config.expected_sender_host == "10.42.0.1"
  assert config.expected_sender_port == 19095


# ---- pedal config --------------------------------------------------------

def test_pedal_config_defaults():
  config = load_pedal_config(params=FakeParams(), env={})
  assert config.role == ROLE_PEDAL
  assert config.enabled is False
  assert config.device_id == "hcc-pedals"
  assert config.stale_threshold_ms == DEFAULT_PEDAL_STALE_THRESHOLD_MS


def test_pedal_config_shares_the_lead_network_settings():
  # The rig is a third peer on the same relay and the same hotspot, so there
  # must only ever be one relay host/port to configure.
  params = FakeParams({"HCCV2VRelayHost": "10.42.0.1", "HCCV2VRelayPort": 19095,
                       "HCCPedalEnabled": True})
  config = load_pedal_config(params=params, env={})
  assert config.enabled is True
  assert config.relay_host == "10.42.0.1"
  assert config.relay_port == 19095


def test_pedal_config_reads_its_own_device_id_and_staleness():
  params = FakeParams({"HCCPedalEnabled": True, "HCCPedalDeviceId": "srp2-rig",
                       "HCCPedalStaleMs": 90})
  config = load_pedal_config(params=params, env={})
  assert config.device_id == "srp2-rig"
  assert config.stale_threshold_ms == 90.0


def test_pedal_config_env_wins():
  params = FakeParams({"HCCPedalEnabled": False, "HCCPedalDeviceId": "srp2-rig"})
  env = {"HCC_PEDAL_ENABLED": "1", "HCC_PEDAL_DEVICE_ID": "bench-rig", "HCC_PEDAL_STALE_MS": "60"}
  config = load_pedal_config(params=params, env=env)
  assert config.enabled is True
  assert config.device_id == "bench-rig"
  assert config.stale_threshold_ms == 60.0


def test_pedal_staleness_is_stricter_than_the_lead_by_default():
  pedal = load_pedal_config(params=FakeParams(), env={})
  lead = load_v2v_config(ROLE_EGO, params=FakeParams(), env={})
  assert pedal.stale_threshold_ms < lead.stale_threshold_ms
