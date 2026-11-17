#!/usr/bin/env bash
# One-shot V2V setup for the EGO comma 3X.
#
#   sudo tools/hcc/device/setup_network.sh [<SSID> [<PSK> [<RELAY_HOST>]]]
#
# The ego is a CLIENT now, not an access point. Since 2026-11-18 the hotspot
# is served by the Linux VM in LinkLab/wn722n-ap, which drives a USB radio on
# the Mac and also runs the relay. That change means:
#
#   - the ego joins an SSID instead of creating one, so bringing up the
#     network no longer costs it its own route (and no longer kills the SSH
#     session running this script, though the switch is still last for safety)
#   - the relay no longer runs here; HCCV2VRelayHost points at the VM
#   - the ego gets INTERNET, so its clock can track NTP. Packets whose sender
#     timestamp is more than 500 ms from the receiver's are dropped silently,
#     and hand-set clocks drifting apart was the most expensive recurring
#     failure in this project.
#
# The lead is a Raspberry Pi and does not use this script -- see
# device/deploy_lead.sh. The pedal laptop is simply a third peer.
set -euo pipefail

SSID="${1:-hcc-v2v-internet}"
PSK="${2:-}"
RELAY_HOST="${3:-10.42.0.1}"
RELAY_PORT="${RELAY_PORT:-19090}"
CONN_NAME="hcc-v2v-client"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OPENPILOT_DIR="$(readlink -f "$SCRIPT_DIR/../../..")"

set_param() {
  local key="$1"
  local val="$2"
  # openpilot's python deps live in /usr/local/venv, which only an interactive
  # comma shell puts on PATH. Run as the invoking user so params files keep
  # their normal ownership.
  local py=/usr/local/venv/bin/python3
  [ -x "$py" ] || py="$OPENPILOT_DIR/.venv/bin/python3"
  [ -x "$py" ] || py=python3
  sudo -u "${SUDO_USER:-comma}" bash -lc "cd '$OPENPILOT_DIR' && '$py' - '$key' '$val'" <<'PY'
import sys
from openpilot.common.params import Params
key, val = sys.argv[1], sys.argv[2]
p = Params()
# Params are typed (bool/int/string); write with the type the registry expects.
if val in ("0", "1"):
  p.put_bool(key, val == "1")
elif val.lstrip("-").isdigit():
  p.put(key, int(val))
else:
  p.put(key, val)
PY
}

# --- retire the old on-device relay -----------------------------------------
#
# The ego used to run the relay itself. Leaving it enabled would put two
# relays on the network, and both would try to own the ego peer registration.
if systemctl list-unit-files 2>/dev/null | grep -q '^hcc-v2v-relay\.service'; then
  echo "[ego] disabling the old on-device relay (it runs on the VM now)"
  systemctl disable --now hcc-v2v-relay.service 2>/dev/null || true
  # AGNOS keeps the system partition read-only; only remount if the unit file
  # is actually there to remove.
  if [ -f /etc/systemd/system/hcc-v2v-relay.service ]; then
    REMOUNTED=0
    if ! touch /etc/systemd/system/.hcc_rw_test 2>/dev/null; then
      mount -o remount,rw / && REMOUNTED=1
    else
      rm -f /etc/systemd/system/.hcc_rw_test
    fi
    rm -f /etc/systemd/system/hcc-v2v-relay.service
    systemctl daemon-reload
    [ "$REMOUNTED" = "1" ] && { mount -o remount,ro / || echo "[ego] WARN: could not remount / read-only; a reboot will restore it" >&2; }
  fi
fi

# --- params ------------------------------------------------------------------
#
# Set before the network switch: if the WiFi move drops this SSH session, the
# configuration is already complete.
echo "[ego] setting V2V + HC3 params (relay at $RELAY_HOST:$RELAY_PORT)"
set_param HCCV2VEnabled 1
set_param HCCV2VOnly 1
set_param HCCV2VDeviceId hcc-ego
set_param HCCV2VRelayHost "$RELAY_HOST"
set_param HCCV2VRelayPort "$RELAY_PORT"
set_param EnableHCCC 1
set_param AlphaLongitudinalEnabled 1
# The pedal stream is left DISABLED on purpose: while it is on, an unhealthy
# pedal link denies the car positive acceleration. Turn it on once the rig is
# actually publishing.
set_param HCCPedalEnabled 0
set_param HCCPedalDeviceId hcc-pedals
set_param HCCPedalStaleMs 150

# --- join the hotspot --------------------------------------------------------
echo "[ego] joining SSID '$SSID'"
echo "[ego] if you are SSHed in over the old network this session may drop;"
echo "[ego] that is fine, the configuration above is already written."

if ! nmcli -t -f NAME connection show | grep -qx "$CONN_NAME"; then
  if [ -n "$PSK" ]; then
    nmcli connection add type wifi ifname wlan0 con-name "$CONN_NAME" ssid "$SSID" \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" autoconnect yes
  else
    # The VM's hotspot may be open (INET_PASS empty in its config.env).
    nmcli connection add type wifi ifname wlan0 con-name "$CONN_NAME" ssid "$SSID" autoconnect yes
  fi
else
  nmcli connection modify "$CONN_NAME" 802-11-wireless.ssid "$SSID"
  if [ -n "$PSK" ]; then
    nmcli connection modify "$CONN_NAME" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK"
  fi
fi
nmcli connection up "$CONN_NAME"

# --- verify ------------------------------------------------------------------
#
# Best-effort: anything below may not run if the session died with the switch.
sleep 3
ADDR="$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)"
GATEWAY="$(ip -4 route 2>/dev/null | awk '/default/ {print $3; exit}')"
echo "[ego] address $ADDR, gateway $GATEWAY"

if [ -n "$GATEWAY" ] && [ "$GATEWAY" != "$RELAY_HOST" ]; then
  echo "[ego] WARN: gateway $GATEWAY != relay host $RELAY_HOST — check which is right" >&2
fi

if ping -c 2 -W 2 "$RELAY_HOST" >/dev/null 2>&1; then
  echo "[ego] relay host reachable"
else
  echo "[ego] WARN: cannot reach $RELAY_HOST" >&2
fi

# Internet is the point of the new topology: it is what keeps the clock sane.
if timeout 8 getent hosts github.com >/dev/null 2>&1; then
  echo "[ego] internet reachable — NTP can keep this clock in sync"
else
  echo "[ego] WARN: no internet; the clock will drift and V2V packets will be dropped" >&2
fi

echo "done."
