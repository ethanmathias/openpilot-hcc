#!/usr/bin/env bash
# One-shot V2V network setup for two Comma 3X devices (in-car testing or HIL).
#
# Usage:
#   On the EGO device:    sudo setup_v2v_network.sh ego  [<SSID> <PSK>]
#   On the LEAD device:   sudo setup_v2v_network.sh lead <SSID> <PSK>
#
# Ego: enables the device's WiFi hotspot and installs+enables a systemd
# unit that runs tools/hcc_v2v/relay_server.py on 0.0.0.0:19090, logging
# every packet to a per-boot CSV under /data/hcc_v2v_logs/. Lead joins the
# same hotspot. No other machine is in the V2V path; this is the same
# topology as the real two-car field deployment.
#
# Both roles also get their HCCV2V* params set: ego → 127.0.0.1, lead →
# the ego's hotspot IP (printed at the end of the ego setup).

set -euo pipefail

ROLE="${1:-}"
SSID="${2:-hcc-v2v}"
PSK="${3:-hcc-v2v-research}"

if [ "$ROLE" != "ego" ] && [ "$ROLE" != "lead" ]; then
  echo "usage: $0 <ego|lead> [SSID] [PSK]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OPENPILOT_DIR="$(readlink -f "$SCRIPT_DIR/../../..")"
RELAY_PY="$OPENPILOT_DIR/tools/hcc_v2v/relay_server.py"

if [ ! -f "$RELAY_PY" ]; then
  echo "relay_server.py not found at $RELAY_PY" >&2
  exit 1
fi

set_param() {
  local key="$1"
  local val="$2"
  # openpilot's python deps (zmq, ...) live in the repo venv, which only an
  # interactive comma shell puts on PATH — use the venv python directly.
  # Run as the invoking user so params files keep their normal ownership.
  local py="$OPENPILOT_DIR/.venv/bin/python3"
  [ -x "$py" ] || py="python3"
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

if [ "$ROLE" = "ego" ]; then
  # IMPORTANT ORDERING: bringing the hotspot up swaps wlan0 out of the LAN,
  # which kills a WiFi SSH session — and with it, this script. Everything
  # that must complete (relay unit, params) therefore happens BEFORE the
  # network switch, which is the very last step.
  echo "[ego] installing systemd unit hcc-v2v-relay.service"
  UNIT_PATH=/etc/systemd/system/hcc-v2v-relay.service
  TMP_UNIT="$(mktemp)"
  cat > "$TMP_UNIT" <<EOF
[Unit]
Description=HCC V2V relay (ego device, on-device)
After=network.target

[Service]
Type=simple
WorkingDirectory=$OPENPILOT_DIR
ExecStart=/bin/sh -c 'mkdir -p /data/hcc_v2v_logs && exec python3 $RELAY_PY --host 0.0.0.0 --port 19090 --log_csv /data/hcc_v2v_logs/relay_\$\$(date +%%Y%%m%%d_%%H%%M%%S).csv'
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
  if cmp -s "$TMP_UNIT" "$UNIT_PATH" 2>/dev/null; then
    echo "[ego] relay unit already up to date — skipping install"
  else
    # AGNOS keeps the system partition read-only; remount rw just for the
    # unit install. Note: the unit lives on the system partition, so an
    # AGNOS update/reflash removes it — re-run this script afterwards.
    REMOUNTED=0
    if ! touch /etc/systemd/system/.hcc_rw_test 2>/dev/null; then
      mount -o remount,rw /
      REMOUNTED=1
    else
      rm -f /etc/systemd/system/.hcc_rw_test
    fi
    cp "$TMP_UNIT" "$UNIT_PATH"
    systemctl daemon-reload
    if [ "$REMOUNTED" = "1" ]; then
      mount -o remount,ro / || echo "[ego] WARN: could not remount / read-only; a reboot will restore it" >&2
    fi
  fi
  rm -f "$TMP_UNIT"
  systemctl enable --now hcc-v2v-relay.service
  systemctl status --no-pager hcc-v2v-relay.service | head -8 || true

  echo "[ego] setting V2V + HC3 params"
  set_param HCCV2VEnabled 1
  set_param HCCV2VOnly 1
  set_param HCCV2VDeviceId hcc-ego
  set_param HCCV2VRelayHost 127.0.0.1
  set_param HCCV2VRelayPort 19090
  set_param EnableHCCC 1
  set_param AlphaLongitudinalEnabled 1

  echo "[ego] enabling hotspot SSID=$SSID — if you are SSHed in over WiFi,"
  echo "[ego] your session will drop NOW. That is fine: setup is complete."
  echo "[ego] Reconnect by joining the '$SSID' network and: ssh comma@10.42.0.1"
  # Comma 3X uses NetworkManager. Reuse an existing hotspot connection if
  # present, otherwise create one. ifname=wlan0 is the standard radio on AGNOS.
  if ! nmcli -t -f NAME connection show | grep -qx "hcc-hotspot"; then
    nmcli connection add type wifi ifname wlan0 con-name hcc-hotspot autoconnect yes ssid "$SSID" \
      mode ap ipv4.method shared 802-11-wireless.band bg
    nmcli connection modify hcc-hotspot wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK"
  fi
  nmcli connection up hcc-hotspot
  HOTSPOT_IP="$(ip -4 -o addr show wlan0 | awk '{print $4}' | cut -d/ -f1 | head -n1)"
  echo "[ego] hotspot up at $HOTSPOT_IP — lead should run: sudo $0 lead $SSID $PSK"

elif [ "$ROLE" = "lead" ]; then
  if [ "$SSID" = "hcc-v2v" ] && [ "$PSK" = "hcc-v2v-research" ]; then
    echo "[lead] using default SSID/PSK; pass real values if you changed them on the ego" >&2
  fi

  # Params first: joining the hotspot swaps wlan0 off the LAN, which kills a
  # WiFi SSH session — and with it, this script. NetworkManager shared mode
  # always puts the ego (the AP) at 10.42.0.1.
  RELAY_HOST="${4:-10.42.0.1}"
  echo "[lead] setting V2V params (relay host $RELAY_HOST)"
  set_param HCCV2VEnabled 1
  set_param HCCV2VOnly 0
  set_param HCCV2VDeviceId hcc-lead
  set_param HCCV2VRelayHost "$RELAY_HOST"
  set_param HCCV2VRelayPort 19090

  echo "[lead] joining hotspot SSID=$SSID — if you are SSHed in over WiFi,"
  echo "[lead] your session will drop NOW. That is fine: setup is complete."
  echo "[lead] Reconnect from a machine on the '$SSID' network."
  if ! nmcli -t -f NAME connection show | grep -qx "hcc-hotspot-client"; then
    nmcli connection add type wifi ifname wlan0 con-name hcc-hotspot-client ssid "$SSID" \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" autoconnect yes
  fi
  nmcli connection up hcc-hotspot-client
  # Anything below may not run if SSH died with the network switch — keep it
  # to best-effort verification only.
  GATEWAY="$(ip -4 route | awk '/default/ {print $3; exit}')"
  if [ -n "$GATEWAY" ] && [ "$GATEWAY" != "$RELAY_HOST" ]; then
    echo "[lead] WARN: gateway is $GATEWAY but HCCV2VRelayHost=$RELAY_HOST — fix with:" >&2
    echo "[lead]   python3 -c \"from openpilot.common.params import Params; Params().put('HCCV2VRelayHost','$GATEWAY')\"" >&2
  fi
  ping -c 2 -W 2 "$RELAY_HOST" || echo "[lead] WARN: ping failed; relay may not be reachable yet" >&2
fi

echo "done."
