#!/usr/bin/env bash
# HIL device launcher.
#
# Usage:  launch_device.sh <role> <PC_IP>
#         role  ∈ {lead, ego}
#         PC_IP is this device's RNDIS peer (the PC's USB-side IP),
#               used as the ZMQ_REMOTE_HOST for SubMaster connections.
#
# Pairs with system/manager/process_config.py — HIL_MODE=1 enables
# remote_sensor_bridge; BLOCK silences the on-device camerad/sensord/pandad.
# Normal-boot launch_chffrplus.sh is unaffected: just choose which launcher
# SSH executes to switch a device between HIL and real-car operation.

set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "usage: $0 <lead|ego> <PC_IP>" >&2
  exit 1
fi

ROLE="$1"
PC_IP="$2"
case "$ROLE" in
  lead|ego) ;;
  *) echo "role must be 'lead' or 'ego'" >&2; exit 1 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OPENPILOT_DIR="$SCRIPT_DIR/../../../.."

export PASSIVE=0
export NOBOARD=1
export SIMULATION=1
export SKIP_FW_QUERY=1
export FINGERPRINT="${FINGERPRINT:-HONDA_CIVIC_2022}"
export HIL_MODE=1
export HIL_ROLE="$ROLE"
export HIL_PC_IP="$PC_IP"
export BLOCK="camerad,pandad,sensord,loggerd,encoderd,micd,logmessaged,ui,updated"

python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"
python3 - <<'PY'
from openpilot.common.params import Params
p = Params()
p.put_bool("AlphaLongitudinalEnabled", True)
p.put_bool("EnableHCCC", True)
PY

# Device-side cereal bridge pair. msgq→zmq exposes the device's local services
# (carControl, carState, ...) for the PC's SubMaster to consume; zmq→msgq pulls
# the PC-published services (pandaStates, can, sensors, camera state) into
# the device's local msgq for pandad/sensord-replacement consumers.
SERVICES_FROM_PC="pandaStates can pandaState peripheralState driverStateV2 driverMonitoringState accelerometer gyroscope gpsLocationExternal liveTracks roadCameraState wideRoadCameraState"
cd "$OPENPILOT_DIR"
./cereal/messaging/bridge >/tmp/hil_bridge_msgq_to_zmq.log 2>&1 &
./cereal/messaging/bridge "$PC_IP" "$SERVICES_FROM_PC" >/tmp/hil_bridge_zmq_to_msgq.log 2>&1 &

cd "$OPENPILOT_DIR/system/manager" && exec ./manager.py
