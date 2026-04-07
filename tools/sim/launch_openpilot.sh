#!/usr/bin/env bash

# Activate virtual environment
SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../
source $OPENPILOT_DIR/.venv/bin/activate

unset DEBUG IMAGE BEAM NOOPT
export DEBUG=0

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"
export OPENPILOT_PREFIX="${OPENPILOT_PREFIX:-hcclead}"

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"
python3 -c 'from openpilot.common.params import Params; p=Params(); p.put_bool("AlphaLongitudinalEnabled", True); p.put_bool("EnableHCCC", True)'

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
