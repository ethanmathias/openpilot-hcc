#!/usr/bin/env bash
# HIL preflight — run on the PC before launch_pc.py to catch setup problems
# early instead of mid-launch. Read-only; changes nothing.
#
# Usage:  ./tools/sim/hil/scripts/check_hil.sh [--lead]
#         --lead   also check the lead device (two-vehicle HIL)
#
# Checks, in dependency order:
#   1. devices.toml present and parseable
#   2. cereal/messaging/bridge binary built
#   3. msgq ZMQ_BIND_ADDRESS patch applied (required for --lead)
#   4. Python deps importable (av, zmq, pygame)
#   5. RNDIS interfaces up with the configured pc_ip
#   6. Devices reachable by ping
#   7. SSH reachable; correct branch checked out on each device
#   8. V2V relay service running on the ego (via SSH)

set -uo pipefail

WITH_LEAD=0
[ "${1:-}" = "--lead" ] && WITH_LEAD=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
HIL_DIR="$SCRIPT_DIR/.."
OPENPILOT_DIR="$(cd "$HIL_DIR/../../.." >/dev/null && pwd)"
TOML="$HIL_DIR/devices.toml"

PASS=0
FAIL=0
ok()   { echo "  [ OK ] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
info() { echo "         $1"; }

roles="ego"
[ "$WITH_LEAD" = 1 ] && roles="ego lead"

echo "== HIL preflight (roles: $roles) =="

# 1. devices.toml
if [ ! -f "$TOML" ]; then
  bad "devices.toml missing — cp $HIL_DIR/devices.example.toml $TOML and edit"
  echo "Aborting: later checks need devices.toml."
  exit 1
fi
ok "devices.toml present"

read_toml() {  # read_toml <role> <key> → value or empty
  python3 - "$TOML" "$1" "$2" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    raw = tomllib.load(f)
print(raw.get(sys.argv[2], {}).get(sys.argv[3], ""))
PY
}

for role in $roles; do
  for key in iface ip pc_ip; do
    val="$(read_toml "$role" "$key")"
    if [ -z "$val" ]; then
      bad "devices.toml [$role].$key is missing"
    fi
  done
done
ok "devices.toml has iface/ip/pc_ip for: $roles"

# 2. bridge binary
BRIDGE="$OPENPILOT_DIR/cereal/messaging/bridge"
if [ -x "$BRIDGE" ]; then
  ok "cereal bridge binary built"
else
  bad "bridge binary missing at $BRIDGE — run scons"
fi

# 3. msgq patch (only required for two-device runs)
IMPL="$OPENPILOT_DIR/msgq_repo/msgq/impl_zmq.cc"
if grep -q "ZMQ_BIND_ADDRESS" "$IMPL" 2>/dev/null; then
  ok "msgq ZMQ_BIND_ADDRESS patch applied"
else
  if [ "$WITH_LEAD" = 1 ]; then
    bad "msgq patch NOT applied — two-device runs will fail (lead bridge port collision)."
    info "Apply with: cd $OPENPILOT_DIR/msgq_repo && git apply ../tools/sim/hil/patches/zmq_bind_address.patch && cd .. && scons -j8 cereal"
  else
    info "[note] msgq patch not applied — fine for ego-only, required before --lead"
  fi
fi

# 4. Python deps
for mod in av zmq pygame; do
  if python3 -c "import $mod" 2>/dev/null; then
    ok "python module '$mod' importable"
  else
    bad "python module '$mod' missing — pip install $mod"
  fi
done

# 5–8. Per-device link checks
for role in $roles; do
  iface="$(read_toml "$role" iface)"
  dev_ip="$(read_toml "$role" ip)"
  pc_ip="$(read_toml "$role" pc_ip)"
  echo "-- $role ($dev_ip via $iface) --"

  if ip link show "$iface" >/dev/null 2>&1; then
    ok "$role: interface $iface exists"
    if ip -4 addr show "$iface" | grep -q "$pc_ip"; then
      ok "$role: PC has $pc_ip on $iface"
    else
      bad "$role: $pc_ip not on $iface — sudo ip addr add $pc_ip/24 dev $iface && sudo ip link set $iface up"
    fi
  else
    bad "$role: interface $iface not found — is the device plugged in? (check 'ip link show' for enx*)"
    continue
  fi

  if ping -c 1 -W 2 "$dev_ip" >/dev/null 2>&1; then
    ok "$role: device answers ping at $dev_ip"
  else
    bad "$role: no ping at $dev_ip — run setup_rndis.sh on the device (see README step 2)"
    continue
  fi

  SSH="ssh -o BatchMode=yes -o ConnectTimeout=3 comma@$dev_ip"
  branch="$($SSH "cd /data/openpilot && git rev-parse --abbrev-ref HEAD" 2>/dev/null)"
  if [ -z "$branch" ]; then
    bad "$role: SSH failed (comma@$dev_ip) — check ssh key / GitHub SSH setup"
    continue
  fi
  expect="hcc-$role"
  if [ "$branch" = "$expect" ]; then
    ok "$role: on branch $branch"
  else
    bad "$role: on branch '$branch', expected '$expect'"
  fi

  if [ "$role" = "ego" ]; then
    if $SSH "systemctl is-active --quiet hcc-v2v-relay" 2>/dev/null; then
      ok "ego: hcc-v2v-relay.service active"
    else
      bad "ego: V2V relay not running — sudo setup_v2v_network.sh ego (on the device)"
    fi
  else
    hotspot_ip="$(read_toml ego hotspot_ip)"
    if $SSH "ping -c 1 -W 2 ${hotspot_ip:-10.42.0.1}" >/dev/null 2>&1; then
      ok "lead: can reach ego hotspot at ${hotspot_ip:-10.42.0.1}"
    else
      bad "lead: cannot reach ego hotspot — sudo setup_v2v_network.sh lead (on the device)"
    fi
  fi
done

echo "== $PASS passed, $FAIL failed =="
[ "$FAIL" = 0 ] && echo "Ready: launch_device.sh on each device, then python -m tools.sim.hil.launch_pc$([ "$WITH_LEAD" = 1 ] && echo ' --lead')"
exit "$((FAIL > 0))"
