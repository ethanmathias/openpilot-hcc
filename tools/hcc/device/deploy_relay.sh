#!/usr/bin/env bash
# Deploy the V2V relay to the host that serves the hotspot.
#
#   ./tools/hcc/device/deploy_relay.sh ap@127.0.0.1 -p 2222 -i <key>
#
# The relay used to run on the ego, which was also the access point. It now
# runs on whatever machine owns the hotspot -- currently the Debian VM in
# LinkLab/wn722n-ap, which drives the USB radio. Both the ego and the lead
# reach it directly on the hotspot subnet, and the pedal publisher on the Mac
# reaches it through the VM's UDP port forward, because the Mac is not on that
# subnet and slirp has no inbound route.
#
# Like the lead bundle this ships pure stdlib only: the relay imports nothing
# but the wire format, so the host needs no openpilot and no dependencies.
set -euo pipefail

TARGET="${1:-}"
shift || true
SSH_EXTRA=("$@")       # e.g. -p 2222 -i keys/id_ed25519
# macOS ships bash 3.2, which errors on "${arr[@]}" for an EMPTY array under
# `set -u`. Expand through this guard everywhere instead.
ssh_() { if [ ${#SSH_EXTRA[@]} -eq 0 ]; then ssh "$@"; else ssh "${SSH_EXTRA[@]}" "$@"; fi; }

DEST="\$HOME/hcc-relay"
PORT="${RELAY_PORT:-19090}"
LOG_DIR="${RELAY_LOG_DIR:-\$HOME/hcc-logs}"

if [ -z "$TARGET" ]; then
  echo "usage: $0 [user@]host [ssh options...]" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." >/dev/null && pwd)"
REMOTE_USER="$(ssh_ "$TARGET" 'id -un')"
cd "$REPO_ROOT"

FILES=(
  selfdrive/controls/lib/hcc/transport/protocol.py
  selfdrive/controls/lib/hcc/transport/relay.py
)
PKG_DIRS=(
  selfdrive
  selfdrive/controls
  selfdrive/controls/lib
  selfdrive/controls/lib/hcc
  selfdrive/controls/lib/hcc/transport
)

for f in "${FILES[@]}"; do
  [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

for d in "${PKG_DIRS[@]}"; do
  mkdir -p "$STAGE/$d"
  touch "$STAGE/$d/__init__.py"
done
for f in "${FILES[@]}"; do
  cp "$f" "$STAGE/$f"
done
mkdir -p "$STAGE/openpilot"
touch "$STAGE/openpilot/__init__.py"
ln -s ../selfdrive "$STAGE/openpilot/selfdrive"

echo "staged $(find "$STAGE" -type f | wc -l | tr -d ' ') files ($(du -sh "$STAGE" | cut -f1))"

# Stop the service before replacing its files, and clear the destination with
# sudo: an earlier deployment may have run as root and left root-owned
# __pycache__ behind, which the login user cannot delete.
ssh_ "$TARGET" "sudo systemctl stop hcc-relay 2>/dev/null; sudo rm -rf \"$DEST\"; mkdir -p \"$DEST\""
tar -C "$STAGE" -cf - . | ssh_ "$TARGET" "tar -C \"$DEST\" -xf -"

# A per-boot CSV, as on the ego: every datagram the relay sees, with the
# forward decision. It is the spine of the run analysis and the only reason
# the 2026-07-11 data was diagnosable.
ssh_ "$TARGET" "sudo tee /etc/systemd/system/hcc-relay.service >/dev/null <<UNIT
[Unit]
Description=HCC V2V relay
After=network.target linklab-ap.service

[Service]
Type=simple
WorkingDirectory=$DEST
Environment=PYTHONPATH=$DEST
User=$REMOTE_USER
ExecStartPre=/bin/sh -c 'mkdir -p $LOG_DIR'
ExecStart=/bin/sh -c 'exec python3 -m openpilot.selfdrive.controls.lib.hcc.transport.relay --host 0.0.0.0 --port $PORT --log_csv $LOG_DIR/relay_\\\$(date +%%Y%%m%%d_%%H%%M%%S).csv'
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable --now hcc-relay.service
sleep 1
systemctl is-active hcc-relay.service"

# Prove it is actually listening rather than assuming the unit started.
ssh_ "$TARGET" "
  ss -lunp 2>/dev/null | grep -q ':$PORT ' && echo '  listening on udp/$PORT' || { echo '  NOT listening on udp/$PORT' >&2; exit 1; }
  echo '  log dir: $LOG_DIR'
"
echo "relay deployed to $TARGET:$DEST"
