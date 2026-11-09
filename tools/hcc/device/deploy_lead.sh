#!/usr/bin/env bash
# Deploy the lead-vehicle publisher to a Raspberry Pi (or any Linux host).
#
#   ./tools/hcc/device/deploy_lead.sh pi@10.42.0.50 [dest]
#
# The lead does not need openpilot. The publisher and the wire format are pure
# stdlib, so this ships ~100 KB instead of a multi-gigabyte checkout: four
# Python modules, the scenario table, and the package directories that make
# `import openpilot.tools...` resolve.
#
# Default destination is ~/hcc-lead. Run it with PYTHONPATH set to that
# directory; `hcc run` does so automatically from config/devices.toml.
#
# Re-run this after changing anything in the file list below.
set -euo pipefail

TARGET="${1:-}"
DEST="${2:-\$HOME/hcc-lead}"

if [ -z "$TARGET" ]; then
  echo "usage: $0 [user@]host [dest]   (dest default: \$HOME/hcc-lead)" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." >/dev/null && pwd)"
cd "$REPO_ROOT"

# Everything the lead publisher needs, and nothing else. Extend this list when
# the lead gains a sensor publisher; keep it minimal so the bundle stays small
# and its Python version requirements stay low.
FILES=(
  selfdrive/controls/lib/hcc/transport/protocol.py
  tools/hcc/publishers/profile.py
  tools/hcc/publishers/publisher.py
  tools/hcc/publishers/virtual_lead.py
  tools/hcc/config/scenarios.csv
)

# Package directories that need an __init__.py for the imports to resolve.
PKG_DIRS=(
  selfdrive
  selfdrive/controls
  selfdrive/controls/lib
  selfdrive/controls/lib/hcc
  selfdrive/controls/lib/hcc/transport
  tools
  tools/hcc
  tools/hcc/publishers
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
mkdir -p "$STAGE/tools/hcc/config"

for f in "${FILES[@]}"; do
  mkdir -p "$STAGE/$(dirname "$f")"
  cp "$f" "$STAGE/$f"
done

# `openpilot.<...>` is how the modules import each other. In the repo that is a
# directory of symlinks; here it is the same thing built from scratch, so the
# import paths are identical on both sides and nothing has to be rewritten.
mkdir -p "$STAGE/openpilot"
touch "$STAGE/openpilot/__init__.py"
ln -s ../selfdrive "$STAGE/openpilot/selfdrive"
ln -s ../tools "$STAGE/openpilot/tools"

echo "staged $(find "$STAGE" -type f | wc -l | tr -d ' ') files ($(du -sh "$STAGE" | cut -f1))"

ssh -o BatchMode=yes -o ConnectTimeout=6 "$TARGET" "mkdir -p \"$DEST\""
rsync -az --delete --exclude='__pycache__' "$STAGE"/ "$TARGET:$DEST"/

echo "deployed to $TARGET:$DEST"

# Confirm it actually runs there rather than assuming it does.
ssh -o BatchMode=yes "$TARGET" "cd \"$DEST\" && python3 -c '
import sys
print(\"  python %d.%d.%d\" % sys.version_info[:3])
sys.path.insert(0, \".\")
from openpilot.tools.hcc.publishers.virtual_lead import VirtualLeadPublisher
from openpilot.tools.hcc.publishers.profile import load_speed_profile
t, v = load_speed_profile(\"tools/hcc/config/scenarios.csv\", 48)
print(\"  scenario 48 loads: %d samples, peak %.1f m/s\" % (len(t), max(v)))
print(\"  publisher imports OK\")
'"
echo "lead is ready"
