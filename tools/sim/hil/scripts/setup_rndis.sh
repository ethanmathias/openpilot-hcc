#!/usr/bin/env bash
# One-shot per device: set a static IP on the USB RNDIS interface so the
# PC's tools/sim/hil/devices.toml entry stays valid across reboots.
#
# Usage:  sudo setup_rndis.sh <DEVICE_IP> <PC_IP> [iface]
#         DEVICE_IP  IP this device should claim on its USB peer-link.
#         PC_IP      IP the PC has on its side of the same link (the gateway).
#         iface      RNDIS interface name on the device (default: usb0).

set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "usage: $0 <DEVICE_IP> <PC_IP> [iface]" >&2
  exit 1
fi

DEVICE_IP="$1"
PC_IP="$2"
IFACE="${3:-usb0}"

INTERFACES_D=/data/etc/network/interfaces.d
mkdir -p "$INTERFACES_D"

cat > "$INTERFACES_D/$IFACE" <<EOF
auto $IFACE
iface $IFACE inet static
  address $DEVICE_IP
  netmask 255.255.255.0
  gateway $PC_IP
EOF

echo "wrote $INTERFACES_D/$IFACE"
echo "bringing $IFACE up with $DEVICE_IP"
ip addr flush dev "$IFACE" 2>/dev/null || true
ip addr add "$DEVICE_IP/24" dev "$IFACE"
ip link set "$IFACE" up
echo "done. Test with: ping -c 1 $PC_IP"
