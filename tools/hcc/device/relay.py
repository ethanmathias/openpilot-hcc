#!/usr/bin/env python3
"""Entry point for the V2V relay, which runs on the EGO device.

The implementation lives in selfdrive/controls/lib/hcc/transport/relay.py so
that controlsd and the relay share one copy of the wire format. This wrapper
exists because the systemd unit installed by setup_network.sh invokes a path,
and a tool under tools/ is the right thing for it to point at.

  python3 tools/hcc/device/relay.py --host 0.0.0.0 --port 19090 --log_csv <path>
"""
from openpilot.selfdrive.controls.lib.hcc.transport.relay import RelayRouter, UDPRelayServer, main

__all__ = ["RelayRouter", "UDPRelayServer", "main"]


if __name__ == "__main__":
  main()
