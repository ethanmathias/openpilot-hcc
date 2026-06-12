# HC3 Hardware-in-the-Loop (HIL) Simulation

MetaDrive runs on the PC as the simulated world. Each Comma 3X runs the full
openpilot stack (modeld, plannerd, controlsd, hCCC, V2V) natively on-device.

```
Comma 3X (lead)                                     Comma 3X (ego)
  manager (HIL_MODE=1)                                manager (HIL_MODE=1)
  - remote_sensor_bridge (ZMQ PULL, decode H.264)     - remote_sensor_bridge
  - modeld / plannerd / controlsd / hCCC              - modeld / plannerd / controlsd / hCCC
  - V2V publisher ──WiFi (ego hotspot)──► relay_server.py (127.0.0.1:19090)
      │                                                       │
      │  USB-C / RNDIS (camera + CAN + carControl)            │ USB-C / RNDIS
      ▼                                                       ▼
                         PC (host)
              ┌─────────────────────────────┐
              │  MetaDriveHILBridge          │
              │   ├ RemoteSensors (lead)     │  H.264 road+wide @ 20 fps
              │   └ RemoteSensors (ego)      │  CAN / pandaStates via cereal bridge
              │  + two MetaDrive vehicles    │  carControl ← device
              │  + 3-pane pygame window      │
              │  + Logitech wheel / keyboard │
              └─────────────────────────────┘
```

Two transports, separated by purpose:
- **USB-C RNDIS** (PC ↔ each device): camera frames + CAN/sensors PC→device,
  `carControl` device→PC.
- **WiFi hotspot** (ego AP, lead client): V2V only. The PC is not in the V2V
  path; this matches the real two-car field deployment.

---

## Prerequisites

### PC
- Python packages: `pyav`, `zmq`, `pygame`, `tomllib` (stdlib ≥ 3.11)
- `cereal/messaging/bridge` binary built (`make` in the `cereal/` submodule)
- macOS: [HoRNDIS](https://joshuawise.com/horndis) for USB RNDIS driver
- Linux: `cdc_ether` / `rndis_host` kernel modules (usually already loaded)

### Each Comma 3X
- openpilot-hcc checked out at `/data/openpilot`
- `pyav` and `zmq` wheels in the device venv: `pip install pyav pyzmq`

---

## Step 1 — Configure `devices.toml`

Copy the example and fill in the real values:

```bash
cp tools/sim/hil/devices.example.toml tools/sim/hil/devices.toml
```

```toml
[lead]
iface = "enx00e04c360001"   # PC-side USB interface name for the lead device
ip    = "192.168.32.10"     # static IP the lead device will use on usb0
pc_ip = "192.168.32.1"      # PC's IP on the same link (ZMQ bind address)

[ego]
iface      = "enx00e04c360002"
ip         = "192.168.32.11"
pc_ip      = "192.168.32.2"
hotspot_ip = "10.42.0.1"   # ego's WiFi hotspot IP (used by the lead for V2V)
```

Find the interface name with `ip link show` (Linux) or `ifconfig` (macOS)
after plugging in each device. To tell the two `enx...` interfaces apart,
plug them in one at a time and watch which name appears.

### One-time for two-device runs: patch msgq on the PC

The stock `bridge` binary binds every service's ZMQ port on all interfaces,
so the second device's bridge dies at startup. Apply the bind-address patch
and rebuild (PC only — devices don't need it):

```bash
cd msgq_repo
git apply ../tools/sim/hil/patches/zmq_bind_address.patch
cd ..
scons -j8 cereal
```

With the patch, each device's bridge binds only its own `pc_ip`, so the two
pairs coexist. Ego-only runs work without the patch.

---

## Step 2 — RNDIS static IP (one-shot per device)

SSH into each device and run:

```bash
# On the lead device:
sudo /data/openpilot/tools/sim/hil/scripts/setup_rndis.sh 192.168.32.10 192.168.32.1

# On the ego device:
sudo /data/openpilot/tools/sim/hil/scripts/setup_rndis.sh 192.168.32.11 192.168.32.2
```

Verify from the PC:

```bash
ping 192.168.32.10   # lead
ping 192.168.32.11   # ego
```

---

## Step 3 — V2V network (one-shot, persistent)

### On the ego device

```bash
sudo /data/openpilot/tools/sim/hil/scripts/setup_v2v_network.sh ego
```

This:
1. Creates a WiFi hotspot (`hcc-v2v` / `hcc-v2v-research` by default)
2. Installs and starts `hcc-v2v-relay.service` on `0.0.0.0:19090`
3. Sets `HCCV2VRelayHost=127.0.0.1` in Params

### On the lead device

```bash
sudo /data/openpilot/tools/sim/hil/scripts/setup_v2v_network.sh lead
```

This joins the ego's hotspot and sets `HCCV2VRelayHost=<ego hotspot IP>`.

> **Note:** the Comma 3X has one WiFi radio, so joining the hotspot drops
> both devices off your regular WiFi network — SSH over the LAN IPs
> (e.g. `192.168.86.x`) stops working. Do Step 2 (RNDIS) first and SSH over
> the USB link (`comma@192.168.32.10` / `comma@192.168.32.11`) from then on.

Verify with:

```bash
# On lead:
ping 10.42.0.1          # ego hotspot IP

# On ego:
systemctl status hcc-v2v-relay
```

---

## Step 4 — Launch openpilot on each device

SSH into each device and run:

```bash
# On the lead device (PC_IP is the PC's USB-side IP for this device):
ssh comma@192.168.32.10
/data/openpilot/tools/sim/hil/scripts/launch_device.sh lead 192.168.32.1

# On the ego device:
ssh comma@192.168.32.11
/data/openpilot/tools/sim/hil/scripts/launch_device.sh ego 192.168.32.2
```

`launch_device.sh` exports `HIL_MODE=1` and `BLOCK=camerad,pandad,sensord,...`,
starts the cereal bridge pair for that device, then runs `manager.py`. Watch the
manager log for `remote_sensor_bridge` starting and the blocked processes NOT
appearing.

> **Single-device bring-up**: omit the lead step for ego-only testing.

---

## Step 5 — Preflight, then launch MetaDrive on the PC

Run the preflight first — it checks every leg (config, binaries, msgq patch,
RNDIS link, ping, SSH, branch, relay) and prints the exact fix for anything
that fails:

```bash
./tools/sim/hil/scripts/check_hil.sh --lead    # or without --lead for ego-only
```

Then launch:

```bash
# Two-device (ego + lead):
python -m tools.sim.hil.launch_pc --lead

# Ego only:
python -m tools.sim.hil.launch_pc

# Options:
#   --raw_yuv          Ship NV12 instead of H.264 (bring-up fallback, ~270 Mbit/s)
#   --dual_camera      Enable wide-road camera stream
#   --keyboard         Force keyboard input instead of auto-detecting wheel
#   --devices_toml PATH  Override path to devices.toml
```

Or via the main bridge script:

```bash
python tools/sim/run_bridge.py --mode hcc_hil [--lead] [--raw_yuv] [--dual_camera]
```

Expected behavior:
- MetaDrive window opens with three panes: top-down, ego POV, lead POV
- Both devices go onroad and `selfdriveState.active` becomes `True`
- Turning the wheel steers only the ego; lead is pure on-device hCCC
- With `EnableHCCC=1`, ego follows lead via V2V `a_lead`/`v_lead`

---

## Individual leg verification

Each leg can be tested independently before stacking.

### Camera leg

```bash
# PC → device (sends 30 s of colour-bar test pattern):
python -m tools.sim.hil.camera_encoder --device 192.168.32.10 --pattern testbars

# On device — check a VisionIpcClient receives frames:
python3 -c "
from msgq.visionipc import VisionIpcClient, VisionStreamType
c = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
c.connect()
buf = c.recv()
print('got frame', buf.width, buf.height)
"
```

### CAN / cereal bridge leg

```bash
# PC: start the bridge pair manually for the ego prefix:
python -m tools.sim.hil.cereal_bridges --prefix hccego --device 192.168.32.11

# PC: publish a test pandaStates message:
OPENPILOT_PREFIX=hccego python3 -c "
import cereal.messaging as messaging, time
pm = messaging.PubMaster(['pandaStates'])
while True:
    msg = messaging.new_message('pandaStates', 1)
    pm.send('pandaStates', msg)
    time.sleep(0.1)
"

# On device: verify it arrives:
python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['pandaStates'])
sm.update(1000)
print('pandaStates valid:', sm.valid['pandaStates'])
"
```

### V2V leg

```bash
# With relay running on ego, from the lead device:
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(5):
    s.sendto(json.dumps({'seq': i, 'v_lead': 10.0, 'a_lead': 0.0,
                         'timestamp_us': int(time.monotonic()*1e6)}).encode(),
             ('10.42.0.1', 19090))
    time.sleep(0.5)
print('sent')
"
```

---

## Switching between HIL and real-car operation

No reflash needed. Choose which launcher SSH executes:

| Mode | Command |
|------|---------|
| HIL  | `launch_device.sh <role> <PC_IP>` |
| Real car | `./launch_chffrplus.sh` (or via the normal AGNOS UI) |

All HIL env vars (`HIL_MODE`, `BLOCK`, `NOBOARD`, ...) are set only by
`launch_device.sh`. A normal boot is byte-equivalent to pre-HIL behavior.

---

## Directory structure

```
tools/sim/hil/
├── README.md                  — this file
├── devices.example.toml       — copy to devices.toml and fill in IPs
├── __init__.py
├── proto.py                   — wire format (header struct, port assignments)
├── camera_encoder.py          — PC-side H.264 encoder + ZMQ PUSH
├── remote_sensor_bridge.py    — device-side ZMQ PULL → VisionIPC republisher
├── remote_sensors.py          — IMU/GPS/camera publisher for one device role
├── cereal_bridges.py          — cereal/messaging/bridge subprocess management
├── device_config.py           — devices.toml loader + RNDIS IP sanity check
├── launch_pc.py               — top-level PC orchestrator
├── window.py                  — 3-pane pygame composite window
├── patches/
│   └── zmq_bind_address.patch — msgq patch for two-device bridge coexistence (PC only)
└── scripts/
    ├── check_hil.sh           — PC-side preflight: verifies every leg before launch
    ├── launch_device.sh       — device-side HIL launcher (runs on the Comma 3X)
    ├── setup_rndis.sh         — one-shot static RNDIS IP setup (on device)
    └── setup_v2v_network.sh   — one-shot hotspot + V2V relay setup (on device)
```

## Real-world readiness: what HIL must demonstrate before two-car testing

The V2V topology in HIL (ego hotspot + on-ego relay, lead as WiFi client) is
byte-identical to the planned two-car field setup — the RNDIS/PC leg is
replaced by real cameras and CAN, but the V2V leg doesn't change. Use HIL to
sign off on each of these before putting the devices in cars:

1. **Hotspot persistence across reboots.** Reboot both devices; the
   NetworkManager connections (`hcc-hotspot`, `hcc-hotspot-client`) are
   `autoconnect yes`, so the link should re-form with no intervention.
   Verify `ping 10.42.0.1` from the lead after both finish booting.
2. **Staleness behavior.** With the ego in V2V-only mode (`HCCV2VOnly=1`), a
   stale signal disengages HC3 rather than falling back to radar. Test by
   stopping `v2vpublisher` on the lead mid-run (or `systemctl stop
   hcc-v2v-relay` on the ego) and confirm the ego disengages cleanly within
   `STALE_THRESHOLD_MS` instead of holding the last acceleration.
3. **WiFi range/dropout.** In cars the devices will be 20–80 m apart through
   two windshields. In HIL, characterize link margin by adding distance or
   attenuation between the devices and watching packet loss in the relay CSV
   log (`/var/log/` on the ego). The hotspot is created on the 2.4 GHz band
   (`802-11-wireless.band bg`) for range.
4. **Latency budget.** The relay CSV logs give per-packet timestamps; confirm
   end-to-end lead→ego latency stays well under the 100 ms staleness
   threshold over the real WiFi link (expected: single-digit ms).
5. **Param persistence.** `setup_v2v_network.sh` writes the `HCCV2V*` params
   once; verify they survive a reboot (`python3 -c "from openpilot.common.params
   import Params; print(Params().get('HCCV2VRelayHost'))"`). Note that
   `EnableHCCC` and `AlphaLongitudinalEnabled` are set by `launch_device.sh`
   on every HIL launch — for real-car operation they must be set manually
   once, since the car uses the normal launcher.

The only V2V difference between HIL and the field deployment is *who powers
the devices and what feeds the cameras* — if items 1–5 pass in HIL, the V2V
link itself is field-ready. (The later cellular deployment via Lightsail is
a separate transport and needs its own validation.)

## Known limitations

- **Two-device cereal bridge port collision**: fixed by the msgq
  `ZMQ_BIND_ADDRESS` patch (see Step 1). Without the patch on the PC, the
  second `msgq→zmq` bridge fails to bind and the lead device receives
  nothing. `check_hil.sh --lead` verifies the patch is applied.
- **VisionIPC is shared-memory only**: it cannot cross the network. The
  `remote_sensor_bridge.py` on-device decoder is mandatory; do not attempt
  to publish VisionIPC from the PC.
- **Raw YUV bandwidth**: `--raw_yuv` ships NV12 (~270 Mbit/s combined),
  which is marginal on USB-RNDIS. Use only for initial bring-up; switch to
  H.264 (default) for sustained operation.
