# openpilot Simulator (MetaDrive): HC3 testing guide

This directory runs openpilot without a car. The MetaDrive driving simulator
provides the world (road, vehicles, cameras), and a bridge process translates
between the simulator and openpilot's normal message interfaces. openpilot runs
its standard pipeline and drives the simulated car.

```
MetaDrive (world) --camera/CAN/sensors--> openpilot --gas/brake/steer--> MetaDrive
                            (the bridge carries both directions)
```

Vocabulary:

- ego: the car openpilot is driving.
- lead: the car in front that the ego follows.
- hCCC / HC3: the cooperative cruise controller. It computes the ego's
  acceleration from the lead's speed/accel, received either from radar or over
  V2V.
- V2V: vehicle-to-vehicle. The lead transmits its speed/accel over the network
  (UDP through a small relay server) instead of the ego sensing it.
- scenario: a pre-recorded lead-car speed profile, one column of
  [`lib/Scenarios.csv`](lib/Scenarios.csv) (column 0 is time in seconds, column N
  is scenario N's speed in m/s). `--scn 48` replays column 48. Scenarios make runs
  repeatable and comparable.
- bridge: `run_bridge.py`, the process that connects MetaDrive to openpilot.

## Which command do I run?

| Goal | Where | Commands |
|---|---|---|
| Test hCCC against a scripted lead (simplest) | PC | Mode 1 below, 2 terminals |
| Test the full V2V path (lead publishes, ego subscribes) | PC | Mode 2 below, 4 terminals, or `./tools/hcc_v2v/launch_ui.sh` |
| Test V2V through the cloud relay | PC + Lightsail | [Lightsail relay testing](#lightsail-relay-testing) |
| Test in a real car (virtual lead from scenario data) | Car + 2 devices + laptop | [`tools/real_world_testing/README.md`](../real_world_testing/README.md), not this directory |
| Just drive around the simulator | PC | `uv run python tools/sim/run_bridge.py --mode default --keyboard` |

Real Comma 3X devices fed by MetaDrive (HIL) is not currently used: the Comma 3X's
USB-C port is wired to the panda, so the PC-to-device link it depends on is not
available. In-car testing replaced it.

## Setup (once)

From the repo root:

```bash
tools/op.sh setup
scons -u -j$(nproc) --minimal
```

`--minimal` builds only what the sim needs. The default build also compiles
`tools/cabana`, which fails if an Anaconda `(base)` env is active (it injects
Anaconda's old Qt, giving errors like `horizontalAdvance` / `QRandomGenerator not
found`). Either use `--minimal`, or `conda deactivate` before a full build.

Every command below is run from the repo root. `uv run python ...` runs a command
inside the project's virtualenv; `source .venv/bin/activate` once per terminal
works too.

## Mode 1: single sim, radar lead (start here)

Everything on one PC, one openpilot instance. MetaDrive renders the lead car;
openpilot sees it through the simulated camera and follows it with hCCC using
radar-style data. No V2V involved.

```bash
# Terminal 1: openpilot
./tools/sim/launch_openpilot_ego.sh

# Terminal 2: the world, with the lead following scenario 48
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard
```

(Or `./tools/sim/open_sim_terminals.sh` opens both terminals for you.)

A MetaDrive window opens. Press `1` to engage cruise. The ego follows the lead
car; telemetry is logged automatically (see [Run outputs](#run-outputs)).

Common mistake: setting `HCC_V2V_ENABLED=1` in this mode. With V2V enabled but no
relay running, hCCC waits forever for a V2V signal and the car never accelerates
even though it shows engaged. Mode 1 wants V2V off (the default in
`launch_openpilot_ego.sh`).

## Mode 2: dual sim, V2V lead

Two openpilot instances on one PC (separate message namespaces `hccego` and
`hcclead`), plus a local V2V relay. The lead instance publishes its state; the ego
instance subscribes and follows via V2V instead of radar. This exercises the same
code path as the real two-car deployment.

One-time: the lead instance runs from a second checkout on the `hcc-lead` branch
(it contains the lead-side `v2vpublisher` process):

```bash
git fetch origin
GIT_LFS_SKIP_SMUDGE=1 git worktree add ../openpilot-hcc-lead hcc-lead
```

```bash
# Terminal 1: local relay
uv run python tools/hcc_v2v/relay_server.py --host 127.0.0.1 --port 19090

# Terminal 2: ego instance (subscribes)
export HCC_V2V_ENABLED=1 HCC_V2V_ONLY=1 HCC_V2V_DEVICE_ID=ego-sim
./tools/sim/launch_openpilot_ego.sh

# Terminal 3: lead instance (publishes)
export HCC_V2V_ENABLED=1 HCC_V2V_DEVICE_ID=lead-sim
cd ../openpilot-hcc-lead && ./tools/sim/launch_openpilot_lead.sh

# Terminal 4: the world, simulating both vehicles
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard --lead_prefix hcclead
```

Or start all four at once with the launcher UI:

```bash
./tools/hcc_v2v/launch_ui.sh
```

Mental model: the bridge owns the world and drives the lead car along the
scenario; the lead instance publishes that car's state over V2V; the ego instance
receives it through the relay and runs hCCC on it.

### Mode comparison

| | Mode 1 | Mode 2 | In-car (phase 1) |
|---|---|---|---|
| openpilot runs on | PC x1 | PC x2 | Comma 3X in a real car |
| hCCC's lead source | camera/radar pipeline | V2V relay (local) | V2V relay (on-device, virtual lead) |
| `HCC_V2V_ENABLED` / `_ONLY` | `0` / `0` | `1` / `1` | params set by device setup script |
| Guide | this file | this file | [`tools/hcc_v2v/README.md`](../hcc_v2v/README.md) |

## Lightsail relay testing

Mode 2 with the relay on a cloud host instead of localhost; validates the future
cellular V2V transport. Differences from Mode 2:

```bash
# Terminal 1: watch the cloud relay's log
ssh ubuntu@<LIGHTSAIL_IP>  tail -f /var/log/hcc-v2v/hcc-v2v-relay.csv

# Terminal 2: ego, pointed at the cloud relay
export HCC_V2V_ENABLED=1 HCC_V2V_ONLY=1 HCC_V2V_DEVICE_ID=ego-sim
export HCC_V2V_RELAY_HOST=<LIGHTSAIL_IP> HCC_V2V_RELAY_PORT=19090
./tools/sim/launch_openpilot_ego.sh

# Terminal 3: lead, same relay
cd ../openpilot-hcc-lead
export HCC_V2V_ENABLED=1 HCC_V2V_DEVICE_ID=lead-sim
export HCC_V2V_RELAY_HOST=<LIGHTSAIL_IP> HCC_V2V_RELAY_PORT=19090
./tools/sim/launch_openpilot_lead.sh

# Terminal 4: bridge, as in Mode 2
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard --lead_prefix hcclead
```

Success: the relay log shows `registered_peer` for both `ego-sim` and `lead-sim`,
then a stream of `forwarded`. If the log stays empty, check the instance firewall
allows UDP 19090.

## Bridge reference

### Modes (`--mode`)

- `default`: free driving on a loop map, no lead car.
- `hc3`: straight road with a lead car; the preset for Modes 1 and 2.
- `hcc_hil`: hardware-in-the-loop (not currently used).

### Driving input

A Logitech wheel is auto-detected; otherwise it falls back to keyboard.

```bash
--keyboard                 # force keyboard
--logitech_wheel           # require the wheel (error if absent)
--wheel_device /dev/input/event5
--joystick                 # generic joystick
```

Keyboard controls:

| key | action | | key | action |
|---|---|---|---|---|
| `1` | cruise resume / accel | | `w` / `s` | throttle / brake |
| `2` | cruise set / decel | | `a` / `d` | steer left / right |
| `3` | cruise cancel | | `z` / `x` | blinkers |
| `i` | toggle ignition | | `r` / `q` | reset / quit |

If Linux denies wheel access: `sudo usermod -aG input $USER`, then re-login.

### Scenario replay

```bash
--scn 48                       # lead follows column 48 of lib/Scenarios.csv
--scn_csv /path/to/other.csv   # alternate CSV
```

### Run outputs

Every replay run logs telemetry automatically:

- CSVs land in `tools/sim/data/` (`Scenarios.openpilot.scn48.<timestamp>.csv`).
- `--output_csv PATH` / `--output_graph PATH` override the CSV/PNG locations.

Known-good scenario-48 reference outputs:
`tools/sim/data/hccc/Test48.vehicle.honda_civic_2022_ICE.hccc.csv` and the
matching PNG under `tools/sim/graphs/hccc/`.

### Other options

```bash
--dual_camera     # publish road + wide-road cameras
--high_quality    # nicer MetaDrive rendering
-h                # everything else
```

## How it works (architecture)

openpilot side: `launch_openpilot_ego.sh` starts openpilot with simulation env
vars (`SIMULATION=1`, `NOBOARD=1`, `SKIP_FW_QUERY=1`,
`FINGERPRINT=HONDA_CIVIC_2022`) and blocks hardware processes like the real
`camerad` so the bridge can supply those interfaces instead.

Bridge side: each tick, the bridge

1. reads ego/lead state and rendered camera frames from MetaDrive,
2. publishes them as openpilot messages: camera frames through VisionIPC
   (RGB to NV12, as if from `camerad`), a synthesized Honda Civic CAN stream
   (wheel speeds, steering, cruise buttons to `carState`), simplified GPS/IMU, and
   an always-attentive driver-monitoring stream,
3. reads openpilot's `carControl` output and applies gas/brake/steer to the
   MetaDrive ego vehicle.

The lead and "radar": there is no radar physics. The bridge computes the lead's
relative distance/velocity from world state and publishes it as `liveTracks`;
openpilot's normal `radard` fuses that with the camera model into `radarState`.
(Historical note: instability in this synthetic `liveTracks` to `radarState` trust
path, not the longitudinal controller, was the root cause of past erratic
radar-lead handoffs.)

Fidelity: strong on the process pipeline, camera-to-model path, lead-follow and
planner behavior, and scenario repeatability. Simplified on CAN, panda state, GPS,
IMU, driver monitoring, and radar sensing. Lateral control is not the focus of
this branch; longitudinal behavior is.

## Troubleshooting

Engaged but never accelerates (`accelCmd=0.00`): V2V settings do not match the
mode. Mode 1 needs V2V off (the default); Mode 2 needs `HCC_V2V_ENABLED=1
HCC_V2V_ONLY=1` and the relay already running. With V2V enabled and no relay, hCCC
sees `v2v_lead.status=False` and commands nothing.

openpilot starts but no simulator window: the bridge is not running. Start
`run_bridge.py` in a second terminal.

`ModuleNotFoundError` / nothing starts: the terminal is not in the venv
(`source .venv/bin/activate`, or prefix commands with `uv run`). With multiple
checkouts, make sure `VIRTUAL_ENV` is not pointing at another repo's venv
(`unset VIRTUAL_ENV`, then `uv run --active ...`).

Wheel input dead: try `--keyboard` to confirm the bridge works, check
`ls -l /dev/input/by-id`, and add yourself to the `input` group.

Scenario replay fails: `--scn` must be 1 or greater and the CSV must exist at
`tools/sim/lib/Scenarios.csv` (or pass `--scn_csv`).

`launch_ui.sh`: `No module named 'tkinter'`: `sudo apt install -y python3-tk`,
restart the shell.

`git worktree add ... hcc-lead` fails on a Git LFS object:
`GIT_LFS_SKIP_SMUDGE=1 git worktree add ../openpilot-hcc-lead hcc-lead`.

`scons` fails in `tools/cabana` mentioning Qt APIs (for example
`horizontalAdvance`): cabana is picking up Anaconda's old Qt. The sim stack does
not need cabana; to fix, deactivate conda, clear `CPATH/LD_LIBRARY_PATH/QT*` env
vars, ensure `qmake -v` resolves to system Qt (`sudo apt install qtbase5-dev
qtchooser qt5-qmake qtbase5-dev-tools libqt5serialbus5-dev`), and rebuild from a
clean shell.

`scons`: `No module named 'opendbc'`: the build started in a contaminated
environment. Re-run from a clean activated venv:
`source .venv/bin/activate && scons -u -j$(nproc)`.
