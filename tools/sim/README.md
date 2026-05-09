openpilot Simulator (MetaDrive)
===============================

This directory contains the MetaDrive-based simulator used to run openpilot in a software-in-the-loop configuration.

In this setup:

- MetaDrive provides the virtual environment, ego vehicle, cameras, and optional lead vehicle.
- The simulator bridge converts simulator state into the message interfaces expected by openpilot.
- openpilot runs its normal process pipeline and returns control commands to the simulator.

At a high level, the runtime loop is:

```text
MetaDrive -> simulated sensors / simulated vehicle interfaces -> openpilot -> control commands -> MetaDrive
```

## Scope

This simulator is primarily intended for:

- longitudinal and lead-follow testing
- scenario replay using `Scenarios.csv`
- hCCC-related experimentation
- software integration and regression testing

This branch is not configured as a full hands-off lateral simulation environment. Longitudinal behavior is the primary focus.

## Primary Entry Points

The most commonly used scripts are:

- `./tools/sim/launch_openpilot.sh`
  Starts openpilot in simulation mode.
- `./tools/sim/launch_openpilot_ego.sh`
  Starts the ego-side sim stack for the V2V HC3 workflow.
- `./tools/sim/run_bridge.py`
  Starts the MetaDrive bridge process and input handling.
- `./tools/sim/open_sim_terminals.sh`
  Opens two prepared terminals for the standard workflow.
- `./tools/hcc_v2v/launch_ui.sh`
  Opens the local V2V launcher UI for relay, ego, lead, and bridge startup.

## Prerequisites

Run all commands from the repository root unless noted otherwise:

```bash
cd /path/to/openpilot-hcc
```

Ensure the Python virtual environment is present:

```bash
ls .venv/bin/activate
```

If the virtual environment is missing, initialize the repository first. A setup path referenced by this repo is:

```bash
tools/op.sh setup
```

## Three Operational Modes

The HC3 simulation stack supports three distinct modes. Each mode exercises
progressively more of the real system. Understand which mode you need before
launching anything.

### Mode 1: Single-Device Sim (radar-based hCCC, no V2V)

Everything runs on one PC. MetaDrive renders a scripted lead vehicle. The ego
openpilot stack sees the lead through its simulated camera, and hCCC uses the
resulting `radarState.leadOne` to compute acceleration. **V2V is off.**

This is the simplest way to test hCCC longitudinal behavior.

```
PC
├── MetaDrive (world sim + scripted lead vehicle)
├── SimulatedCar (fake Honda CAN → openpilot)
├── SimulatedSensors (fake IMU/GPS/camera → openpilot)
└── openpilot (modeld → radard → controlsd/hCCC → carControl)
    hCCC input: radarState.leadOne (from camera/model)
    V2V: OFF
```

**Launch (2 terminals):**

```bash
# Terminal 1 — openpilot
./tools/sim/launch_openpilot_ego.sh

# Terminal 2 — MetaDrive bridge
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard
```

**Key V2V settings** (defaults in `launch_openpilot_ego.sh`):

| Variable | Value | Why |
|---|---|---|
| `HCC_V2V_ENABLED` | `0` | No V2V subscriber — hCCC uses radar lead only |
| `HCC_V2V_ONLY` | `0` | Not in V2V-only mode |

If `HCC_V2V_ENABLED=1` is set by accident, the V2V subscriber is created but
cannot reach a relay. hCCC sees `v2v_lead.status=False`, returns `None`, and
the car never accelerates — even though `Engaged: True` appears.

### Mode 2: Dual-Device Sim (V2V via relay, all on PC)

Two openpilot instances run on the same PC under different cereal prefixes
(`hccego` and `hcclead`). MetaDrive simulates both vehicles. A V2V relay runs
at `127.0.0.1:19090`. The lead's `v2vpublisher` sends its state through the
relay to the ego's V2V subscriber, and hCCC follows using V2V data.

```
PC
├── MetaDrive (two vehicles, one per prefix)
├── SimulatedCar × 2 (ego prefix + lead prefix)
├── openpilot "ego"  (OPENPILOT_PREFIX=hccego, V2V subscriber)
├── openpilot "lead" (OPENPILOT_PREFIX=hcclead, V2V publisher)
└── relay_server.py (127.0.0.1:19090)
    hCCC input: V2V lead data from the relay
    V2V: ON
```

**Launch (4 terminals):**

```bash
# Terminal 1 — V2V relay
uv run python tools/hcc_v2v/relay_server.py --host 127.0.0.1 --port 19090

# Terminal 2 — ego openpilot (V2V enabled, V2V-only mode)
HCC_V2V_ENABLED=1 HCC_V2V_ONLY=1 ./tools/sim/launch_openpilot_ego.sh

# Terminal 3 — lead openpilot
cd /path/to/openpilot-hcc-lead
./tools/sim/launch_openpilot_lead.sh

# Terminal 4 — bridge (feeds both prefixes)
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard --lead_prefix hcclead
```

If the lead worktree does not exist yet, create it from the ego repo:

```bash
cd /path/to/openpilot-hcc
git fetch origin
GIT_LFS_SKIP_SMUDGE=1 git worktree add ../openpilot-hcc-lead hcc-lead
```

`GIT_LFS_SKIP_SMUDGE=1` is recommended when the lead branch references LFS
objects that are unavailable from the remote.

#### V2V HC3 Launcher UI

A convenience UI can start all four processes at once:

```bash
./tools/hcc_v2v/launch_ui.sh
```

In the launcher UI, set the ego and lead repo paths and scenario number, then
use `Start all`. The UI uses the same launch scripts as the manual workflow.

### Mode 3: HIL (real Comma 3X devices, MetaDrive on PC)

Two physical Comma 3X devices run the real openpilot stack. MetaDrive on the
PC sends H.264 camera frames (ZMQ) and CAN/sensor data (cereal bridge) to
each device over USB-C RNDIS. Each device returns `carControl` over the same
link. V2V runs device-to-device over WiFi (ego hosts hotspot, lead joins as
client). The PC is not in the V2V path.

```
PC (MetaDrive + camera encoder + cereal bridges)
  │ USB-C RNDIS              │ USB-C RNDIS
  ▼                          ▼
Comma 3X (ego)             Comma 3X (lead)
  openpilot (full stack)     openpilot (full stack)
  V2V subscriber             V2V publisher
  relay_server.py            ◄── WiFi hotspot ──►
```

**Launch:** See [`tools/sim/hil/README.md`](hil/README.md) for the full
5-step bring-up (RNDIS setup, V2V network, device launch, PC launch).

### Mode summary

| | Mode 1 | Mode 2 | Mode 3 |
|---|---|---|---|
| **Where openpilot runs** | PC | PC (×2) | Comma 3X (×2) |
| **Lead vehicle** | MetaDrive IDM/scripted | MetaDrive + lead openpilot | MetaDrive + lead openpilot |
| **hCCC input** | Radar (camera → model) | V2V relay | V2V relay |
| **V2V** | Off | Local relay on PC | Device-to-device WiFi |
| **Terminals** | 2 | 4 | 2 SSH + 1 PC |
| **`HCC_V2V_ENABLED`** | `0` | `1` | `1` (set by device script) |
| **`HCC_V2V_ONLY`** | `0` | `1` | `1` (set by device script) |

---

## Quick Start (Mode 1)

For the simplest workflow, use two terminals from the repo root:

```bash
# Terminal 1 — start openpilot
./tools/sim/launch_openpilot_ego.sh

# Terminal 2 — start the MetaDrive bridge
uv run python tools/sim/run_bridge.py --mode hc3 --scn 48 --keyboard
```

Or use the helper script that opens both terminals:

```bash
./tools/sim/open_sim_terminals.sh
```

## Lightsail Relay Testing

In the dual-repo HC3 simulator flow, the bridge owns the scenario and the lead vehicle trajectory. The lead branch is not idle: it publishes the simulated lead vehicle state that the bridge is driving.

Use 4 terminals when testing against a remote Lightsail relay.

### Terminal 1: Lightsail log

```bash
ssh ubuntu@<LIGHTSAIL_STATIC_IP>
tail -f /var/log/hcc-v2v/hcc-v2v-relay.csv
```

### Terminal 2: ego

```bash
cd ~/openpilot-hcc
export HCC_V2V_ENABLED=1
export HCC_V2V_ONLY=1
export HCC_V2V_DEVICE_ID=ego-sim
export HCC_V2V_RELAY_HOST=<LIGHTSAIL_STATIC_IP>
export HCC_V2V_RELAY_PORT=19090
./tools/sim/launch_openpilot_ego.sh
```

### Terminal 3: lead

```bash
cd ~/openpilot-hcc-lead
export HCC_V2V_ENABLED=1
export HCC_V2V_DEVICE_ID=lead-sim
export HCC_V2V_RELAY_HOST=<LIGHTSAIL_STATIC_IP>
export HCC_V2V_RELAY_PORT=19090
./tools/sim/launch_openpilot_lead.sh
```

### Terminal 4: bridge with scenario

```bash
cd ~/openpilot-hcc/tools/sim
source ../../.venv/bin/activate
./run_bridge.py --mode hc3 --scn 48 --lead_prefix hcclead --keyboard
```

That bridge command is what tells the simulated lead car which scenario to follow.

Success looks like:

- the bridge prints a line similar to `Loaded lead trajectory speed profile for scenario 48 ...`
- the Lightsail relay log shows `registered_peer` for `ego-sim`
- the Lightsail relay log shows `registered_peer` for `lead-sim`
- the Lightsail relay log then shows repeated `forwarded`

Mental model:

- bridge = world + scenario + lead trajectory
- lead branch = publishes the lead vehicle's simulated state
- ego branch = receives forwarded V2V and runs HC3 in `V2V-only` mode

If the Lightsail CSV stays empty, check the instance firewall rule for `UDP 19090` first.

## Bridge Mode Presets

The `--mode` flag selects the scenario and map configuration:

- `default` — Standard simulator driving environment (loop map, no lead).
- `hc3` — HC3 straight-road preset with a lead vehicle (Modes 1 and 2).
- `hcc_hil` — Hardware-in-the-loop preset (Mode 3, requires `--lead` and `devices.toml`).

Examples:

```bash
./run_bridge.py --mode default --keyboard
./run_bridge.py --mode hc3 --scn 48 --keyboard
./run_bridge.py --mode hcc_hil --lead --devices_toml tools/sim/hil/devices.toml
```

## Control Input

The simulator supports Logitech wheel input, keyboard input, and generic joystick input.

Default behavior:

- the bridge attempts to use a Logitech wheel first
- if no supported wheel is available, it falls back to the keyboard

Force keyboard input:

```bash
./run_bridge.py --mode default --keyboard
```

Require Logitech wheel input:

```bash
./run_bridge.py --mode default --logitech_wheel
```

Specify a Linux input device explicitly:

```bash
./run_bridge.py --mode default --logitech_wheel --wheel_device /dev/input/event5
```

Use a generic joystick:

```bash
./run_bridge.py --mode default --joystick
```

List available input devices:

```bash
ls -l /dev/input/by-id
```

If Linux denies wheel access:

```bash
sudo usermod -aG input $USER
```

Log out and back in after updating group membership.

## Keyboard Controls

Keyboard input supports the following commands:

```text
| key  | action                 |
|------|------------------------|
| 1    | Cruise Resume / Accel  |
| 2    | Cruise Set / Decel     |
| 3    | Cruise Cancel          |
| w    | Throttle               |
| a    | Steer left             |
| s    | Brake                  |
| d    | Steer right            |
| z    | Left blinker           |
| x    | Right blinker          |
| i    | Toggle ignition        |
| r    | Reset simulation       |
| q    | Quit                   |
```

## Scenario Replay

Scenario replay uses a speed profile from `Scenarios.csv`.

Run a replay scenario:

```bash
./run_bridge.py --scn 48
```

Default CSV lookup path:

```text
tools/sim/lib/Scenarios.csv
```

Specify an alternate CSV path:

```bash
./run_bridge.py --scn 48 --scn_csv /path/to/Scenarios.csv
```

Notes:

- `--scn` must be greater than or equal to `1`
- replay requires a valid scenario CSV file

## Output Files

Replay runs automatically generate telemetry output.

Default behavior:

- output files are written under `tools/sim/data/`
- replay runs use names similar to `Scenarios.openpilot.scn48.<timestamp>.csv`
- non-replay runs use names similar to `metadrive_bridge_log.<timestamp>.csv`
- the output directory is created automatically when needed

Specify a custom CSV path:

```bash
./run_bridge.py --scn 48 --output_csv /path/to/output.csv
```

Generate a PNG speed plot:

```bash
./run_bridge.py --scn 48 --output_graph /path/to/output.png
```

Known-good SCN 48 replay outputs from the dual-repo HC3 setup are:

- `tools/sim/data/hccc/Test48.vehicle.honda_civic_2022_ICE.hccc.csv`
- `tools/sim/graphs/hccc/Test48.vehicle.honda_civic_2022_ICE.hccc.png`

## Additional Options

For simulator work, the default launch path uses the standard smaller UI window:

```bash
./tools/sim/launch_openpilot.sh
./tools/sim/open_sim_terminals.sh
```

If you want the larger openpilot UI layout instead:

```bash
BIG=1 ./tools/sim/launch_openpilot.sh
SIM_BIG=1 ./tools/sim/open_sim_terminals.sh
```

Enable additional bridge options:

- `--dual_camera`
  Publishes both road and wide-road camera streams.
- `--high_quality`
  Uses higher visual quality settings in MetaDrive.

Example:

```bash
./run_bridge.py --mode default --dual_camera --high_quality
```

Show all bridge options:

```bash
./run_bridge.py -h
```

## Architecture Overview

The simulator consists of two cooperating systems:

### openpilot runtime

`launch_openpilot.sh` starts openpilot with simulation-specific environment variables. This includes:

- `SIMULATION=1`
- `NOBOARD=1`
- `SKIP_FW_QUERY=1`
- `FINGERPRINT=HONDA_CIVIC_2022`

The script also blocks hardware-oriented processes such as the normal `camerad`, allowing the simulator to provide those interfaces instead.

### MetaDrive bridge runtime

`run_bridge.py` creates the simulator bridge and launches the MetaDrive-backed world. The bridge is responsible for:

- reading simulator state
- publishing simulated vehicle and sensor messages
- receiving openpilot outputs
- applying the resulting commands back to MetaDrive

## End-to-End Data Flow

The runtime loop operates as follows:

1. MetaDrive creates the map, ego vehicle, camera sensors, and optional lead vehicle.
2. The bridge reads ego state, lead state, lane/debug state, and rendered camera frames.
3. The bridge publishes simulated interfaces into openpilot.
4. openpilot processes the simulated data using its standard pipeline.
5. The bridge reads openpilot control outputs and applies them to the MetaDrive ego vehicle.
6. The simulator advances, and the cycle repeats.

## Camera Path

The camera path is one of the most representative parts of the simulation.

Flow:

1. MetaDrive renders RGB images from the ego-vehicle camera pose.
2. Frames are written into shared memory.
3. The simulator converts RGB frames to the NV12/YUV format expected by openpilot.
4. Frames are published through VisionIPC as simulated `camerad` output.
5. openpilot processes such as `modeld` consume the frames normally.

With `--dual_camera`, the bridge publishes both:

- road camera
- wide-road camera

## Vehicle Interface and CAN Path

There is no physical CAN bus in the simulator. Instead, the bridge synthesizes the subset of vehicle messages needed by openpilot.

Simulated CAN content includes:

- wheel speeds
- transmission speed
- cruise button presses
- steering angle
- steering torque from manual input
- blinker state
- brake pressed state

These messages allow openpilot's standard car interface to construct `carState` as if it were connected to a Honda Civic 2022.

## GPS, IMU, and Auxiliary Sensor Path

The simulator also publishes simplified non-CAN sensor streams, including:

- `gpsLocationExternal`
- `accelerometer`
- `gyroscope`
- `peripheralState`
- `driverStateV2`
- `driverMonitoringState`

These streams are sufficient to keep the broader software stack operational, but they are simplified relative to real hardware.

In particular:

- GPS is derived from planar simulator coordinates
- IMU behavior is simplified
- driver monitoring is intentionally synthesized as valid / attentive

## Radar and Lead-Tracking Model

This simulator does not implement a physics-based radar sensor model.

Instead:

1. MetaDrive maintains the lead vehicle directly.
2. The bridge computes relative lead measurements from world state.
3. Those measurements are published as `liveTracks`.
4. openpilot's normal `radard` process consumes `liveTracks` together with camera-model output.
5. `radard` produces `radarState` in the usual pipeline.

Accordingly, the radar path should be understood as a synthetic lead-object feed injected into openpilot's radar fusion pipeline, not a raw radar reflection simulation.

For hCCC debugging, one practical lesson from this branch is that the root issue was in the simulated radar trust path, not in the ego longitudinal controller itself. The synthetic `liveTracks` lead remained present in simulation, but it was not always being treated consistently as the authoritative lead by the `liveTracks` -> `radarState` path. That led to unstable `radar=True` / `radar=False` handoffs, and later `longcontrol` changes were largely reacting to that unstable input rather than causing the original issue.

## Lead Vehicle Behavior

The lead vehicle supports two primary operating modes.

### Standard lead-follow mode

In standard operation:

- a lead vehicle is spawned ahead of the ego vehicle
- the lead vehicle is kept lane-aware
- the bridge computes relative distance and velocity from the simulated world state

### Replay mode

In replay mode:

- a speed profile is loaded from `Scenarios.csv`
- the profile is converted into longitudinal position over time
- the lead vehicle follows the replayed scenario trajectory

This mode is useful for repeatable longitudinal evaluation.

## openpilot Processing Path

Once simulated inputs are published, openpilot runs through its normal process graph.

- `card` parses simulated CAN and publishes `carState`
- `modeld` consumes simulated camera streams
- `radard` fuses `liveTracks`, `modelV2`, and `carState`
- the planner computes longitudinal targets
- `controlsd` publishes `carControl`

The bridge then reads `carControl` and converts it into MetaDrive steering and longitudinal commands.

## Current Branch Behavior

This branch is configured primarily for longitudinal experimentation.

Practical implications:

- longitudinal control is active
- lead-follow behavior is central to the simulation workflow
- steering is not configured here as a full openpilot lateral-control demonstration path
- manual steering input remains relevant

## Fidelity and Limitations

The simulator is strongest as a software-in-the-loop integration environment.

Representative components:

- openpilot process pipeline
- camera-to-model path
- lead-follow and planner behavior
- scenario replay workflow

Simplified components:

- CAN synthesis
- panda state
- GPS conversion
- IMU behavior
- driver monitoring
- radar sensing physics

This simulator should therefore be viewed as a practical integration and behavior-testing tool, not a full sensor-accurate vehicle dynamics and perception simulator.

## Troubleshooting

### Ego is engaged but never accelerates (`accelCmd=0.00`)

This almost always means the V2V settings are wrong for the mode you are
running. In Mode 1 (single-device sim), V2V must be **disabled**:

```bash
# Correct for Mode 1 — these are the defaults in launch_openpilot_ego.sh
HCC_V2V_ENABLED=0 HCC_V2V_ONLY=0 ./tools/sim/launch_openpilot_ego.sh
```

If `HCC_V2V_ENABLED=1` is set without a relay running, the hCCC controller
creates a V2V subscriber, sees `v2v_lead.status=False`, and returns `None`
before it ever checks the radar lead. The car shows `Engaged: True` but
`accelCmd` stays at zero.

For Mode 2 (dual-device sim), V2V must be **enabled** and the relay must be
running before the ego openpilot starts:

```bash
# Start relay first, then:
HCC_V2V_ENABLED=1 HCC_V2V_ONLY=1 ./tools/sim/launch_openpilot_ego.sh
```

### openpilot starts but the simulator does not

Ensure `./run_bridge.py` is running in a second terminal.

### Nothing starts

Ensure the virtual environment is activated in both terminals:

```bash
source .venv/bin/activate
```

### Wheel input does not work

Check the following:

- run with `--keyboard` to confirm the bridge is functioning
- run with `--logitech_wheel` to require wheel-only behavior
- inspect `/dev/input/by-id`
- add your user to the `input` group if Linux blocks device access

### Scenario replay fails

Confirm that:

- the `--scn` index is valid
- the scenario CSV file exists
- the CSV file is located at `tools/sim/lib/Scenarios.csv` or passed through `--scn_csv`

### `./tools/hcc_v2v/launch_ui.sh` fails with `No module named 'tkinter'`

Install the Ubuntu Tk package:

```bash
sudo apt update
sudo apt install -y python3-tk
```

Then restart the shell and relaunch the UI.

### `git worktree add ../openpilot-hcc-lead hcc-lead` asks for GitLab and fails on an LFS object

That is usually Git LFS during checkout, not a problem with `git worktree` itself.

Use:

```bash
cd ~/ethanmathias/openpilot-hcc
rm -rf ../openpilot-hcc-lead
GIT_LFS_SKIP_SMUDGE=1 git worktree add ../openpilot-hcc-lead hcc-lead
```

This skips downloading LFS-backed assets during checkout so the lead worktree can still be created.

### `scons -u -j$(nproc)` fails in `tools/cabana` on Ubuntu

If the errors mention APIs such as `horizontalAdvance`, `QRandomGenerator`, `readAllFrames`, or `invokeMethod`, check the include paths. If they point at Anaconda, for example `/home/<user>/anaconda3/include/qt`, then `cabana` is building against an older Qt version than the code expects.

This is separate from the V2V HC3 control path. The sim stack can still be fine even when `cabana` fails.

Recommended clean-shell rebuild:

```bash
deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
unset CONDA_PREFIX CONDA_DEFAULT_ENV _CE_CONDA _CE_M
unset CPATH CPLUS_INCLUDE_PATH LIBRARY_PATH LD_LIBRARY_PATH QT_PLUGIN_PATH QTDIR
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/.local/bin"

cd ~/ethanmathias/openpilot-hcc-lead
source .venv/bin/activate
which qmake
qmake -v
scons -u -j$(nproc)
```

`qmake -v` should resolve to a system Qt installation, not an Anaconda path.

If the system Qt development packages are missing, install them:

```bash
sudo apt update
sudo apt install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5serialbus5-dev
```

### `scons` fails with `ModuleNotFoundError: No module named 'opendbc'`

This usually means the build started from an incomplete or contaminated environment before the repo setup was fully in place. After submodules are installed, rerun the build from a clean activated repo virtualenv:

```bash
cd ~/ethanmathias/openpilot-hcc-lead
source .venv/bin/activate
scons -u -j$(nproc)
```

If the remaining failures are only in `tools/cabana`, focus on fixing the Qt environment issue above.
