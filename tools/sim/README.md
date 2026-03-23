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
- `./tools/sim/run_bridge.py`
  Starts the MetaDrive bridge process and input handling.
- `./tools/sim/open_sim_terminals.sh`
  Opens two prepared terminals for the standard workflow.

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

## Quick Start

For the standard workflow, use:

```bash
./tools/sim/open_sim_terminals.sh
```

This opens:

- one terminal running openpilot in simulation mode
- one terminal prepared to launch the simulator bridge

From the second terminal, start the bridge:

```bash
./run_bridge.py --mode default
```

## Manual Startup

If you prefer to launch each process directly, use two terminals.

### Terminal 1: openpilot

```bash
source .venv/bin/activate
./tools/sim/launch_openpilot.sh
```

This script:

- activates the repository virtual environment
- enables simulation-related environment variables
- starts the openpilot manager process

### Terminal 2: simulator bridge

```bash
source .venv/bin/activate
cd tools/sim
./run_bridge.py --mode default
```

This process:

- starts the MetaDrive simulator
- starts the bridge between MetaDrive and openpilot
- handles keyboard, wheel, or joystick input

## Modes

The bridge supports the following mode presets:

- `default`
  Standard simulator driving environment.
- `hc3`
  BeamNG-parity hC3 straight-road preset. This mode matches the legacy BeamNG HC3 controller on controller math and 10 Hz update cadence, while using a MetaDrive-calibrated throttle/brake adapter for smoother physical behavior in the simulator.

Examples:

```bash
./run_bridge.py --mode default
./run_bridge.py --mode hc3
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
- replay with `--mode hc3` or `--scn` is the primary BeamNG-parity verification path

## Output Files

Replay runs automatically generate telemetry output.

Default behavior:

- replay outputs are written under `tools/sim/data/<control_method>/`
- replay graphs are written under `tools/sim/graphs/<control_method>/`
- HC3 replay runs use BeamNG-style names such as `Test1.vehicle.honda_civic_2022_ICE.scn48.hccc.csv`
- the output directories are created automatically when needed
- replay roads are auto-sized from the loaded scenario distance plus a 1000 m buffer, with a 2000 m minimum

Specify a custom CSV path:

```bash
./run_bridge.py --scn 48 --output_csv /path/to/output.csv
```

Generate a PNG speed plot:

```bash
./run_bridge.py --scn 48 --output_graph /path/to/output.png
```

The replay CSV includes BeamNG comparison signals such as:

- target lead speed from `Scenarios.csv`
- ego and lead speed
- sim-derived ego acceleration
- `carState.aEgo`
- planner `aTarget`
- HC3 contribution
- manual contribution
- final combined command
- headway and delta-v
- ego and lead lane diagnostics
- lead pose replay status and fallback state
- driver gas/brake and final throttle/brake sent to MetaDrive

## Additional Options

Launch the openpilot UI in a larger layout:

```bash
BIG=1 ./tools/sim/launch_openpilot.sh
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

In `hc3` mode, the bridge keeps BeamNG-style HC3 command generation but calibrates the final MetaDrive pedal application:

- the HC3 controller updates internally at `0.1 s`
- manual cooperative input in simulation is raw `gas - brake`
- positive combined command maps to throttle using `/1.6` scaling
- negative combined command maps to brake using `/4.0` scaling
- a `0.05` command deadband suppresses tiny oscillatory pedal inputs
- pedal outputs are slew-limited before they reach MetaDrive

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
