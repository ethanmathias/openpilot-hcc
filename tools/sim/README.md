openpilot Simulator (MetaDrive)
===============================

This folder contains the simulator setup for running openpilot with [MetaDrive](https://github.com/metadriverse/metadrive).

The goal of this simulator is simple:

- `openpilot` runs like it would in a car.
- `MetaDrive` creates the virtual road and vehicles.
- The simulator bridge connects the two so openpilot can "drive" inside the virtual world.

If you are new to this folder, the most important thing to know is that you usually run **two programs at the same time**:

1. `openpilot`
2. the simulator bridge

## What Each Script Does

These are the files you will use most often:

- `./tools/sim/launch_openpilot.sh`
  Starts openpilot in simulation mode.
- `./tools/sim/run_bridge.py`
  Starts the MetaDrive simulator and sends control/state data between MetaDrive and openpilot.
- `./tools/sim/open_sim_terminals.sh`
  Convenience script that opens two terminals for you.

## Before You Start

Start from the repository root:

```bash
cd /path/to/openpilot-hcc
```

Make sure the Python virtual environment already exists:

```bash
ls .venv/bin/activate
```

If that file is missing, the simulator is not set up yet. This repo includes a hint in `open_sim_terminals.sh` that setup may be done with:

```bash
tools/op.sh setup
```

## Easiest Way

If you want the easiest workflow, run:

```bash
./tools/sim/open_sim_terminals.sh
```

What this does:

- opens one terminal that starts openpilot
- opens a second terminal in `tools/sim`
- activates the repo virtual environment for both terminals

After that, use the second terminal to start the simulator bridge with the command you want, for example:

```bash
./run_bridge.py --mode default
```

## Manual Start

If you prefer to start everything yourself, use two terminals.

### Terminal 1: Start openpilot

From the repo root:

```bash
source .venv/bin/activate
./tools/sim/launch_openpilot.sh
```

What this does:

- activates the repo Python environment
- turns on simulation-related environment variables
- starts the openpilot manager process

Leave this terminal running.

### Terminal 2: Start the simulator bridge

From the repo root:

```bash
source .venv/bin/activate
cd tools/sim
./run_bridge.py --mode default
```

What this does:

- starts MetaDrive
- creates the bridge between openpilot and the simulator
- listens for keyboard, wheel, or joystick input

Leave this terminal running too.

## Picking a Driving Mode

The bridge supports two mode presets:

- `default`
  Regular simulator driving.
- `hc3`
  Uses the hC3 straight-road scenario preset.

Examples:

```bash
./run_bridge.py --mode default
./run_bridge.py --mode hc3
```

## Choosing How You Control the Car

The simulator can take input from a Logitech wheel, a keyboard, or a joystick.

### Default behavior

If you do not pass any input flags, the bridge tries to use a Logitech wheel first.

If a supported wheel is not found, it falls back to the keyboard.

### Force keyboard input

Use this if you want keyboard control even when a wheel is plugged in:

```bash
./run_bridge.py --mode default --keyboard
```

### Force Logitech wheel input

Use this if you want the bridge to require a Logitech wheel:

```bash
./run_bridge.py --mode default --logitech_wheel
```

If you know the exact Linux input device:

```bash
./run_bridge.py --mode default --logitech_wheel --wheel_device /dev/input/event5
```

To see available input devices:

```bash
ls -l /dev/input/by-id
```

If Linux says you do not have permission to use the wheel:

```bash
sudo usermod -aG input $USER
```

Then log out and log back in before trying again.

### Use a generic joystick

```bash
./run_bridge.py --mode default --joystick
```

## Keyboard Controls

If you are using the keyboard, these keys are available:

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
| q    | Quit everything        |
```

## Scenario Replay

The bridge can replay a scenario from a `Scenarios.csv` file.

Use `--scn` to choose the scenario column number:

```bash
./run_bridge.py --scn 48
```

By default, the bridge looks for the CSV file in:

```text
tools/sim/lib/Scenarios.csv
```

If your CSV is somewhere else, pass it directly:

```bash
./run_bridge.py --scn 48 --scn_csv /path/to/Scenarios.csv
```

Important notes:

- `--scn` must be `1` or greater
- if the CSV file cannot be found, the bridge will stop with an error

## Output Files

When you replay a scenario with `--scn`, the bridge automatically saves telemetry output.

Default behavior:

- output files are written to `tools/sim/data/`
- replay runs use names like `Scenarios.openpilot.scn48.<timestamp>.csv`
- normal runs use names like `metadrive_bridge_log.<timestamp>.csv`
- the `tools/sim/data/` folder is created automatically if needed

If you want a custom CSV path:

```bash
./run_bridge.py --scn 48 --output_csv /path/to/output.csv
```

You can also save a speed graph as a PNG:

```bash
./run_bridge.py --scn 48 --output_graph /path/to/output.png
```

## Optional Visual Settings

### Bigger openpilot UI

If you want the openpilot UI to appear larger:

```bash
BIG=1 ./tools/sim/launch_openpilot.sh
```

### Extra bridge options

You can also enable:

- `--dual_camera`
  Publishes both wide and road camera streams.
- `--high_quality`
  Uses higher visual quality settings in the simulator.

Example:

```bash
./run_bridge.py --mode default --dual_camera --high_quality
```

## Helpful Commands

Show all bridge options:

```bash
./run_bridge.py -h
```

Start the standard simulator flow:

```bash
./run_bridge.py --mode default
```

Start the hC3 preset:

```bash
./run_bridge.py --mode hc3
```

Run a replay scenario and save output automatically:

```bash
./run_bridge.py --scn 48
```

## Troubleshooting

### Nothing starts

Make sure both terminals have activated the virtual environment:

```bash
source .venv/bin/activate
```

### openpilot starts, but there is no simulator

Make sure you also ran `./run_bridge.py` in a second terminal.

### Wheel does not work

Try one of these:

- run with `--keyboard` to confirm the bridge itself is working
- run with `--logitech_wheel` if you want wheel-only behavior
- check `/dev/input/by-id`
- add your user to the `input` group if Linux blocks device access

### Scenario replay fails

Make sure:

- you passed a valid `--scn` number
- the CSV file exists
- the CSV path is either `tools/sim/lib/Scenarios.csv` or provided with `--scn_csv`

## How the Simulator Works

This section explains the simulator from the beginning.

The short version is:

1. MetaDrive creates the virtual world.
2. The simulator bridge turns that world into fake car data and fake sensor data.
3. openpilot reads that data as if it came from a real vehicle.
4. openpilot sends driving commands back.
5. The simulator applies those commands to the virtual car.

So the full loop is:

```text
MetaDrive world -> simulated sensors / simulated CAN -> openpilot -> control commands -> MetaDrive world
```

## The Two Sides of the Simulator

When you run the simulator, you are really starting two systems:

### 1. openpilot

This is started by:

```bash
./tools/sim/launch_openpilot.sh
```

That script starts openpilot in simulation mode.

Important things it does:

- sets `SIMULATION=1`
- skips real hardware board requirements
- uses a fake Honda Civic 2022 fingerprint
- blocks real hardware daemons like the normal `camerad`

This lets the simulator provide fake hardware data instead.

### 2. The simulator bridge

This is started by:

```bash
cd tools/sim
./run_bridge.py --mode default
```

The bridge is the glue between MetaDrive and openpilot.

Its job is to:

- start the MetaDrive world
- read the virtual car state
- send fake sensor data into openpilot
- read openpilot outputs
- apply those outputs back to the simulator

## Step-by-Step Data Flow

Here is the simulator flow in plain language.

### Step 1: MetaDrive creates the world

MetaDrive creates:

- the road map
- the ego car
- camera views attached to the ego car
- an optional lead vehicle

Depending on the mode, the road is either:

- a looped driving map
- a long straight road for lead-follow and replay scenarios

### Step 2: The bridge reads the world state

The bridge continuously reads:

- ego vehicle speed
- ego vehicle position
- ego heading
- steering angle
- lead vehicle position and speed
- lane-related debug information
- rendered camera images

This information becomes the raw source for the fake sensor system.

### Step 3: The bridge publishes fake car and sensor data

The bridge sends several kinds of messages into openpilot.

These include:

- fake CAN messages
- fake panda state
- fake GPS
- fake accelerometer and gyroscope messages
- fake driver monitoring state
- camera frames
- synthetic radar-like lead tracks

From openpilot's point of view, these look like normal incoming vehicle and sensor messages.

### Step 4: openpilot runs normally

Once openpilot receives those messages, its normal processes run.

That includes things like:

- `card`
- `modeld`
- `radard`
- `plannerd`
- `controlsd`

These processes do the usual openpilot work:

- parse car state
- understand the road from camera images
- estimate lead vehicles
- plan speed
- generate control commands

### Step 5: The bridge reads openpilot outputs

The bridge reads messages such as:

- `carControl`
- `selfdriveState`
- `carState`
- `radarState`

It then converts the control outputs into simulator commands.

### Step 6: The simulator updates the virtual car

The final throttle, brake, and steering commands are applied to the MetaDrive ego car.

That changes the virtual world state.

Then the whole loop repeats.

## How the Camera Path Works

The camera path is one of the most important parts of the simulator.

Here is what happens:

1. MetaDrive renders an RGB image from the ego vehicle's camera position.
2. That image is copied into shared memory.
3. The simulator converts the RGB image into the NV12/YUV format openpilot expects.
4. The image is published through VisionIPC as if it came from `camerad`.
5. openpilot processes like `modeld` consume it normally.

That means the vision stack is still doing real work on simulated camera frames.

If `--dual_camera` is enabled, the simulator publishes both:

- road camera
- wide road camera

## How CAN and Vehicle State Work

openpilot normally learns about the car through CAN messages.

In this simulator, there is no real CAN bus, so the bridge creates fake CAN traffic instead.

The fake CAN includes things like:

- wheel speeds
- transmission speed
- cruise button presses
- steering angle
- steering torque from manual input
- blinker state
- brake pressed state

Those messages are enough for openpilot's car interface to build a normal `carState`.

In other words:

- MetaDrive provides the raw virtual motion
- the simulator converts that into fake CAN
- openpilot reads the CAN and believes it is talking to a Honda Civic 2022

## How GPS, IMU, and Other Sensors Work

The simulator also sends fake non-CAN sensor messages.

These include:

- `gpsLocationExternal`
- `accelerometer`
- `gyroscope`
- `peripheralState`
- `driverStateV2`
- `driverMonitoringState`

These are useful for keeping openpilot happy and allowing the rest of the software stack to run.

Important note:

- the GPS is generated from simple XY world coordinates
- the IMU path is much simpler than real hardware
- driver monitoring is intentionally faked as valid / attentive

So these parts are functional, but not as physically realistic as the vision or lead-vehicle parts.

## How Radar Works in This Simulator

This is the part many people expect to be more "physical" than it actually is.

There is **no real radar sensor simulation** here.

Instead, the simulator does this:

1. MetaDrive keeps track of the lead vehicle directly.
2. The simulator computes the lead vehicle's relative position and velocity from the world state.
3. The bridge publishes that information as `liveTracks`.
4. openpilot's normal `radard` process consumes those tracks.
5. `radard` combines the synthetic tracks with camera model output and produces `radarState`.

So radar in this simulator is better thought of as:

- a synthetic lead-object feed
- inserted into openpilot's radar pipeline

This is still useful, because openpilot's normal radar fusion and lead-selection logic still runs.

But it is not simulating real radar reflections, noise, or detection physics.

## How the Lead Vehicle Works

The lead vehicle can run in two main ways.

### Normal lead mode

In normal lead-follow mode:

- the simulator spawns a lead vehicle ahead of the ego car
- the lead vehicle follows lane-aware behavior
- the lead vehicle's motion is used to compute relative lead measurements

### Replay mode

In replay mode:

- the simulator loads a speed profile from `Scenarios.csv`
- it converts that profile into position over time
- it moves the lead vehicle according to that profile
- the ego car can then follow the replayed lead

This is useful for repeatable longitudinal testing.

## How openpilot Uses the Simulated Inputs

Once the simulator publishes all of its fake inputs, openpilot behaves mostly like normal.

### Vision

`modeld` reads the simulated road camera frames and estimates:

- lane geometry
- road shape
- lead vehicle hypotheses
- future motion predictions

### Radar / lead fusion

`radard` reads:

- `liveTracks`
- `modelV2`
- `carState`

It then creates `radarState`, including lead objects like `leadOne`.

### Longitudinal planning

The planner uses:

- lead information
- model predictions
- cruise targets

to decide the desired acceleration and whether the car should stop.

### Control output

`controlsd` turns planning outputs into:

- acceleration commands
- steering-related commands

Those are published in `carControl`.

The bridge then reads `carControl` and sends the final commands back into MetaDrive.

## Important Behavior in This Branch

This branch is set up mainly for longitudinal / lead-follow work.

That means:

- openpilot longitudinal control is active
- steering is not being fully controlled by openpilot in the normal lane-centering way
- manual steering input is still important

So if you are expecting full hands-off lateral driving in this branch, that is not the main design here.

This simulator is especially useful for:

- hCCC testing
- lead-follow experiments
- replaying traffic scenarios
- comparing speed and spacing behavior

## What Is Realistic vs Simplified

### More realistic parts

- the openpilot process pipeline
- the camera-to-model path
- the planner and control logic
- the lead-follow loop

### More simplified parts

- CAN generation
- panda state
- GPS conversion
- IMU behavior
- driver monitoring
- radar sensing physics

So this simulator is best understood as:

- **strong for software-in-the-loop testing**
- **not a full physical sensor-accurate vehicle simulator**

## A Simple Mental Model

If you want one sentence to remember:

> MetaDrive provides the world, the bridge pretends that world is real car hardware, and openpilot drives inside it.
