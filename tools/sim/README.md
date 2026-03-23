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
