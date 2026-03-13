openpilot Simulator (MetaDrive)
===============================

This folder contains the simulator bridge that connects openpilot to [MetaDrive](https://github.com/metadriverse/metadrive).

## Quick Start
Use two terminals from the repo root.

Terminal 1:
```bash
source .venv/bin/activate
./tools/sim/launch_openpilot.sh
```

Terminal 2:
```bash
source .venv/bin/activate
cd tools/sim
./run_bridge.py --mode default
```

If a Logitech wheel is connected, the bridge now prefers it by default and falls back to keyboard if auto-detect fails.

Optional big-screen UI:
```bash
BIG=1 ./tools/sim/launch_openpilot.sh
```

## Bridge CLI
```bash
./run_bridge.py -h
```

Main options:
- `--mode {default,hc3}`: scenario preset.
- `--keyboard`: force keyboard input and skip Logitech wheel autodetect.
- `--logitech_wheel`: explicitly require Logitech wheel/pedals input.
- `--wheel_device /dev/input/eventX`: explicit input device path.
- `--wheel_hz N`: wheel command publish rate (default `100`).
- `--joystick`: generic joystick input instead of keyboard.
- `--dual_camera`: publish wide + road camera.
- `--high_quality`: use higher visual quality settings.
- `--scn N`: replay scenario column `N` from `Scenarios.csv`.
- `--output_csv PATH`: override the output telemetry CSV path.

## Modes
- `default`: standard driving map.
- `hc3`: hC3 preset (internally mapped to the straight-road hCCC scenario).

Examples:
```bash
./run_bridge.py --mode default
./run_bridge.py --mode hc3
```

Scenario replay with automatic output CSV:
```bash
./run_bridge.py --scn 48
```

## Logitech Wheel (G920/G29/G923)
Wheel input is now the default when available:
```bash
./run_bridge.py --mode hc3
```

If you want to force a specific device:
```bash
./run_bridge.py --mode hc3 --logitech_wheel --wheel_device /dev/input/by-id/<your-wheel>-event-joystick
```

If you want keyboard input even when a wheel is connected:
```bash
./run_bridge.py --mode hc3 --keyboard
```

Find input devices:
```bash
ls -l /dev/input/by-id
```

On Ubuntu, if wheel access is denied, add your user to input group and re-login:
```bash
sudo usermod -aG input $USER
```

## Scenario CSV Output
When replaying a scenario with `--scn`, the bridge writes a telemetry CSV automatically even if `--output_csv` is not provided.

Default behavior:
- The bridge writes output to `tools/sim/data/`.
- Replay runs use a timestamped name such as `Scenarios.openpilot.scn48.<timestamp>.csv`.
- Non-replay runs use `metadrive_bridge_log.<timestamp>.csv`.
- The `tools/sim/data` directory is created automatically if it does not exist.

Use `--output_csv /path/to/file.csv` only when you want a fixed location or filename.

## Keyboard Controls
```text
| key  | functionality         |
|------|-----------------------|
| 1    | Cruise Resume / Accel |
| 2    | Cruise Set / Decel    |
| 3    | Cruise Cancel         |
| r    | Reset simulation      |
| i    | Toggle ignition       |
| q    | Exit all              |
| wasd | Manual control        |
```
