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
- `--logitech_wheel`: use Logitech wheel/pedals input.
- `--wheel_device /dev/input/eventX`: explicit input device path.
- `--wheel_hz N`: wheel command publish rate (default `100`).
- `--joystick`: generic joystick input instead of keyboard.
- `--dual_camera`: publish wide + road camera.
- `--high_quality`: use higher visual quality settings.

## Modes
- `default`: standard driving map.
- `hc3`: hC3 preset (internally mapped to the straight-road hCCC scenario).

Examples:
```bash
./run_bridge.py --mode default
./run_bridge.py --mode hc3
```

## Logitech Wheel (G920/G29/G923)
If autodetection works:
```bash
./run_bridge.py --mode hc3 --logitech_wheel
```

If you want to force a specific device:
```bash
./run_bridge.py --mode hc3 --logitech_wheel --wheel_device /dev/input/by-id/<your-wheel>-event-joystick
```

Find input devices:
```bash
ls -l /dev/input/by-id
```

On Ubuntu, if wheel access is denied, add your user to input group and re-login:
```bash
sudo usermod -aG input $USER
```

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
