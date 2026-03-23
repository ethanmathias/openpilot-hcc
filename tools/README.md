# openpilot Tools

This directory contains local development utilities, setup scripts, simulator entry points, and supporting tooling for working with the repository on a development machine.

For device installation instructions, see [README.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/README.md). For simulator-specific documentation, see [tools/sim/README.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/tools/sim/README.md).

## Supported Development Environments

The primary development target is **Ubuntu 24.04**.

Additional notes:

- macOS is supported for much of the local development workflow
- Windows is best used through WSL with Ubuntu 24.04
- native development on other operating systems is not recommended and may require local modifications

## Managed Local Setup

The repository includes a managed setup path through `tools/op.sh`.

### 1. Clone the repository

```bash
git clone https://github.com/ethanmathias/openpilot-hcc.git
cd openpilot-hcc
```

### 2. Run setup

```bash
tools/op.sh setup
```

### 3. Activate the virtual environment

```bash
source .venv/bin/activate
```

### 4. Build the project

```bash
scons -u -j$(nproc)
```

If your machine defaults to an older Qt installation, prepend the newer Qt path before building. A local example used in this environment is:

```bash
export PATH=/usr/bin:/usr/lib/x86_64-linux-gnu/qt5/bin:$PATH
scons -u -j$(nproc)
```

## WSL on Windows

[Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/about) can provide a development experience close to native Ubuntu.

Recommended setup:

1. Install WSL.
2. Install the `Ubuntu-24.04` distribution.
3. Follow the Linux setup steps above from within WSL.

For GUI applications under WSL, see Microsoft's documentation for [Linux GUI apps on WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

If GUI applications fail under WSL, software rendering may help:

```bash
LIBGL_ALWAYS_SOFTWARE=1 selfdrive/ui/ui
```

## Simulator

The MetaDrive simulator is located in `tools/sim`.

Basic startup:

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

Common variants:

```bash
./run_bridge.py --mode hc3
./run_bridge.py --mode hc3 --logitech_wheel
./run_bridge.py --mode hc3 --logitech_wheel --wheel_device /dev/input/by-id/<wheel>-event-joystick
```

For architecture, controls, replay workflow, and limitations, see [tools/sim/README.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/tools/sim/README.md).

## CTF

To explore the openpilot ecosystem and tooling through guided exercises, see [tools/CTF.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/tools/CTF.md).

## Directory Overview

```text
├── cabana/             View and plot CAN messages from logs or live streams
├── camerastream/       Stream cameras over the network
├── joystick/           Control a vehicle with a joystick
├── lib/                Shared libraries used by tools and log readers
├── plotjuggler/        Plot openpilot logs
├── replay/             Replay drives and mock openpilot services
├── scripts/            Miscellaneous scripts
├── serial/             Serial-related tooling for comma hardware
├── sim/                Run openpilot in the MetaDrive simulator
└── webcam/             Run openpilot on a PC with webcams
```

## Notes and Known Issues

### MetaDrive asset issues

If setup or runtime errors reference MetaDrive assets such as `front_tire.gltf`, inspect the MetaDrive installation inside the virtual environment.

In some local environments, replacing the installed MetaDrive `assets/` directory with the upstream repository assets resolves the issue. If you use this workaround, ensure any automatic update path does not immediately overwrite the replacement assets.

### Steering wheel dependencies

Some wheel setups may require `evdev` inside the virtual environment:

```bash
pip install evdev
```
