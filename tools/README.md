# openpilot tools

## System Requirements

openpilot is developed and tested on **Ubuntu 24.04**, which is the primary development target aside from the [supported embedded hardware](https://github.com/commaai/openpilot#running-on-a-dedicated-device-in-a-car).

Most of openpilot should work natively on macOS. On Windows you can use WSL for a nearly native Ubuntu experience. Running natively on any other system is not currently recommended and will likely require modifications.

## Native setup on Ubuntu 24.04 and macOS

Follow these instructions for a fully managed setup experience. If you'd like to manage the dependencies yourself, just read the setup scripts in this directory.

**1. Clone openpilot**
``` bash
git clone https://github.com/commaai/openpilot.git
```

**2. Run the setup script**
``` bash
cd openpilot
tools/op.sh setup
```

**3. Activate a Python shell**
Activate a shell with the Python dependencies installed:
``` bash
source .venv/bin/activate
```

**4. Build openpilot**
``` bash

export PATH=/usr/bin:/usr/lib/x86_64-linux-gnu/qt5/bin:$PATH (this is only for the dell pc since it defaults to old Qt)
scons -u -j$(nproc)
```

## WSL on Windows

[Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/about) should provide a similar experience to native Ubuntu. [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/compare-versions) specifically has been reported by several users to be a seamless experience.

Follow [these instructions](https://docs.microsoft.com/en-us/windows/wsl/install) to setup the WSL and install the `Ubuntu-24.04` distribution. Once your Ubuntu WSL environment is setup, follow the Linux setup instructions to finish setting up your environment. See [these instructions](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) for running GUI apps.

**NOTE**: If you are running WSL and any GUIs are failing (segfaulting or other strange issues) even after following the steps above, you may need to enable software rendering with `LIBGL_ALWAYS_SOFTWARE=1`, e.g. `LIBGL_ALWAYS_SOFTWARE=1 selfdrive/ui/ui`.

## CTF
Learn about the openpilot ecosystem and tools by playing our [CTF](/tools/CTF.md).

## Simulator Quickstart
Use the MetaDrive bridge from `tools/sim` to run openpilot on a PC.

From the repo root, use two terminals:

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

Useful variants:
- `./run_bridge.py --mode hc3`
- `./run_bridge.py --mode hc3 --logitech_wheel`
- `./run_bridge.py --mode hc3 --logitech_wheel --wheel_device /dev/input/by-id/<wheel>-event-joystick`

See `tools/sim/README.md` for full simulator usage and controls.

## Directory Structure

```
├── cabana/             # View and plot CAN messages from drives or in realtime
├── camerastream/       # Cameras stream over the network
├── joystick/           # Control your car with a joystick
├── lib/                # Libraries to support the tools and reading openpilot logs
├── plotjuggler/        # A tool to plot openpilot logs
├── replay/             # Replay drives and mock openpilot services
├── scripts/            # Miscellaneous scripts
├── serial/             # Tools for using the comma serial
├── sim/                # Run openpilot in a simulator
└── webcam/             # Run openpilot on a PC with webcams
```

## Setup Commands
```bash
tools/op.sh setup
source .venv/bin/activate
scons -u -j$(nproc)

if error related to front_tire.gltf, then go through .venv directory to metadrive/.../assets and replace with assets folder from github repository. Make sure to patch the "check for updates" so that it doesn't auto patch and delete the assets again.

for steering wheel, may need to install evdev in the venv
```
