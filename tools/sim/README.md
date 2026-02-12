openpilot in simulator
=====================

openpilot implements a [bridge](run_bridge.py) that allows it to run in the [MetaDrive simulator](https://github.com/metadriverse/metadrive).

## Launching openpilot
First, start openpilot.
``` bash
# Run locally
./tools/sim/launch_openpilot.sh
to launch in big screen on pc run BIG=1 ./tools/sim/launch_openpilot.sh
```
note: run the openpilot and bridge in two seperate terminal windows.

## Bridge usage
```
$ ./run_bridge.py -h
usage: run_bridge.py [-h] [--joystick] [--high_quality] [--dual_camera]
Bridge between the simulator and openpilot.

options:
  -h, --help            show this help message and exit
  --joystick
  --high_quality
  --dual_camera
  --scenario {default,hccc_step}
  --enable_hcc

Notes:
If you see warning: EnableHCCC param key is unavailable ... --enable_hcc ignored, then HCC was not enabled.
To truly enable via flag, rebuild on Ubuntu so params key table includes EnableHCCC.
From repo root on Ubuntu:

rg -n "EnableHCCC" common/params_keys.h
scons -u -j"$(nproc)"
Then rerun:

cd tools/sim
./run_bridge.py --enable_hcc
```

### hCCC step scenario
Run a lead profile that ramps up and down in speed for acceleration/deceleration testing:
```
./run_bridge.py --scenario hccc_step
```

#### Bridge Controls:
- To engage openpilot press 2, then press 1 to increase the speed and 2 to decrease.
- To disengage, press "S" (simulates a user brake)

#### All inputs:

```
| key  |   functionality       |
|------|-----------------------|
|  1   | Cruise Resume / Accel |
|  2   | Cruise Set    / Decel |
|  3   | Cruise Cancel         |
|  r   | Reset Simulation      |
|  i   | Toggle Ignition       |
|  q   | Exit all              |
| wasd | Control manually      |
```

## MetaDrive

### Launching Metadrive
Start bridge processes located in tools/sim:
``` bash
./run_bridge.py
```
