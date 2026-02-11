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
usage: run_bridge.py [-h] [--joystick] [--high_quality] [--dual_camera] [--scenario {default,hccc_step}] [--manual_no_lead]
Bridge between the simulator and openpilot.

options:
  -h, --help            show this help message and exit
  --joystick
  --high_quality
  --dual_camera
  --scenario {default,hccc_step}
  --manual_no_lead
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

### hCCC test scenario
To run a deterministic lead-vehicle profile for hCCC testing:
``` bash
./run_bridge.py --scenario hccc_step
```
This scenario injects a virtual lead into `liveTracks` (used by `radard`) with a speed step profile:
- starts at ~22 m/s
- slows to ~10 m/s
- then recovers back to ~22 m/s

### Manual no-lead driving (WASD only)
To drive manually without auto-engage and without any virtual lead:
``` bash
./run_bridge.py --scenario default --manual_no_lead
```
Use `W/A/S/D` keys to drive.
