# tools/hcc

Everything needed to run a human-in-the-loop cooperative cruise control (hCCC)
field test: the V2V publishers, the on-device tooling, the run orchestrator and
the analysis.

Full project documentation lives in the Notion **openpilot Research Log**, not
here. This file is the command reference.

## Configure once

[`config/devices.toml`](config/devices.toml) holds every host, user, repo path,
port and device id. Set it once and no command needs flags. It is version
controlled on purpose: the addresses *are* the test setup.

## Run a test

From the laptop, joined to the ego's hotspot:

```bash
./tools/hcc/hcc preflight          # are both devices ready?
./tools/hcc/hcc find-lead          # what can the ego's hotspot see?
./tools/hcc/hcc run                # start a logged run; Ctrl-C ends and collects
./tools/hcc/hcc run --pedal        # ... with the pedal rig as the manual input
./tools/hcc/hcc run --bench        # ... with no car at all
./tools/hcc/hcc abort              # kill the remote processes, collect nothing
./tools/hcc/hcc collect <run_id>   # re-pull a run that ended badly
./tools/hcc/hcc plot <run_id>      # link-health statistics and plots
```

Everything a run produces lands in one folder under `runs/<run_id>/`:
`relay.csv`, `sent.csv`, `pedal_sent.csv`, `ego_monitor.csv`, `metadata.json`,
`stats.txt` and `plot.png`.

## One-time setup

### Ego (comma 3X)

From its openpilot checkout:

```bash
sudo tools/hcc/device/setup_network.sh ego   # hotspot + relay service + params
```

Your SSH session drops when the network switches. That is expected and means
setup finished. Re-run it after an AGNOS update, which removes the relay unit.

### Lead (Raspberry Pi)

The lead does not run openpilot. Join the Pi to the ego's hotspot, then from
the laptop:

```bash
./tools/hcc/device/deploy_lead.sh pi@10.42.0.50
```

That ships ~100 KB -- four pure-stdlib modules and the scenario table -- and
then proves it imports and loads scenario 48 on the device. It runs on any
Python 3.8+, so Pi OS Bullseye or Bookworm are both fine.

Re-run it after changing anything in the publisher. `hcc preflight` checks the
bundle is present and importable.

The Pi carries no config of its own: `config/devices.toml` on the laptop is the
single source of truth, and `hcc run` passes the relay address, port, device id
and rate to the Pi as flags.

## Layout

| Path | What it is |
|---|---|
| `hcc` | the single entry point |
| `config/` | `devices.toml` and `scenarios.csv` |
| `publishers/` | things that put data on the wire: `virtual_lead.py`, `pedal.py` |
| `device/` | things that run on a device: `relay.py`, `monitor.py`, `bench_ego.py`, `setup_network.sh` (ego), `deploy_lead.sh` (lead) |
| `fieldtest/` | laptop-side orchestration: `remote.py`, `preflight.py`, `session.py`, `cli.py` |
| `analysis/` | `load.py`, `plot_run.py` (link health), `plot_accel.py` (lead vs ego) |

The control code itself is not here — it lives in
[`selfdrive/controls/lib/hcc/`](../../selfdrive/controls/lib/hcc/).

## The pedal rig

`publishers/pedal.py` runs on the laptop and is the human-in-the-loop input.
Pick axes with `--list-axes`, then run it; it calibrates per session because a
load cell's rest offset moves with temperature and mount preload.

`--source synthetic` needs no hardware and still exercises the whole path, which
is what the bench steps use.

**While `HCCPedalEnabled` is set, an unhealthy pedal link denies the car
positive acceleration.** That is deliberate — a stale pedal stream must not let
a positive controller command resume alone — but it means stopping the
publisher will stop the car accelerating.

## Tests

```bash
PYTHONPATH=. python -m pytest --noconftest -o addopts="" tools/hcc/tests selfdrive/controls/tests/test_hcc_*.py
```

`--noconftest -o addopts=""` is needed off-device: the repo conftest and the
pytest-xdist addopts both require a built openpilot. On a device, plain
`pytest` works.
