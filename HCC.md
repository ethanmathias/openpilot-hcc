# HCC — human-in-the-loop cooperative cruise control

This fork adds a cooperative longitudinal controller (hCCC) to openpilot for a
driving-research study, plus the V2V link and field tooling it needs.

Project documentation — design, findings, field reports, platform assessments —
lives in the Notion **openpilot Research Log**, not in this repository.

## What is here

| Path | What it is |
|---|---|
| [`selfdrive/controls/lib/hcc/`](selfdrive/controls/lib/hcc/) | the controller, V2V transport, pedal blend and config |
| [`tools/hcc/`](tools/hcc/) | publishers, device tooling, run orchestration, analysis |
| [`selfdrive/controls/tests/test_hcc_*.py`](selfdrive/controls/tests/) | unit tests for the control path |

Everything else in the tree is upstream openpilot. The footprint in upstream
files is deliberately small — a seam in `longcontrol.py` and `controlsd.py`,
some params, and four settings toggles — so this fork stays rebaseable.

## Turning it on

Nothing activates until its params are set, and with none set openpilot behaves
exactly as upstream. The toggles are in **Settings → Developer**:

| Param | What it does |
|---|---|
| `EnableHCCC` | the hCCC longitudinal controller |
| `HCCV2VEnabled` | subscribe to V2V lead packets |
| `HCCV2VOnly` | take the lead only from V2V, never the onboard lead |
| `HCCPedalEnabled` | take the human input from the pedal rig over V2V |

`AlphaLongitudinalEnabled` is also required — it is what gives openpilot
longitudinal control and sets the panda's longitudinal safety flag.

## Running a test

See [`tools/hcc/README.md`](tools/hcc/README.md). In short, from a laptop on the
ego's hotspot:

```bash
./tools/hcc/hcc preflight
./tools/hcc/hcc run --scn 48
```

## Safety

The safety driver steers throughout — lateral actuation is disabled outright,
not tuned down. The driver's brake pedal disengages openpilot and is the abort.
No change in this fork touches vehicle safety firmware: the `opendbc` and
`panda` submodules are stock.
