# Real-world testing (in-car HC3)

Engineering reference for the algorithm and system:
[`HCC_TECHNICAL_OVERVIEW.md`](HCC_TECHNICAL_OVERVIEW.md).

This directory is the PC side of in-car HC3 testing: starting scenarios
remotely, watching the run, and collecting every artifact into one folder. The
V2V components it drives (relay, virtual lead, hotspot setup) are documented in
[`tools/hcc_v2v/README.md`](../hcc_v2v/README.md).

## Setup

Phase 1 uses one car. The ego Comma 3X is on the car harness and drives. The
lead Comma 3X sits powered in the same car, joined to the ego's WiFi hotspot,
replaying a scenario speed profile as V2V packets (`virtual_lead.py`). The
laptop joins the same hotspot and controls the test over SSH.

```
                 +----------------- the car -----------------+
                 |   lead 3X --V2V--> ego 3X (drives)         |
                 +------^--------------^----------------------+
                        | SSH          | SSH
                   laptop on the ego's hotspot
                   (field_test.py / field_test_ui.py)
```

Every run produces a folder under `runs/`:

```
runs/run_20260612_153000_scn48/
  metadata.json    run id, scenario, hosts, start/end time, notes
  relay.csv        every V2V packet the relay saw during the run
  sent.csv         every packet the virtual lead actually sent
  ego_monitor.csv  ego response at 10 Hz: vEgo, engaged, HC3/manual/planner accel
  virtual_lead.log lead-side console output
  stats.txt        packet count, loss %, rate, inter-arrival jitter
  plot.png         v_lead / a_lead / inter-arrival over time
```

## One-time setup

1. Devices: follow the setup in [`tools/hcc_v2v/README.md`](../hcc_v2v/README.md)
   (hotspot plus relay on the ego, hotspot join on the lead, `EnableHCCC` and
   `AlphaLongitudinalEnabled` params on the ego). Both devices need this repo at
   `/data/openpilot`.
2. Laptop SSH: `field_test.py` uses non-interactive SSH. Make sure
   `ssh comma@10.42.0.1 true` works without a password prompt.
3. Laptop WiFi: join the ego's hotspot (`hcc-v2v` / `hcc-v2v-research`). The ego
   is `10.42.0.1`; the lead gets a DHCP address.

## Running a test

### With the UI

```bash
./tools/real_world_testing/launch_ui.sh
```

1. Find lead IP: lists devices on the ego's hotspot; put the lead's address in
   the Lead host field.
2. Preflight check: verifies SSH to both devices, the relay, all HC3/V2V params,
   the scenario CSV, connectivity, and clock skew. Fix anything it flags before
   driving.
3. Set scenario (for example `48`), optional duration cap, start delay, and
   notes. Press START RUN and confirm.
4. Drive: the driver engages openpilot; the ego follows the virtual lead.
5. End run (collect data): stops the scenario and monitors and pulls everything
   into the run folder. ABORT is the emergency version: it kills the publisher
   immediately, and the driver should still brake/disengage.

### From the command line

The UI wraps these:

```bash
python3 tools/real_world_testing/field_test.py find-lead
python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.34
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 60 \
    --lead_host 10.42.0.34 --notes "parking lot, dry, second ramp test"
python3 tools/real_world_testing/field_test.py abort --lead_host 10.42.0.34
python3 tools/real_world_testing/field_test.py collect run_20260612_153000_scn48 --lead_host 10.42.0.34
```

Ctrl-C during a run does a graceful end plus data collection.

`run` flags: `--start_delay` (default 5 s of initial speed before the profile
moves, giving the driver time to engage), `--end stop|hold` (default `stop`:
publisher exits at profile end and the ego coasts/disengages via staleness),
`--max_speed_mph` (scale the whole profile down to a top-speed cap, for example
`--max_speed_mph 20` turns scenario 48's 32 mph peak into 20; accelerations scale
proportionally), `--speed_scale`, `--loop`, `--monitor_hz`, `--skip_checks`.

## Bench test first (no car needed)

The whole pipeline minus driving can be rehearsed with both devices on a desk.
The one thing missing without a car is the real ego subscriber (it lives inside
`controlsd`, which only runs onroad), so `run --bench` starts a stand-in
subscriber (`tools/hcc_v2v/bench_ego.py`) on the ego that registers with the
relay and counts what it receives:

```bash
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 30 \
    --lead_host <lead-ip> --bench --notes "bench rehearsal"
```

A good bench run: preflight green, the lead's status line ramping, and afterwards
a run folder whose `stats.txt` shows about 50 Hz with 0% loss and whose
`bench_ego.log` shows `received=` counting up. `ego_monitor.csv` is empty/invalid
on the bench (openpilot is offroad), which is expected.

Never pass `--bench` when the ego is in a car: the relay tracks one ego peer, and
the stand-in would steal the registration from the real subscriber.

## Switching test vehicles: undo the Sportage device hacks FIRST

The 2026-06-14 test (2023 Kia Sportage) required four device-only workarounds to
force openpilot to accept an unsupported camera-SCC car. They are not in git;
they live only on the ego at `/data/openpilot`. Before testing any other car,
revert all four, or they will sabotage the new vehicle (the forced `FINGERPRINT`
would make openpilot treat, for example, a Tucson as a Sportage). See
[`field_report_20260614.md`](field_report_20260614.md) for the full story.

Run these on the laptop, then reboot the ego:

```bash
ssh comma@10.42.0.1 "sed -i '/FINGERPRINT/d; /SKIP_FW_QUERY/d' /data/openpilot/launch_env.sh"
ssh comma@10.42.0.1 'F=/data/openpilot/opendbc_repo/opendbc/car/hyundai/values.py; cp $F.bak $F'
ssh comma@10.42.0.1 'cd /data/openpilot && git checkout -- tools/joystick/joystick_control.py'
ssh comma@10.42.0.1 'PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "from openpilot.common.params import Params; Params().put_bool(\"JoystickDebugMode\", False)"'
ssh comma@10.42.0.1 'sudo reboot'
```

Then confirm the new car fingerprints natively (no forcing): a real fingerprint,
`dashcam=False`, `opLong=True`, and an `fwdRadar` (0x7d0) ECU in the firmware
list (its absence is what broke the Sportage):

```bash
ssh comma@10.42.0.1 'PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "from openpilot.common.params import Params; from cereal import car; d=Params().get(\"CarParamsPersistent\") or Params().get(\"CarParams\"); cp=car.CarParams.from_bytes(d).__enter__(); print(cp.carFingerprint, \"dashcam=\", cp.dashcamOnly, \"opLong=\", cp.openpilotLongitudinalControl); [print(fw.ecu, hex(fw.address)) for fw in cp.carFw]"'
```

Lesson from the Sportage: it lacked the SCC (adaptive cruise) hardware openpilot
needs to inject acceleration, so it engaged but never actuated. For the next car,
pick a radar-SCC, fully-supported vehicle (for example a 2022-2024 Hyundai Tucson
4th gen: radar-SCC, no minimum engage speed, engages from a standstill). Confirm
the specific car has factory Smart Cruise Control before relying on it.

## In-car test day (phase 1)

The bench test (2026-06-12) validated everything except the driving. In the car,
the real subscriber inside `controlsd` replaces `bench_ego` (so no `--bench`), and
`ego_monitor.csv` records a real response instead of NaNs.

### Day before (devices on lab WiFi, internet available)

1. Reconcile the device checkouts (both trees are dirty from scp'd fixes). On each
   device: `cd /data/openpilot && git checkout -- . && git pull origin hcc-ego`.
2. Sync the clocks: while both devices have internet, confirm NTP has them within
   about 100 ms (preflight verifies again on the day).
3. Re-run the ego setup if anything changed
   (`sudo tools/hcc_v2v/scripts/setup_v2v_network.sh ego`); it is idempotent.
4. Update your laptop checkout (`git pull`).

### Packing list

- Both Comma 3X devices (ego mounts on the car harness).
- USB power for the lead device (car USB port or battery pack).
- Laptop, charged; it will be on the ego's hotspot with no internet, so get
  anything you need beforehand.
- Two people: a driver (hands on wheel, foot over brake) and an operator (laptop,
  runs/aborts the scenario). Do not solo this.

### In the car

1. Mount and plug the ego, power the lead from USB. Both boot; the ego brings up
   the hotspot automatically (relay is a systemd service), and the lead rejoins.
2. Laptop: join the `hcc-v2v` hotspot, then preflight:
   ```bash
   python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.60
   ```
   All 14 checks green before moving the car. Each failure prints its fix.
3. Walk the [safety checklist](#safety-checklist-before-every-session) out loud
   with the driver.
4. First run, short and capped. Drive to the start point, car in gear, road clear,
   then:
   ```bash
   python3 tools/real_world_testing/field_test.py run --scn 48 --duration 30 \
       --lead_host 10.42.0.60 --start_delay 10 --notes "first in-car run, <location>, <driver>"
   ```
   `--start_delay 10` holds the initial speed for 10 s so the driver can engage,
   then the profile starts moving.
5. While it runs, the operator watches the streamed lead status line and the road.
   The car will accelerate to follow the virtual lead. Brake is instant manual
   override.
6. The run ends at `--duration`; collection is automatic. Ctrl-C ends it early and
   still collects. `abort` (second terminal) stops the data feed; the driver's
   brake stops the car.
7. Between runs, check `runs/<id>/stats.txt` for transport health; `ego_monitor.csv`
   has the real response.

### Success criteria for the first in-car session

- Preflight 14/14 green in the car.
- Relay stats comparable to the bench (about 50 Hz, about 0% loss, max gap under
  100 ms).
- `ego_monitor.csv` shows `engaged=1` during the run and a nonzero `hccc_accel`
  trace.
- `v_ego_mps` tracks the scenario ramp (plot it against `relay.csv`'s `v_lead`;
  both are on the ego's clock).

### If something looks wrong while driving

| Symptom | Meaning | Action |
|---|---|---|
| Engaged but no acceleration | V2V signal not reaching hCCC | End run; check `stats.txt` rejection reasons (`missing_ego_peer` = subscriber not registered; skew = clocks) |
| Car accelerates harder than expected | Tracking the profile; scenario 48 ramps to 32 mph | Driver brakes (disengages); pick a longer road or smaller `--duration` |
| Lead status line stops streaming | Laptop fell off the hotspot or lead died | Driver disengages; rejoin hotspot, `abort`, then `collect <run_id>` recovers the data |
| Anything else | n/a | Brake, disengage, `abort`, review the run folder before retrying |

## Reviewing a run

`stats.txt` and `plot.png` are generated automatically. To re-analyze or overlay
the sent profile:

```bash
python3 tools/hcc_v2v/plot_v2v_run.py runs/<id>/relay.csv --sent runs/<id>/sent.csv
```

To compare the lead's transmitted acceleration against the ego's actual measured
acceleration (one `accel_comparison.png` per run, plus a combined
`runs/accel_comparison_all.png` montage):

```bash
python3 tools/real_world_testing/plot_run_comparison.py            # all runs
python3 tools/real_world_testing/plot_run_comparison.py runs/<id>  # one run
python3 tools/real_world_testing/plot_run_comparison.py --with_cmd # also show commanded accel
```

It reads `a_lead` from `relay.csv` and `a_ego_mps2` from `ego_monitor.csv` (both
on the ego clock, so they align on time) and reports the lead-vs-ego correlation
per run.

`ego_monitor.csv` holds the ego's actual response (10 Hz): wall-clock timestamp,
`v_ego_mps`, `engaged`, longitudinal state, and the three accel contributions
(`hccc_accel`, `manual_accel`, `planner_accel`). Its timestamps and `relay.csv`'s
are both on the ego's clock, so the two files join on time. Full openpilot route
logs are on the ego as usual for deeper analysis.

## What this tooling handles

- Remote start/stop of scenarios from the laptop over SSH; no one touches the lead
  once it is powered.
- Every artifact lands in one timestamped run folder; nothing is overwritten.
- The ego response is recorded (`hcc_monitor.py --log_csv` runs on the ego for
  every run).
- Preflight measures lead-to-ego clock skew and fails loudly with the fix (the
  subscriber silently rejects packets more than 500 ms off the ego clock, and the
  lead has no internet on the hotspot so it drifts).
- Publisher and monitor run under `nohup`, so a dropped laptop connection does not
  end the run (reconnect and `abort` or `collect`).
- Preflight checks `EnableHCCC`, `AlphaLongitudinalEnabled`, `HCCV2VEnabled`,
  `HCCV2VOnly`, and relay host on both devices before anything moves.

## Known limitations

- Driver engagement is manual. The tooling never engages openpilot; the driver
  does, after the scenario is live (`--start_delay` exists for this). It cannot
  disengage the car; the driver's brake is the safety system. `abort` only stops
  the data feed.
- Hotspot range: laptop, lead, and ego must stay in WiFi range of the ego. For
  phase 2 (two cars) the laptop should be in the ego car.
- Device clocks drift over weeks. If preflight keeps failing on skew, give both
  devices internet briefly (NTP) before a test day.
- `/data/hcc_v2v_logs/` grows. Clear old files occasionally
  (`ssh comma@<dev> 'rm /data/hcc_v2v_logs/*'`; collected runs are safe on the
  laptop).

## Device gotchas

- Python on the devices: openpilot's deps live in `/usr/local/venv`, which only
  interactive login shells put on PATH. Non-interactive SSH and `sudo` shells must
  use `/usr/local/venv/bin/python3` with `PYTHONPATH=/data/openpilot` explicitly.
  `field_test.py` handles this; remember it for ad-hoc commands.
- One WiFi radio per device: activating the hotspot (ego) or joining it (lead)
  drops the device off lab WiFi. Your SSH session dying at the end of
  `setup_v2v_network.sh` is the success signal.
- No internet on the hotspot: once both devices are on `hcc-v2v`, `git pull` does
  not work. Push single-file fixes with `scp`; reconcile with
  `git checkout -- . && git pull` next time they are on lab WiFi.
- Read-only rootfs: writing systemd units needs `sudo mount -o remount,rw /` first
  (and `,ro` after). An AGNOS update/reflash deletes the relay unit; re-run
  `setup_v2v_network.sh ego` afterwards.
- Typed params: `HCCV2VRelayPort` is INT-typed; `Params().get()` returns an int,
  not a string (handled in `hcc_v2v.py::_read_str_param`, commit `6a2da835c`).
- Run state lives on the devices under `/data/hcc_v2v_logs/`. If a run fails before
  collection, use `field_test.py collect <run_id>`, and `abort` to kill strays.

## Bench-test status log

- 2026-06-12: full device setup completed (ego `comma-7259cb5e` = 10.42.0.1
  hotspot plus relay plus params; lead = 10.42.0.60 joined plus params). Preflight
  12/12 green, clock skew about 20 ms. Take 1 caught the int-param crash (fixed);
  takes 2 and 3 hit an SSH timeout launching the virtual lead. Root cause: the
  lead's sshd keeps the session open until the nohup'd child exits, even with
  stdio fully redirected, so the launch always outlived the SSH timeout (the
  launch itself was succeeding). Fixed PC-side in `field_test.py` with
  `ssh_launch()`: read the echoed PID, then close the local ssh client. Take 4:
  launch fix confirmed. But all 1750 packets were rejected `missing_ego_peer`:
  `/data/hcc_v2v_logs` on the ego was created root-owned by the relay service, so
  the comma-user log redirections failed and bench_ego and hcc_monitor died at
  launch while still echoing plausible PIDs. Fixed: launches now verify the process
  survived 1 s, preflight checks log-dir writability on both devices, and the relay
  unit chowns the dir to comma. Take 5 (after a one-time
  `sudo chown comma:comma /data/hcc_v2v_logs` on the ego): SUCCESS. 1750/1750
  packets forwarded at 50.0 Hz, 0.00% loss, inter-arrival median 19.9 ms / max
  51.4 ms (limit 100 ms), bench_ego received all 1750, `ego_monitor.csv` created
  with 410 samples (all-nan/invalid as expected offroad), full artifact set plus
  plot collected (`runs/run_20260612_190830_scn48`). The bench pipeline is
  validated.

## Safety checklist (before every session)

- Empty road / closed area; scenario top speed fits the space (scenario 48 peaks
  at 14.4 m/s, about 32 mph).
- Driver knows the lead is virtual (nothing is physically ahead) and the car will
  accelerate to track it once engaged.
- Driver knows the run plan (scenario shape, duration) and that brake pressure
  disengages everything.
- Preflight is green, including clock skew.
- `--duration` is set for early runs so the profile cannot outlive the road.
