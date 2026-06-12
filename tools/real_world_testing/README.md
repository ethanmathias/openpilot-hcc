# Real-world testing (in-car HC3)

Everything for running HC3 tests in a real car lives here. The test design,
the V2V components it relies on (relay, virtual lead, hotspot setup), and
the device-side details are documented in
[`tools/hcc_v2v/README.md`](../hcc_v2v/README.md) — **this** directory is the
PC side: starting scenarios remotely, watching the run, and getting every
byte of data back into one reviewable folder.

## The big picture

**Phase 1 (current):** one car. The *ego* Comma 3X is on the car harness and
drives the car. The *lead* Comma 3X sits powered in the same car, joined to
the ego's WiFi hotspot, replaying a scenario speed profile as V2V packets
(`virtual_lead.py`). The ego follows that virtual lead. Your laptop also
joins the ego's hotspot and controls the whole test over SSH:

```
                 ┌────────────────── the car ──────────────────┐
                 │   lead 3X ──V2V──► ego 3X (drives the car)   │
                 └───────▲──────────────▲──────────────────────┘
                         │ SSH          │ SSH
                    laptop on the ego's hotspot
                    (field_test.py / field_test_ui.py)
```

Every run produces a folder under `tools/real_world_testing/runs/`:

```
runs/run_20260612_153000_scn48/
├── metadata.json      run id, scenario, hosts, exact start/end time, notes
├── relay.csv          every V2V packet the relay saw during the run window
├── sent.csv           every packet the virtual lead actually sent
├── ego_monitor.csv    ego response at 10 Hz: vEgo, engaged, HC3/manual/planner accel
├── virtual_lead.log   the lead-side console output
├── stats.txt          packet count, loss %, rate, inter-arrival jitter
└── plot.png           v_lead / a_lead / inter-arrival over time
```

## One-time setup

1. **Devices**: follow the one-time setup in
   [`tools/hcc_v2v/README.md`](../hcc_v2v/README.md) (hotspot + relay on
   the ego, hotspot join on the lead, `EnableHCCC` /
   `AlphaLongitudinalEnabled` params on the ego). Both devices need this
   repo at `/data/openpilot`.
2. **Laptop SSH keys**: `field_test.py` uses non-interactive SSH. Make sure
   `ssh comma@10.42.0.1 true` works without a password prompt from the
   laptop (the devices accept the GitHub keys configured in their settings,
   or add your key to `~/.ssh/authorized_keys` as `comma`).
3. **Laptop WiFi**: join the ego's hotspot (`hcc-v2v` / `hcc-v2v-research`
   by default). The ego is `10.42.0.1`; the lead gets a DHCP address.

## Running a test

### With the UI

```bash
./tools/real_world_testing/launch_ui.sh
```

1. **Find lead IP** — lists the devices on the ego's hotspot; put the
   lead's address in the *Lead host* field.
2. **Preflight check** — verifies SSH to both devices, the relay service,
   all HC3/V2V params, the scenario CSV, lead→ego connectivity, and
   device clock skew. Fix anything it flags before driving.
3. Set scenario (e.g. `48`), optional duration cap, start delay, and notes
   (location, weather, who's driving). Press **START RUN** and confirm.
4. Drive: the driver engages openpilot as usual; the ego follows the
   virtual lead. The log pane streams the lead's status line
   (`t=… v=… a=… seq=…`).
5. **End run (collect data)** — stops the scenario, stops the monitors, and
   pulls everything into the run folder. **ABORT** is the emergency
   version: it kills the publisher immediately (the ego stops receiving
   V2V and stops commanding acceleration within ~100 ms) — the driver
   should still brake/disengage as normal.

### From the command line

The UI is a wrapper over these — everything is scriptable:

```bash
# Who's on the hotspot? (find the lead's IP)
python3 tools/real_world_testing/field_test.py find-lead

# Preflight
python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.34

# A 60-second capped run of scenario 48 with notes
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 60 \
    --lead_host 10.42.0.34 --notes "parking lot, dry, second ramp test"

# Ctrl-C during a run = graceful end + data collection.

# Emergency stop (kills the publisher on the lead immediately)
python3 tools/real_world_testing/field_test.py abort --lead_host 10.42.0.34

# The PC died mid-run? Pull whatever the devices still have:
python3 tools/real_world_testing/field_test.py collect run_20260612_153000_scn48 --lead_host 10.42.0.34
```

`run` flags: `--start_delay` (default 5 s of initial speed before the
profile moves — gives the driver time to engage), `--end stop|hold`
(default `stop`: when the profile ends the publisher exits and the ego
coasts/disengages via staleness), `--loop`, `--monitor_hz`, `--skip_checks`.

## Bench test first (no car needed)

The whole pipeline minus the driving can be rehearsed with both devices on a
desk. The only thing missing without a car is the real ego subscriber — it
lives inside `controlsd`, which only runs onroad — so `run --bench` starts a
stand-in subscriber (`tools/hcc_v2v/bench_ego.py`) on the ego device that
registers with the relay and counts what it receives:

```bash
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 30 \
    --lead_host <lead-ip> --bench --notes "bench rehearsal"
```

A good bench run looks like: preflight green, the lead's status line ramping
(`v=` climbing at scenario rate), and afterwards a run folder whose
`stats.txt` shows ~50 Hz with 0% loss and whose `bench_ego.log` shows
`received=` counting up. `ego_monitor.csv` will be empty/invalid on the
bench — openpilot is offroad — that's expected.

**Never pass `--bench` when the ego is in a car**: the relay tracks one ego
peer, and the stand-in would steal the registration from the real
subscriber.

## Reviewing a run

`stats.txt` and `plot.png` are generated automatically. To re-analyze or
overlay the sent profile:

```bash
python3 tools/hcc_v2v/plot_v2v_run.py runs/<id>/relay.csv --sent runs/<id>/sent.csv
```

`ego_monitor.csv` holds the ego's actual response (10 Hz): wall-clock
timestamp, `v_ego_mps`, `engaged`, longitudinal state, and the three accel
contributions (`hccc_accel`, `manual_accel`, `planner_accel`). Its
timestamps and `relay.csv`'s are both on the ego's clock, so the two files
join directly on time — lead command vs. ego response on one axis.

The full openpilot route logs (camera, CAN, every message) are also on the
ego device as usual and can be pulled with the normal openpilot tooling if
deeper analysis is needed.

## Known gaps this tooling closes (and the ones it doesn't)

Closed:

- **No remote start** — scenarios start/stop from the laptop over SSH; no
  one has to touch the lead device once it's powered.
- **Scattered data** — every artifact lands in one timestamped run folder
  with metadata; nothing is overwritten between runs.
- **Ego response wasn't recorded** — `hcc_monitor.py --log_csv` runs on the
  ego for the duration of every run.
- **Silent clock-skew failures** — the V2V subscriber rejects packets whose
  sender timestamp is >500 ms off the ego's clock, *silently*. The lead has
  no internet while on the hotspot, so its clock drifts between test days.
  Preflight measures lead↔ego skew and fails loudly with the fix.
- **SSH drops killing the test** — the publisher and monitor run under
  `nohup` on the devices; a dropped laptop connection doesn't end the run
  (reconnect and `abort` or `collect`).
- **Misconfigured params discovered while driving** — preflight checks
  `EnableHCCC`, `AlphaLongitudinalEnabled`, `HCCV2VEnabled`, `HCCV2VOnly`,
  and relay host on both devices before anything moves.

Still open (be aware):

- **Driver engagement is manual.** The tooling never engages openpilot —
  the driver does, after the scenario is live (`--start_delay` exists for
  exactly this). It also cannot disengage the car; the driver's brake is
  the real safety system. `abort` only stops the data feed.
- **Hotspot range.** Laptop, lead, and ego must stay within WiFi range of
  the ego. In a single car this is a non-issue; for phase-2 (two cars) the
  laptop should be in the ego car.
- **Device clocks drift over weeks.** If preflight keeps failing on skew,
  give both devices internet briefly (NTP) before a test day.
- **`/data/hcc_v2v_logs/` grows.** Relay/monitor/sent logs accumulate on
  the devices; clear old files occasionally
  (`ssh comma@<dev> 'rm /data/hcc_v2v_logs/*'` — collected runs are safe
  on the laptop).

## Device gotchas (learned during bring-up — read before debugging)

- **Python on the devices**: openpilot's deps live in `/usr/local/venv`, which
  only interactive login shells put on PATH. Non-interactive SSH and `sudo`
  shells must use `/usr/local/venv/bin/python3` with
  `PYTHONPATH=/data/openpilot` explicitly. `field_test.py` handles this
  automatically; remember it when running ad-hoc commands.
- **One WiFi radio per device**: activating the hotspot (ego) or joining it
  (lead) drops the device off the lab WiFi — your SSH session dying at the
  end of `setup_v2v_network.sh` is the success signal, not a failure. The
  script does all real work *before* the network switch for this reason.
- **No internet on the hotspot**: once both devices are on `hcc-v2v`,
  `git pull` doesn't work on them. Push single-file fixes with
  `scp <file> comma@10.42.0.x:/data/openpilot/<file>`; reconcile with
  `git checkout -- . && git pull` next time they're on lab WiFi.
- **Read-only rootfs**: writing systemd units needs
  `sudo mount -o remount,rw /` first (and `,ro` after). If remount says
  "busy", reboot and retry. An AGNOS update/reflash deletes the relay unit —
  re-run `setup_v2v_network.sh ego` afterwards.
- **Typed params**: `HCCV2VRelayPort` is INT-typed; `Params().get()` returns
  an `int`, not a string. Code that assumes str crashes (fixed in
  `hcc_v2v.py::_read_str_param`, commit `6a2da835c`).
- **Run state lives on the devices** under `/data/hcc_v2v_logs/`. If a run
  fails before collection, the logs are still there — use
  `field_test.py collect <run_id>`, and `abort` to kill stray processes.

## Bench-test status log

- **2026-06-12**: full device setup completed (ego `comma-7259cb5e` =
  10.42.0.1 hotspot + relay + params; lead = 10.42.0.60 joined + params).
  Preflight 12/12 green, clock skew ~20 ms. Take 1 caught the
  int-param crash (fixed); takes 2 and 3 both hit an SSH timeout launching
  the virtual lead. Root cause found after take 3: the lead's sshd keeps the
  session open until the nohup'd child *exits*, even with stdin/stdout/stderr
  fully redirected — so the launch always outlived the SSH timeout (the
  launch itself was succeeding). Fixed PC-side in `field_test.py` with
  `ssh_launch()`: read the echoed PID, then close the local ssh client
  ourselves. No device-side update needed (ego already has the
  `hcc_monitor.py` fix via scp; both devices have the `hcc_v2v.py` fix).
  **Next: take 4** —
  ```bash
  python3 tools/real_world_testing/field_test.py abort --lead_host 10.42.0.60   # clear strays (safe if none)
  python3 tools/real_world_testing/field_test.py run --scn 48 --duration 30 \
      --lead_host 10.42.0.60 --bench --notes "bench take 4"
  ```
  Open questions for take 4: confirm `ego_monitor.csv` gets created (take 1
  produced none; the CSV-open was moved ahead of SubMaster init to fix it),
  and check `/data/hcc_v2v_logs/vl_run_*.log` + `sent_*.csv` from takes 2/3
  on the lead — if they have data, the publisher really was running all along.

## Safety checklist (before every session)

- Empty road / closed area; scenario top speed fits the space
  (scenario 48 peaks at 14.4 m/s ≈ 32 mph).
- Driver knows: the "lead" is virtual — nothing physical is ahead; the car
  **will accelerate** to track it once engaged.
- Driver knows the run plan (scenario shape, duration) and that brake
  pressure disengages everything, always.
- Preflight is green, including clock skew.
- `--duration` is set for early runs so the profile cannot outlive the
  road available.
