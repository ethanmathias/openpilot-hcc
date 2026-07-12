# Field test command sheet: Tucson 2024 (copy-paste ready)

Every command block below is paste-safe for zsh (no `#` comment lines inside
blocks; explanations are in the prose). Run everything from the laptop.

**IPs**

| Where | Ego | Lead |
|---|---|---|
| Lab WiFi | `192.168.86.38` | `192.168.86.22` |
| Hotspot (`hcc-v2v` / password `hcc-v2v-research`) | `10.42.0.1` | `10.42.0.60` (verify with find-lead) |

Expected git state: both devices and the laptop print the SAME commit from
`git log --oneline -1`, and it is `129f18d7a` (the commit adding
`--max_speed_mph`) or newer.

Once you join the hotspot there is no internet. Read this whole file before
leaving lab WiFi.

## 1. Day before / morning of, on lab WiFi

Pull both devices and the laptop to the same commit:

```bash
ssh comma@192.168.86.38 'cd /data/openpilot && git checkout -- . && git pull origin hcc-ego'
```

```bash
ssh comma@192.168.86.22 'cd /data/openpilot && git checkout -- . && git pull origin hcc-ego'
```

The `Encountered 1 file that should have been a pointer` LFS warning is harmless.

Verify both are on the expected commit (want `129f18d7a ...`):

```bash
ssh comma@192.168.86.38 'cd /data/openpilot && git log --oneline -1'
```

```bash
ssh comma@192.168.86.22 'cd /data/openpilot && git log --oneline -1'
```

Verify the ego's params (want `joystick= False`, everything else `True`):

```bash
ssh comma@192.168.86.38 'PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "from openpilot.common.params import Params; p=Params(); print(\"joystick=\", p.get_bool(\"JoystickDebugMode\"), \"alphaLong=\", p.get_bool(\"AlphaLongitudinalEnabled\"), \"hccc=\", p.get_bool(\"EnableHCCC\"), \"v2v=\", p.get_bool(\"HCCV2VEnabled\"), \"v2vonly=\", p.get_bool(\"HCCV2VOnly\"))"'
```

If any param is wrong, fix it (substitute the key):

```bash
ssh comma@192.168.86.38 'PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "from openpilot.common.params import Params; Params().put_bool(\"EnableHCCC\", True)"'
```

Leave both devices powered on lab WiFi so NTP settles their clocks (the lead
drifts whenever it is offline).

## 2. At the car: setup

1. Plug the ego into the harness (same `hyundai_n` harness as before), power the
   lead over USB. Both boot; with no lab WiFi in range, the ego hosts the
   `hcc-v2v` hotspot and the lead joins it automatically.
2. Laptop: join WiFi `hcc-v2v`, password `hcc-v2v-research`.
3. Turn the car fully ON (ignition, not just accessory) so openpilot can
   fingerprint it.

Confirm the lead's hotspot IP (expect `10.42.0.60`; substitute below if
different):

```bash
python3 tools/real_world_testing/field_test.py find-lead
```

Fingerprint check, the single most important check of the day. Want
`HYUNDAI_TUCSON_4TH_GEN`, `dashcam= False`, `opLong= True`, and a line containing
`fwdRadar 0x7d0` in the firmware list (that radar is the SCC hardware the Sportage
was missing):

```bash
ssh comma@10.42.0.1 'PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "from openpilot.common.params import Params; from cereal import car; d=Params().get(\"CarParamsPersistent\") or Params().get(\"CarParams\"); cp=car.CarParams.from_bytes(d).__enter__(); print(cp.carFingerprint, \"dashcam=\", cp.dashcamOnly, \"opLong=\", cp.openpilotLongitudinalControl); [print(fw.ecu, hex(fw.address)) for fw in cp.carFw]"'
```

If it says `MOCK` / `dashcam= True`: reboot the ego with the car fully on and
re-check. If it still fails, the trim likely lacks SCC, so stop and reassess. Do
NOT force the fingerprint.

Preflight, all 14 must pass before the car moves; each failure prints its own fix:

```bash
python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.60
```

## 3. Clock sync fix (when preflight fails on skew)

The V2V subscriber silently rejects packets whose sender timestamp is more than
500 ms off the ego's clock. The lead drifts while offline. The fix sets the lead's
clock FROM the ego's (only relative skew matters; the ego is the reference).

The naive sync always lands about 600 ms behind because of the SSH round-trip, so
pre-compensate by about 0.9 s. Run these two lines together:

```bash
EGO=$(ssh comma@10.42.0.1 'date +%s.%N')
ssh comma@10.42.0.60 "sudo date -s @$(echo "$EGO + 0.9" | bc)"
```

Re-run preflight and read the residual skew:

```bash
python3 tools/real_world_testing/field_test.py check --lead_host 10.42.0.60
```

Tuning: if preflight still shows about -600 ms the offset did not apply (rerun both
lines). If it shows about -150 ms, increase `0.9` to `1.05`; if about +300 ms,
decrease to `0.6`. Anything inside 300 ms passes with margin; last time `+ 0.9`
landed at +228 ms on the first try. A `[WARN]` under 500 ms is OK to drive on.
Re-check before every run; a free-running clock drifts tens of ms per hour. If the
lead ever reboots, its clock resets, so re-run this section.

## 4. Live monitor (optional but recommended)

Second terminal, leave it running during runs. Watch `engaged`, `v`, and the
`hccc=` accel term:

```bash
ssh comma@10.42.0.1 'cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 tools/hcc_v2v/scripts/hcc_monitor.py'
```

## 5. Low-speed shakedown (parking lot, before any scenario)

Driver briefing first, say it out loud:

- The lead is virtual. Nothing is physically ahead. In V2V-only mode openpilot
  does NOT brake for real obstacles; the driver is the only collision avoidance.
- CANCEL button is the clean disengage. Brake cuts actuation instantly at the
  panda, but the UI may show engaged for about 2 more seconds, which is expected.
- Gas pedal does NOT disengage; it adds on top physically.
- Scenario 48 starts at 0 m/s, so you engage at a standstill during the
  start-delay window.

Shakedown sequence, in gear, area clear:

1. Hold brake, press SET at standstill, release brake. Note whether the car holds
   or creeps (creep is expected; the forced-pid path never enters the stopping
   state).
2. Press CANCEL, confirm clean disengage.
3. Re-engage at about 10 mph, tap the brake, confirm actuation cuts instantly (UI
   disengage may lag about 2 s).
4. Re-engage with the virtual lead publishing, feet off. The moment `hccc=` goes
   nonzero and the car physically responds, the Sportage blocker is gone.

## 6. Scenario runs

First run, 20 mph cap, short, with start-delay for engagement (edit
location/driver in the notes):

```bash
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 30 --max_speed_mph 20 --lead_host 10.42.0.60 --start_delay 10 --notes "first tucson run, 20mph cap"
```

The lead prints `speed scale 0.6197` and `top 20.0 mph` at startup; confirm you
see that line in the streamed output before the profile moves. `--start_delay 10`
holds the initial speed (0 for scn 48) for 10 s: the driver presses SET during
that window, then the profile starts.

The run ends itself at `--duration` and collects automatically. Ctrl-C ends early
and still collects. NEVER pass `--bench` with a real car.

Emergency stop for the data feed (driver's brake is the emergency stop for the
car):

```bash
python3 tools/real_world_testing/field_test.py abort --lead_host 10.42.0.60
```

If the laptop died mid-run, recover the artifacts afterwards:

```bash
python3 tools/real_world_testing/field_test.py collect <run_id> --lead_host 10.42.0.60
```

Longer follow-up run once the capped run tracks well (still capped, longer
window):

```bash
python3 tools/real_world_testing/field_test.py run --scn 48 --duration 60 --max_speed_mph 20 --lead_host 10.42.0.60 --start_delay 10 --notes "tucson run 2, 20mph cap, 60s"
```

Full-speed scenario 48 (32 mph peak) only when you have the road for it: drop
`--max_speed_mph 20`.

## 7. Between runs: quick health read

Each run folder is under `tools/real_world_testing/runs/<run_id>/`.

- `stats.txt`: want about 50 Hz, about 0% loss, max gap under 100 ms.
- `ego_monitor.csv`: want `engaged=1` samples, `long_state=pid`, nonzero
  `hccc_accel`, and `v_ego_mps` tracking the (scaled) profile.
- Re-run preflight before the next run (clock drift).

## 8. Troubleshooting quick refs

Engaged but no acceleration, check the relay's rejection reasons (look at the
`reason` column of the newest relay CSV):

```bash
ssh comma@10.42.0.1 'ls -t /data/hcc_v2v_logs/relay_*.csv | head -1'
```

```bash
ssh comma@10.42.0.1 'tail -20 $(ls -t /data/hcc_v2v_logs/relay_*.csv | head -1)'
```

`missing_ego_peer` means the subscriber in controlsd never registered (is
openpilot onroad? is `HCCV2VEnabled=1`?). Timestamp-skew rejections mean redo
section 3.

Relay service status:

```bash
ssh comma@10.42.0.1 'systemctl status hcc-v2v-relay'
```

Lead publisher log from the current run (files are named `vl_<run_id>.log`):

```bash
ssh comma@10.42.0.60 'tail -20 $(ls -t /data/hcc_v2v_logs/vl_*.log | head -1)'
```

Kill stray processes on both devices (safe reset between attempts):

```bash
python3 tools/real_world_testing/field_test.py abort --lead_host 10.42.0.60
```

Car shows "dashcam mode" mid-session: the car was probably power-cycled without
full ignition. Reboot the ego with the car fully on, redo the fingerprint check in
section 2.
