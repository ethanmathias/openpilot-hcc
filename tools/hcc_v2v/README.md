# HCC V2V: relay, virtual lead, and real-world testing

Engineering reference for the algorithm and system:
[`tools/real_world_testing/HCC_TECHNICAL_OVERVIEW.md`](../real_world_testing/HCC_TECHNICAL_OVERVIEW.md).

This directory holds the V2V (vehicle-to-vehicle) leg of HC3: the UDP relay, the
tools that publish and analyze lead data, and the in-car test procedure. In HC3
the ego car follows a lead car using speed/acceleration data the lead transmits
over WiFi, not radar. The data path is always the same three hops:

```
lead publisher --UDP--> relay (runs ON the ego device) --UDP--> ego subscriber (inside openpilot)
```

The lead publisher can be:

- a real lead device measuring a real car (two-car field test),
- `virtual_lead.py` replaying a scenario CSV (phase-1 in-car test, see below),
- a simulated openpilot instance (PC simulation, see
  [`tools/sim/README.md`](../sim/README.md)).

The ego's hCCC controller treats all three identically.

## What's in this directory

| File | What it does | Where it runs |
|---|---|---|
| `relay_server.py` | UDP relay: registers one lead plus one ego, forwards lead packets to the ego, optionally logs every packet to CSV | Ego device (systemd service), or PC in simulation |
| `virtual_lead.py` | Replays a `Scenarios.csv` speed profile as V2V packets (a fake lead car) | Lead device, or any machine that can reach the relay |
| `plot_v2v_run.py` | Stats plus PNG plots from a relay CSV log (loss, rate, speed/accel traces) | PC, after a run |
| `scripts/setup_v2v_network.sh` | One-time setup: ego hotspot plus relay service plus params; lead joins hotspot plus params | Each device, once |
| `scripts/hcc_monitor.py` | Live one-line-per-second status: engaged, speed, HC3/manual/planner accel | Either device, during a run |
| `launch_ui.sh` / `launcher_ui.py` | Tk launcher for the all-on-PC simulation workflow | PC |

Wire format and subscriber/publisher logic live in
`selfdrive/controls/lib/vendor/hcc_v2v_core.py`; the ego's controller reads the
V2V signal through `selfdrive/controls/lib/longcontrol.py`.

## Phase-1 in-car test: one car, virtual lead

Goal: prove the full V2V control loop in a real car with minimal risk. The ego
device is installed normally and drives the car. The lead device sits in the same
car (USB power, mounted anywhere), joined to the ego's WiFi hotspot, replaying a
scenario speed profile. The ego follows the virtual lead as it would a real one.

```
            +------------------- one real car -------------------+
            |  Comma 3X "lead" (USB power only)                  |
            |    virtual_lead.py --scn 48                        |
            |        | WiFi (ego's hotspot, 10.42.0.1)           |
            |        v                                           |
            |  Comma 3X "ego" (car harness, drives the car)      |
            |    relay (systemd) -> openpilot hCCC -> gas/brake  |
            +----------------------------------------------------+
```

### Safety notes (read before driving)

- In V2V-only mode (`HCCV2VOnly=1`) the ego follows the transmitted profile. There
  is no physical car ahead; the ego will accelerate on an empty road to track the
  virtual lead's speed. Pick a closed/empty road and a scenario whose top speed
  fits it (scenario 48 peaks at 14.4 m/s, about 32 mph).
- The driver can always take over: brake pressure disengages openpilot, and manual
  pedal input blends on top of HC3 output.
- If the lead device stops publishing (crash, WiFi drop, Ctrl-C), the ego marks
  the V2V signal stale within about 100 ms and stops commanding acceleration.
  `virtual_lead.py --end stop` uses this at the end of a profile; `--end hold`
  keeps publishing the final speed.
- Keep the SSH session to the lead alive through the ego's hotspot, or start
  `virtual_lead.py` under `tmux`/`nohup` so it survives a dropped SSH session.

### One-time setup (per device)

Both devices need the `hcc-ego` branch at `/data/openpilot` (the lead only runs
`virtual_lead.py`, so it does not need the `hcc-lead` branch for this test).

Ego device (SSH in over your normal WiFi):

```bash
sudo /data/openpilot/tools/hcc_v2v/scripts/setup_v2v_network.sh ego

python3 -c "
from openpilot.common.params import Params
p = Params()
p.put_bool('EnableHCCC', True)
p.put_bool('AlphaLongitudinalEnabled', True)
"
```

The first command sets up the hotspot, relay service, and V2V params. The relay
service starts immediately, restarts on boot, and logs every packet to
`/data/hcc_v2v_logs/relay_<timestamp>.csv` (a new file per service start).

Lead device (after the ego hotspot exists; your SSH over regular WiFi drops when
it switches networks):

```bash
sudo /data/openpilot/tools/hcc_v2v/scripts/setup_v2v_network.sh lead
```

This joins the `hcc-v2v` hotspot and sets `HCCV2VRelayHost` to the ego's hotspot
IP (10.42.0.1). From now on, SSH to the lead through the ego's hotspot.

Verify the link (from the lead):

```bash
ping -c 3 10.42.0.1
```

The manual steps below work, but
[`tools/real_world_testing/`](../real_world_testing/README.md) automates them from
a laptop: preflight checks (params, relay, clock skew), remote start/stop of the
scenario, and automatic collection into a per-run folder with stats and plots.

### Test-day run procedure

1. Ego: boot normally in the car (normal openpilot UI, no special launcher).
   Confirm the relay is up: `systemctl status hcc-v2v-relay`.
2. Lead: power it from USB in the same car. SSH in via the hotspot and start the
   virtual lead:

   ```bash
   cd /data/openpilot
   tmux new -s lead
   python3 tools/hcc_v2v/virtual_lead.py --scn 48 --log_csv /data/hcc_v2v_logs/sent_$(date +%Y%m%d_%H%M%S).csv
   ```

   It prints one status line per second (`t=... v=... a=... seq=...`). Relay
   host/port/device-id come from the params the setup script wrote.
3. Optional live monitor on the ego (second SSH session):

   ```bash
   cd /data/openpilot && python3 tools/hcc_v2v/scripts/hcc_monitor.py
   ```

   Watch `engaged`, `v`, and the `hccc=` accel contribution.
4. Drive: engage openpilot as usual. With the virtual lead publishing, hCCC tracks
   the scenario profile.
5. End the run: Ctrl-C the virtual lead (ego stops commanding accel within about
   100 ms), disengage, park.

Useful `virtual_lead.py` options:

```
--scn N            scenario column of tools/sim/lib/Scenarios.csv (required)
--start_delay S    publish the initial speed for S seconds before the profile moves
--end hold|stop    after the profile: keep final speed forever, or stop publishing
--loop             wrap around to t=0 at the end of the profile
--duration S       hard time limit for the whole run
--max_speed_mph M  scale the whole profile down so its top speed is at most M mph
                   (proportional, accelerations scale too; use 20 for a first run)
--speed_scale F    multiply the profile's speeds/accelerations by F directly
--log_csv PATH     record every sent packet (pairs with plot_v2v_run.py --sent)
--relay_host/--relay_port/--device_id/--hz   override the params/defaults
```

### After the run: pull data and make graphs

```bash
scp comma@10.42.0.1:/data/hcc_v2v_logs/relay_*.csv .
scp comma@<lead-ip>:/data/hcc_v2v_logs/sent_*.csv .

python3 tools/hcc_v2v/plot_v2v_run.py relay_<ts>.csv --sent sent_<ts>.csv
```

This prints packet count, loss %, achieved rate, and inter-arrival gap stats, and
writes a PNG with three panels: v_lead, a_lead, and inter-arrival time (with the
100 ms staleness threshold marked). The ego's own driving response is in the
normal openpilot route logs on the ego. Every relay restart and every
`virtual_lead.py` invocation makes a fresh timestamped CSV, so runs never
overwrite each other.

### Phase 2: two cars

Identical setup, except the lead device goes in a second car on the car harness
(so it measures real speed/accel) running the `hcc-lead` branch, whose
`v2vpublisher` process replaces `virtual_lead.py`. The hotspot, relay, params,
logging, and analysis are unchanged, which is the point of phase 1.

## Simulation (everything on one PC)

See [`tools/sim/README.md`](../sim/README.md). The same relay and wire format run
on the PC at `127.0.0.1:19090`, with simulated openpilot instances as lead and
ego. The launcher UI starts all of it:

```bash
./tools/hcc_v2v/launch_ui.sh
```

## Relay CSV format

One row per packet seen by the relay:

| column | meaning |
|---|---|
| `recv_wall_time_us` | relay receive time (us since epoch) |
| `event_type` | `hello` (registration) or `data` |
| `role` / `device_id` | sender identity |
| `source_host/port`, `dest_host/port` | UDP addresses |
| `seq`, `a_lead`, `v_lead` | packet contents (data packets) |
| `forwarded` | `True` if relayed to the ego |
| `reason` | `forwarded`, or why not (`missing_ego_peer`, `duplicate_or_replayed_seq`, ...) |

`missing_ego_peer` rows at the start of a run are normal: the lead started
publishing before the ego's subscriber sent its hello.

## Troubleshooting

- `virtual_lead` runs but the ego never engages on HC3: check `hcc_monitor.py` on
  the ego. If `hccc=+0.00` while engaged, the V2V signal is not arriving. Check
  `systemctl status hcc-v2v-relay`, then the newest `/data/hcc_v2v_logs/relay_*.csv`
  `reason` column.
- Relay CSV shows only `missing_ego_peer`: the ego openpilot is not running with
  `HCCV2VEnabled=1`, so no subscriber registered. Re-run the ego setup script and
  restart openpilot.
- Relay CSV shows `sender_address_mismatch`: the lead's IP changed mid-run (WiFi
  reconnect). Restart `virtual_lead.py`; it re-registers within a second.
- Lead SSH drops when joining the hotspot: expected (one WiFi radio). Reconnect
  through the ego's hotspot network instead of lab WiFi.
