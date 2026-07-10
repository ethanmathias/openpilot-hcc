# HCC V2V — relay, virtual lead, and real-world testing

> Non-technical overview of the whole project (what, why, every design
> decision): [`HCC_PROJECT_GUIDE.md`](../../HCC_PROJECT_GUIDE.md).

This directory holds everything for the V2V (vehicle-to-vehicle) leg of HC3:
the UDP relay, the tools that publish and analyze lead data, and the
procedure for in-car testing.

**New here? Read this first.** In HC3, the *ego* car follows a *lead* car
using speed/acceleration data the lead transmits over WiFi — not radar.
The data path is always the same three hops:

```
lead publisher ──UDP──► relay (runs ON the ego device) ──UDP──► ego subscriber (inside openpilot)
```

The lead publisher can be:
- a real lead device measuring a real car (two-car field test),
- **`virtual_lead.py` replaying a scenario CSV (phase-1 in-car test — see below)**,
- a simulated openpilot instance (PC simulation, see [`tools/sim/README.md`](../sim/README.md)).

The ego's hCCC controller treats all three identically.

---

## What's in this directory

| File | What it does | Where it runs |
|---|---|---|
| `relay_server.py` | UDP relay: registers one lead + one ego, forwards lead packets to the ego, optionally logs every packet to CSV | Ego device (as a systemd service), or PC in simulation |
| `virtual_lead.py` | Replays a `Scenarios.csv` speed profile as V2V packets — a fake lead car | Lead device (or any machine that can reach the relay) |
| `plot_v2v_run.py` | Stats + PNG plots from a relay CSV log (loss, rate, speed/accel traces) | Your PC, after a run |
| `scripts/setup_v2v_network.sh` | One-time setup: ego hotspot + relay service + params; lead joins hotspot + params | Each device, once |
| `scripts/hcc_monitor.py` | Live one-line-per-second status: engaged, speed, HC3/manual/planner accel | Either device, during a run |
| `launch_ui.sh` / `launcher_ui.py` | Tk launcher for the all-on-PC simulation workflow | PC |

Wire format and subscriber/publisher logic live in
`selfdrive/controls/lib/vendor/hcc_v2v_core.py`; the ego's controller reads
the V2V signal through `selfdrive/controls/lib/longcontrol.py`.

---

## Phase-1 in-car test: one car, virtual lead

**Goal:** prove the full V2V control loop in a real car with minimal risk.
The ego device is installed normally in the car and drives it. The lead
device sits **in the same car** (powered by USB, mounted anywhere), joined to
the ego's WiFi hotspot, replaying a scenario speed profile. The ego follows
the *virtual* lead car exactly as it would a real one.

```
            ┌────────────────────── one real car ──────────────────────┐
            │                                                          │
            │  Comma 3X "lead" (USB power only)                        │
            │    virtual_lead.py --scn 48                              │
            │        │ WiFi (ego's hotspot, 10.42.0.1)                 │
            │        ▼                                                 │
            │  Comma 3X "ego" (car harness, drives the car)            │
            │    relay (systemd) → openpilot hCCC → gas/brake          │
            └──────────────────────────────────────────────────────────┘
```

### Safety notes — read before driving

- In V2V-only mode (`HCCV2VOnly=1`) the ego follows the **transmitted**
  profile. There is **no physical car ahead** — the ego will accelerate on an
  empty road to track the virtual lead's speed. Pick a closed/empty road and
  a scenario whose top speed fits it (scenario 48 peaks at **14.4 m/s ≈ 32 mph**).
- The driver can always take over: brake pressure disengages openpilot
  normally, and manual pedal input is blended on top of HC3 output.
- If the lead device stops publishing (crash, WiFi drop, Ctrl-C), the ego
  marks the V2V signal stale within ~100 ms and stops commanding
  acceleration. `virtual_lead.py --end stop` uses this deliberately at the
  end of a profile; `--end hold` keeps publishing the final speed instead.
- Keep the phone/PC SSH session to the lead device alive through the ego's
  hotspot, or start `virtual_lead.py` under `tmux`/`nohup` so it survives a
  dropped SSH session.

### One-time setup (per device)

Both devices need the `hcc-ego` branch checked out at `/data/openpilot`
(the lead device only runs `virtual_lead.py`, so it does not need the
`hcc-lead` branch for this test).

**Ego device** (SSH in over your normal WiFi):

```bash
# 1. Hotspot + relay service + V2V params (HCCV2VEnabled/Only, relay host/port)
sudo /data/openpilot/tools/hcc_v2v/scripts/setup_v2v_network.sh ego

# 2. HC3 itself (the V2V script does not set these)
python3 -c "
from openpilot.common.params import Params
p = Params()
p.put_bool('EnableHCCC', True)
p.put_bool('AlphaLongitudinalEnabled', True)
"
```

The relay service starts immediately, restarts on boot, and logs every
packet to `/data/hcc_v2v_logs/relay_<timestamp>.csv` (a new file per
service start).

**Lead device** (after the ego hotspot exists — note your SSH over regular
WiFi will drop when it switches networks):

```bash
sudo /data/openpilot/tools/hcc_v2v/scripts/setup_v2v_network.sh lead
```

This joins the `hcc-v2v` hotspot and sets `HCCV2VRelayHost` to the ego's
hotspot IP (10.42.0.1). From now on, SSH to the lead through the ego's
hotspot network.

**Verify the link** (from the lead):

```bash
ping -c 3 10.42.0.1                      # ego hotspot reachable
```

> **Prefer the orchestrator.** The manual steps below work, but
> [`tools/real_world_testing/`](../real_world_testing/README.md) automates
> all of them from a laptop on the ego's hotspot: preflight checks (params,
> relay, clock skew), remote start/stop of the scenario, and automatic
> collection of every artifact into a per-run folder with stats and plots.

### Test-day run procedure

1. **Ego**: boot normally in the car (normal openpilot UI — no special
   launcher). Confirm the relay is up:
   `systemctl status hcc-v2v-relay`.
2. **Lead**: power it from USB in the same car. SSH in via the hotspot and
   start the virtual lead:

   ```bash
   cd /data/openpilot
   tmux new -s lead
   python3 tools/hcc_v2v/virtual_lead.py --scn 48 --log_csv /data/hcc_v2v_logs/sent_$(date +%Y%m%d_%H%M%S).csv
   ```

   It prints one status line per second (`t=… v=… a=… seq=…`). Relay
   host/port/device-id come from the params the setup script wrote — no
   flags needed.
3. **Optional live monitor** on the ego (second SSH session):

   ```bash
   cd /data/openpilot && python3 tools/hcc_v2v/scripts/hcc_monitor.py
   ```

   Watch `engaged`, `v`, and the `hccc=` accel contribution.
4. **Drive**: engage openpilot as usual. With the virtual lead publishing,
   hCCC tracks the scenario profile.
5. **End the run**: Ctrl-C the virtual lead (ego stops commanding accel
   within ~100 ms), disengage, park.

Useful `virtual_lead.py` options:

```
--scn N            scenario column of tools/sim/lib/Scenarios.csv (required)
--start_delay S    publish the initial speed for S seconds before the profile moves
--end hold|stop    after the profile: keep final speed forever, or stop publishing
--loop             wrap around to t=0 at the end of the profile
--duration S       hard time limit for the whole run
--max_speed_mph M  scale the whole profile down so its top speed is at most M mph
                   (proportional, accelerations scale too — e.g. 20 for a first run)
--speed_scale F    multiply the profile's speeds/accelerations by F directly
--log_csv PATH     record every sent packet (pairs with plot_v2v_run.py --sent)
--relay_host/--relay_port/--device_id/--hz   override the params/defaults
```

### After the run: pull data and make graphs

```bash
# From your PC (on the ego's hotspot, or back on shared WiFi):
scp comma@10.42.0.1:/data/hcc_v2v_logs/relay_*.csv .
scp comma@<lead-ip>:/data/hcc_v2v_logs/sent_*.csv .

python3 tools/hcc_v2v/plot_v2v_run.py relay_<ts>.csv --sent sent_<ts>.csv
```

This prints packet count, loss %, achieved rate, and inter-arrival gap
stats, and writes a PNG with three panels: v_lead, a_lead, and inter-arrival
time (with the 100 ms staleness threshold marked). The ego's own driving
response (vEgo vs. the profile) is in the normal openpilot route logs on the
ego device.

Repeat runs are cheap: every relay restart and every `virtual_lead.py`
invocation makes a fresh timestamped CSV, so runs never overwrite each other.

### Phase 2: two cars

Identical setup, except the lead device goes in a second car on the car
harness (so it measures real speed/accel) running the `hcc-lead` branch,
whose `v2vpublisher` process replaces `virtual_lead.py`. The hotspot, relay,
params, logging, and analysis are unchanged — that is the point of phase 1.

---

## Simulation (everything on one PC)

See [`tools/sim/README.md`](../sim/README.md). The same relay and the same
wire format run on the PC at `127.0.0.1:19090`, with simulated openpilot
instances as lead and ego. The launcher UI starts all of it:

```bash
./tools/hcc_v2v/launch_ui.sh
```

## Relay CSV format

One row per packet seen by the relay:

| column | meaning |
|---|---|
| `recv_wall_time_us` | relay receive time (µs since epoch) |
| `event_type` | `hello` (registration) or `data` |
| `role` / `device_id` | sender identity |
| `source_host/port`, `dest_host/port` | UDP addresses |
| `seq`, `a_lead`, `v_lead` | packet contents (data packets) |
| `forwarded` | `True` if relayed to the ego |
| `reason` | `forwarded`, or why not (`missing_ego_peer`, `duplicate_or_replayed_seq`, …) |

`missing_ego_peer` rows at the start of a run are normal — the lead started
publishing before the ego's subscriber sent its hello.

## Troubleshooting

- **`virtual_lead` runs but the ego never engages on HC3** — check
  `hcc_monitor.py` on the ego: if `hccc=+0.00` while engaged, the V2V signal
  isn't arriving. Check `systemctl status hcc-v2v-relay`, then look at the
  newest `/data/hcc_v2v_logs/relay_*.csv` `reason` column.
- **Relay CSV shows only `missing_ego_peer`** — the ego openpilot isn't
  running with `HCCV2VEnabled=1`, so no subscriber ever registered. Re-run
  the ego setup script and restart openpilot.
- **Relay CSV shows `sender_address_mismatch`** — the lead's IP changed
  mid-run (WiFi reconnect). Restart `virtual_lead.py`; it re-registers
  within a second.
- **Lead SSH drops when joining the hotspot** — expected: one WiFi radio.
  Reconnect through the ego's hotspot network instead of the lab WiFi.
