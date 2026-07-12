# HC3 — Technical Overview

*Human-in-the-loop Cooperative Cruise Control on openpilot.*
Engineering reference: the algorithm, the system, and every change we made to
stock openpilot — where, and why. Companion to the non-technical framing; this
document assumes you read code.

- **Base:** openpilot 0.10.4 (fork point `de024fd4`). `opendbc` and `panda`
  submodules are **unmodified** — the car port and safety firmware are stock.
- **Working branch:** `hcc-ego`. Phase-1 milestone tagged `phase1-complete`.
- **Everything HC3 lives in ~15 changed core files + a `tools/hcc_v2v` and
  `tools/real_world_testing` tree.** No change touches vehicle safety firmware.

---

## 1. What HC3 is

In cooperative cruise control the *ego* car follows a *lead* car using the
lead's **broadcast speed and acceleration** (vehicle-to-vehicle radio), not its
own radar. Because the ego reacts to what the lead *transmits* — including the
lead's acceleration *before* a speed gap even opens — a string of HC3 cars
damps disturbances instead of amplifying them (string stability), which
radar-only adaptive cruise cannot do. "Human-in-the-loop" means the driver's
pedal input is meant to blend on top of the automated command rather than only
disengage it.

Our job was to take a controller developed and tuned in the BeamNG simulator
and run it on a real car through openpilot, over a real V2V radio link, with a
*virtual* lead (a replayed scenario) so a single car can be tested safely
before a two-car test.

### The data path (identical at every test rung)

```
lead source ──UDP──► relay (on the ego device) ──UDP──► ego subscriber
                                                              │
                                                     hCCC controller
                                                              │
                                                   openpilot longitudinal
                                                              │
                                                     SCC accel → wheels
```

Only *who plays lead* changes between environments: a simulated openpilot
instance (PC sim), `virtual_lead.py` replaying a scenario CSV (in-car phase 1),
or a second real car (phase 2). The ego treats all three identically.

---

## 2. The algorithm — cascade hCCC controller

File: [`selfdrive/controls/lib/hccc_controller.py`](selfdrive/controls/lib/hccc_controller.py).
Ported from the mentor's BeamNG reference
([`hccintegration/hCCC_controller_old.py`](hccintegration/hCCC_controller_old.py));
our production controller matches it to machine precision (verified 1.7e-16 over
400 steps) apart from the openpilot-specific output mapping.

**Structure: a two-stage cascade.** Stage 1 decides *what speed I want*; stage 2
is a PID that *chases that speed*.

### Stage 1 — cooperative desired-speed law

```
v_des = v_ego + beta·(v_lead − v_ego) + FF(a_lead),   clamped ≥ 0
```

- `beta = 0.65` — aim to close 65 % of the speed gap (partial convergence is the
  string-stability knob; closing 100 % each tick makes followers amplify
  oscillation).
- `FF(a_lead)` — the **feedforward**, and the reason V2V exists: it feeds the
  lead's *transmitted* acceleration through a lead-lag compensator
  `F(s) = (τ·s + (1 − β·th_bar)) / (th_bar·s + 1)` with `τ = 0.12`,
  `th_bar = 1.5`. The `τ·s` numerator zero gives an *immediate* response to the
  lead *changing* its acceleration — information a radar follower doesn't have.
  With `th_bar = 1.5` the DC gain is `1 − 0.65·1.5 = 0.025`, so a *constant* lead
  acceleration contributes almost nothing sustained (that shows up as a growing
  speed gap, which the `beta` term already handles). Feedforward for
  anticipation, feedback for correction — no double-counting.
- The spacing term from the reference (`0.4·spacing_err`) is **intentionally
  absent**: it needs a measured gap, and the V2V packet carries only speed and
  acceleration (no position). This matches the reference's own non-spacing
  variant.

The filter is discretized with the bilinear (Tustin) transform, but we compute
the coefficients in **closed form** in `_feedforward_coeffs()` — no scipy
dependency in controlsd.

### Stage 2 — PID speed tracker

Error `= v_des − v_ego`, gains `K_P = 0.35`, `K_I = 0.05`, `K_d = 0.02 s`,
output clipped to `[−1, +1]`.

- **Integral** eliminates steady-state tracking error (a real car has lag, drag,
  hills; P alone settles *near* the target). Anti-windup: the integral is clamped
  so a saturated phase can't bank unlimited "revenge acceleration."
- **Derivative** damps the correction. Ported as a *per-second* gain
  (`K_d = 0.02 s`) so it behaves identically at our 100 Hz as it did at the
  reference's 10 Hz — a literal per-step copy would have been 10× too weak.
- **Low-speed attenuation:** below `v_des = 10 m/s` the command is scaled by
  `v_des/10`, so pull-away and parking-lot speeds are proportionally gentle.

### Stage 3 — into openpilot

The `[−1, +1]` pedal fraction is scaled by **`_OUTPUT_ACCEL_SCALE = 3.0`** m/s²
(the reference design's own ±3 m/s² authority envelope), then clamped again by
the planner's per-tick accel limits. This scale is the successor to the older
`0.6` output scale — a single, principled tuning knob if the ride needs to be
gentler.

### Signal-loss behavior (grace before reset)

`run_step()` returns `None` when the lead signal is invalid → openpilot commands
zero → the car coasts ("silence means do less"). Crucially, a *brief* dropout
zeroes the output **without** wiping the controller: the PID integral and
feedforward state survive up to `_RESET_AFTER_INVALID_S = 1.0 s` of continuous
invalidity; only sustained loss triggers a full `reset()`. (This is the fix to
field bug #3 below — the original code reset every tick the signal was invalid.)

**Verification:** `selfdrive/controls/tests/test_hccc_controller.py` — reference
parity at the reference dt, dt-invariance at 100 Hz, integral clamp, low-speed
attenuation, integral convergence, standstill safety, and the grace-vs-reset
behavior.

---

## 3. The V2V transport layer

Wire format, buffer, publisher/subscriber, and relay routing:
[`selfdrive/controls/lib/vendor/hcc_v2v_core.py`](selfdrive/controls/lib/vendor/hcc_v2v_core.py)
(pure stdlib, importable off-device);
[`hcc_v2v_relay.py`](selfdrive/controls/lib/vendor/hcc_v2v_relay.py) is the UDP
server. [`selfdrive/controls/lib/hcc_v2v.py`](selfdrive/controls/lib/hcc_v2v.py)
reads openpilot params/env into a `V2VConfig`.

- **UDP, ~50 Hz.** Late data is worse than lost, so there's no retransmission;
  silence degrades gracefully to "do less."
- **Relay** on the ego registers exactly one lead + one ego by role/device-id
  and forwards lead → ego, logging every packet (forwarded, or the reason it
  wasn't) to CSV.
- **Staleness:** the subscriber marks the signal invalid if the newest packet is
  older than `DEFAULT_STALE_THRESHOLD_MS = 300 ms` (was 100 ms — see bug #3).
- **Clock skew:** packets whose sender timestamp is >`500 ms` off the ego clock
  are rejected. The lead has no internet on the test hotspot and its clock
  drifts, so field procedure syncs the lead to the ego before each session.
- **Session/replay guard:** the buffer drops `seq ≤ last_seq` as replays — but a
  *large* backward jump (> `SESSION_RESET_SEQ_GAP = 50`) is a publisher restart
  (every run renumbers from 0) and is adopted as a new session (bug #4).

---

## 4. Every change to stock openpilot — where and why

### 4.1 New HC3 modules (additive, gated behind params)

| File | Role |
|---|---|
| `selfdrive/controls/lib/hccc_controller.py` | the cascade controller (§2) |
| `selfdrive/controls/lib/hcc_v2v.py` | param/env → `V2VConfig` loader |
| `selfdrive/controls/lib/vendor/hcc_v2v_core.py` | wire format, buffer, pub/sub |
| `selfdrive/controls/lib/vendor/hcc_v2v_relay.py` | UDP relay server |
| `tools/hcc_v2v/` | relay, `virtual_lead.py`, `hcc_monitor.py`, `setup_v2v_network.sh`, plotting, PC-sim launcher |
| `tools/real_world_testing/` | `field_test.py` orchestrator, UI, docs, field reports |

**Params** (`common/params_keys.h`): `EnableHCCC` (master switch),
`HCCV2VEnabled`, `HCCV2VOnly`, `HCCV2VDeviceId`, `HCCV2VRelayHost`,
`HCCV2VRelayPort`. None of the new logic activates until these are set.

### 4.2 Longitudinal control — the integration point

[`selfdrive/controls/lib/longcontrol.py`](selfdrive/controls/lib/longcontrol.py)
is the most substantive modification. **Stock openpilot's longitudinal PID that
tracks the planner's `a_target` is removed.** The output is now:

```
output_accel = controller_accel (hCCC) + manual_accel
```

- When `EnableHCCC`, the hCCC command is the longitudinal output; the planner's
  `a_target` is recorded as debug only, never applied. This is by design — HC3
  *is* the longitudinal controller in V2V-only mode.
- `manual_accel` is the human-in-the-loop blend term. **On Hyundai it is
  structurally 0.0**: the carstate never reports pedal *position*, only
  pressed/not-pressed booleans, so the additive blend has no input signal on this
  platform (the pedals still act physically — gas overrides at the powertrain,
  brake cuts at the panda). Documented as a platform dependency.
- `_v2v_only_enabled()` selects the V2V signal over radar and runs the standstill
  state machine (bug #5).

[`selfdrive/controls/controlsd.py`](selfdrive/controls/controlsd.py):
- Subscribes to `radarState` and passes `radar_state.leadOne` + the V2V snapshot
  into `LoC.update()`.
- **`CC.latActive = False`** — steering is hard-disabled. HC3 is a longitudinal
  study; the driver steers. Intentional and always-on.
- `CC.longActive` is extended so hCCC can engage even when
  `openpilotLongitudinalControl` alone wouldn't.
- The `up/ui/ufAccelCmd` telemetry fields are repurposed to carry
  planner/hCCC/manual accel so the monitor/CSV can log them without new schema.

### 4.3 Supporting core changes

- **`selfdrive/car/car_specific.py`** — `gasPressedOverride` event is suppressed
  when `EnableHCCC` is on, so a gas press blends rather than disengages. *Gated.*
- **`selfdrive/selfdrived/selfdrived.py`** — three changes, all always-on:
  - the `pedalPressed` software disengage block is removed (brake still cuts
    actuation at the panda; **CANCEL is the clean disengage** on this build);
  - FCW/AEB-family events are filtered at the openpilot layer
    (`COLLISION_ALERT_EVENTS`);
  - **`accFaulted` is suppressed while `gasPressed`** (bug #6).
- **`selfdrive/controls/lib/longitudinal_planner.py`** — `allow_throttle` forced
  `True`. Inert during V2V-only (the planner's output isn't applied), affects
  only non-HC3 driving.
- **`selfdrive/controls/radard.py`** — falls back to the nearest forward radar
  track when lead-matching fails. Inert in V2V-only (radar lead is `None`).
- **`selfdrive/car/card.py`** — suppresses `liveTracks` publication under the
  `SIMULATION` env only; no effect on a real car.
- **`system/manager/{manager,process_config}.py`** — msgq bootstrap ordering,
  `BLOCK`/`OPENPILOT_PREFIX` env handling, and a HIL sim-bridge process gated on
  `HIL_MODE`. Infrastructure for the sim/HIL workflows.
- **`selfdrive/ui/.../developer.py`** — settings entries for the HC3 toggles.

> **Design note on the always-on changes.** Several (steering-off, FCW/AEB
> filter, planner throttle, radard fallback) originated for the sim workflow but
> are not gated behind `SIMULATION`, so they change behavior on a real car. They
> are deliberate for this research build; the security-relevant consequence is
> that **CANCEL, not brake, is the software disengage**, and **openpilot has no
> obstacle avoidance in V2V-only mode** — the driver is the collision-avoidance
> system (the car's factory AEB remains as a hardware backstop). This is why
> field procedure mandates a closed/empty road and a fully attentive driver.

---

## 5. Real-world testing

### 5.1 The test ladder

1. **PC simulation** — relay + wire format + both openpilot instances on one PC
   at `127.0.0.1`. Proves the software loop.
2. **Bench** — two Comma 3X devices on a desk over WiFi, `--bench` starts a
   stand-in ego subscriber (`bench_ego.py`) since the real subscriber only runs
   onroad. Proves transport at 50 Hz / 0 % loss. *Passed 2026-06-12.*
3. **Phase 1 in-car** — one car: ego on the harness driving, lead USB-powered in
   the cabin replaying a scenario, laptop orchestrating on the ego's hotspot.
   No `--bench` (the real subscriber lives in controlsd). *Complete 2026-07-11.*
4. **Phase 2** — two cars; the lead runs a publisher instead of `virtual_lead`.
   Everything else is unchanged — that is the point of phase 1.

### 5.2 Tooling

- **`field_test.py`** — laptop orchestrator: preflight (14 checks: SSH, relay,
  params, scenario CSV, connectivity, **clock skew**), remote scenario
  start/stop over SSH, and automatic collection of every artifact into one
  timestamped `runs/<id>/` folder with stats + plots.
- **`virtual_lead.py`** — replays a `Scenarios.csv` column as V2V packets;
  `--max_speed_mph` / `--speed_scale` scale a profile down for a first run,
  `--start_delay` holds the initial speed so the driver can engage.
- **`hcc_monitor.py`** — per-sample research record now captures the full causal
  chain: `hccc_accel` (controller) → `cmd_accel` (decision) → `out_accel` (wire)
  → `a_ego_mps2` (achieved), plus `gas/brake_pressed` and `standstill`.

### 5.3 What each run folder contains

`metadata.json` (run params, times, notes) · `sent.csv` (what the lead sent) ·
`relay.csv` (what the relay saw + forward/reject reasons) · `ego_monitor.csv`
(the ego response at 10–20 Hz, all columns above) · `stats.txt` (loss %, rate,
jitter) · `plot.png`. `relay.csv` and `ego_monitor.csv` share the ego clock, so
they join directly on time: lead command vs. ego response on one axis.

### 5.4 Results

**Phase 1 is complete** (2026-07-11, 2024 Hyundai Tucson): the ego drove the
full scenario-48 profile under HC3 — engaged via SET, feet off, tracking the
transmitted speed profile through acceleration, a mid-profile slowdown,
re-acceleration, and final braking, to a **14.66 m/s (32.8 mph)** peak, with
commanded vs. achieved acceleration matching ~1:1 (`cmd +1.33 → aEgo +1.31`;
`cmd −1.33 → aEgo −1.27`). Scenarios 49 and 50 also ran. Canonical dataset:
[`tools/real_world_testing/runs/scn48_THIS_WORKED/`](tools/real_world_testing/runs/scn48_THIS_WORKED/).
All run data is committed for provenance.

---

## 6. Field bugs found and fixed

The campaign surfaced six real bugs. **None were in the controller** — the
cascade hCCC behaved correctly whenever its input was valid.

| # | Bug | Root cause | Fix |
|---|---|---|---|
| 1 | Car unrecognized (MOCK/dashcam) on a supported Tucson | 2024 camera FW (`99211-CW020`) was catalogued under the Santa Cruz, not the Tucson, in the pinned opendbc | Appended the string to the Tucson entry on the device (upstream-worthy; missing in comma master too). **Device-only edit.** |
| 2 | SET didn't engage | button-message decoding on the facelift; short-lived diagnostic watchers gave false readings | Resolved in the field; verify config next session |
| 3 | Car crawled at 3–5 mph | subscriber thread starved ~100 ms once/sec → 100 ms staleness tripped every second → `reset()` wiped the PID integral/feedforward every second | Threshold → 300 ms; **grace-before-reset** in the controller (`5a0ce012b`) |
| 4 | Every *second* run of a session was deaf | replay guard dropped `seq ≤ last_seq` forever, but each publisher renumbers from 0 | Adopt a new session on a large seq regression (`5a7d9a543`, tests) |
| 5 | Car ignored positive accel after a full stop | HKG standstill latch; V2V-only forced `long_state = pid` and never sent the `stopReq`/release handshake | Derive `should_stop` from the V2V target, run the real stopping/starting state machine (`5a0ce012b`) |
| 6 | Gas press disengaged HC3 | the Tucson TCS reports `ACCEnable != 0` during a gas override → `accFaulted` → immediate disable | Suppress `accFaulted` while `gasPressed` (`5a0ce012b`; hypothesis, confirm at car) |

**Diagnostic lessons recorded for future sessions:** `SubMaster.update(t)`
returns on every message, *not* after `t` ms — wall-clock-time all live watchers
(two wrong verdicts came from this). The swaglog `hcc_v2v_mode_transition`
history is the subscriber's ground truth. The injection probe
(`hccc → cmd → out → aEgo`) settles "controller vs. car" questions in one run.

---

## 7. Status and what's next

- **Done:** PC sim ✓, bench ✓ (50 Hz, 0 % loss), **phase-1 in-car ✓** (32.8 mph
  full-profile tracking), cascade controller validated on hardware, six field
  bugs fixed or cornered.
- **Committed but not yet on the devices:** the three bug-5/3/6 fixes
  (`5a0ce012b`). Next session starts by pulling this and rebooting the ego.
- **Day-before-next-run checklist** (in
  [`field_report_20260711.md`](tools/real_world_testing/field_report_20260711.md)):
  reconcile the ego with git and **re-apply the fingerprint camera-FW edit**
  (device-only, wiped by `git checkout`); confirm the button config; confirm the
  `accFaulted`-on-gas hypothesis with the watcher at the car.
- **Rollback:** the pre-fix, field-proven state is tagged `phase1-complete`.
- **Phase 2:** two cars. Hotspot, relay, params, logging, analysis, and the
  controller are all unchanged — the lead simply measures a real car instead of
  replaying a scenario.

---

## Appendix — file map

| Path | What |
|---|---|
| `selfdrive/controls/lib/hccc_controller.py` | cascade controller (the algorithm) |
| `selfdrive/controls/lib/longcontrol.py` | integration: hCCC replaces the stock long PID |
| `selfdrive/controls/lib/hcc_v2v.py` | param/env → V2VConfig |
| `selfdrive/controls/lib/vendor/hcc_v2v_core.py` | wire format, buffer, pub/sub |
| `selfdrive/controls/lib/vendor/hcc_v2v_relay.py` | UDP relay server |
| `selfdrive/controls/controlsd.py` | wiring + steering-off + telemetry |
| `selfdrive/selfdrived/selfdrived.py` | disengage/alert policy (CANCEL, FCW/AEB, gas) |
| `selfdrive/car/car_specific.py` | gas-override event gating |
| `tools/hcc_v2v/` | relay, virtual lead, monitor, setup, plotting |
| `tools/real_world_testing/` | field orchestrator, reports, command sheet |
| `hccintegration/` | mentor's BeamNG reference controllers |
```
