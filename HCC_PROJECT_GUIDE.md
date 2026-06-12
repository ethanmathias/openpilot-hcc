# HC3 Project Guide — how the codebase is organized and why

*Written for readers who know the research domain well but don't write
software. It explains how the hCCC/V2V work is implemented and tested, and
the reasoning behind each engineering decision. Every file mentioned is a
clickable link; each section starts with the short version and adds detail
below it.*

---

## Part 1 — The implementation, in one page

The controller is built on [openpilot](https://github.com/commaai/openpilot),
an open-source driver-assistance system that runs on a
[Comma 3X](https://comma.ai/shop/comma-3x) (a windshield-mounted device)
and actuates gas, brake, and steering on supported production cars. Our
codebase is a modified copy of openpilot — a "fork" — with hCCC and the V2V
data path added. **Why build on openpilot:** it provides production-grade
vehicle interfaces, a vetted safety model, and logging/replay tooling for
free, so our code stays focused on the cooperative controller itself rather
than on talking to a car.

The V2V leg always has the same shape, in every environment we test in —
simulation, desk, or car:

```
lead publisher ──UDP──► relay ──UDP──► ego subscriber ──► hCCC ──► gas/brake
```

The lead's speed and acceleration are sent 50 times per second as small
network packets. A small forwarding program — the **relay** — sits in the
middle ([why a relay exists](#why-is-there-a-relay-at-all)). On the ego
side, the subscriber hands the lead state to hCCC, which computes the
ego's acceleration command.

What varies between environments is only *who plays the lead* (a simulated
vehicle, a replayed speed profile, eventually a real instrumented car) and
*where the relay runs* (the PC, the ego device, a cloud server). The code
is the same everywhere — that's the core design decision, explained next.

---

## Part 2 — The testing ladder, and why each rung exists

The principle behind the project's structure: **find every defect in the
cheapest, safest place it can possibly be found.** A bug discovered in
simulation costs minutes; the same bug discovered during a live following
maneuver costs far more. So validation climbs a ladder, and nothing moves
up a rung until the rung below passes:

| Rung | What runs | What it validates | Risk |
|---|---|---|---|
| 1. Simulation, sensed lead | one openpilot instance on a PC, simulated world | controller logic and tuning, no V2V | none |
| 2. Simulation, V2V lead | two openpilot instances + relay, one PC | the entire V2V code path | none |
| 3. Simulation, cloud relay | same, relay on a rented server | transport over a real wide-area network | none |
| 4. Bench test | the two real devices on a desk | device-specific behavior | none |
| 5. **Phase-1 in-car** *(next)* | ego drives a real car; the lead device replays a speed profile from inside the same car | the full control loop with real vehicle dynamics | controlled |
| 6. Phase-2 two cars | real lead vehicle transmitting real measurements | everything | controlled |

Two decisions make the ladder meaningful:

**Decision: identical plumbing at every rung.** The publisher, relay, and
subscriber are the *same programs* at every rung; only network addresses
change. **Why:** if each rung used different plumbing, passing rung 2
would prove nothing about rung 5. Because the path is identical, each rung
climbed is transferable evidence about the next.

**Decision: a "virtual lead" for the first car test.** Phase 1 puts both
devices in *one* car: the ego device drives it, while the lead device —
sitting in the cabin on USB power — replays a pre-recorded speed profile
as V2V packets. The ego cannot distinguish this from a real transmitting
lead (same packets, same path). **Why:** it exercises 100% of our code
with one car, one driver, and a perfectly repeatable lead, before any
two-vehicle coordination is attempted. Phase 2 then changes exactly one
thing: where the lead's numbers come from.

The profiles themselves are **scenarios** — columns of
[tools/sim/lib/Scenarios.csv](tools/sim/lib/Scenarios.csv), each a
speed-vs-time trace. Our standard case, scenario 48, ramps to ~14 m/s and
then oscillates. **Why fixed scenarios:** every simulation run, bench run,
and car run can follow the *same* profile, so results are directly
comparable across rungs and across weeks.

### The dead end we kept off the ladder

Between rungs 3 and 4 we attempted **HIL** (hardware-in-the-loop): the
real device driving the *simulated* world over a USB cable. It is
permanently shelved for a hardware reason, not a software one: the Comma
3X's USB-C port is wired to its internal safety microcontroller (the
"panda"), not to its main computer, so the high-bandwidth PC↔device link
HIL requires cannot exist on this hardware. The investigation is recorded
in [tools/sim/hil/README.md](tools/sim/hil/README.md). The bench test
(rung 4) was built as its replacement and caught everything we had hoped
HIL would (see Part 5).

---

## Part 3 — Tour of the code

Two directories, one per environment.

### Simulation — [`tools/sim/`](tools/sim/README.md)

The simulated world is [MetaDrive](https://github.com/metadriverse/metadrive),
an open-source driving simulator. The essential trick: **openpilot does
not know it is in a simulation.** A "bridge" program feeds it synthetic
camera frames and vehicle signals in exactly the format a real car
produces, and applies openpilot's actuation commands back to the simulated
vehicle. **Why this matters:** the code under test is the production code,
unmodified — so simulation results carry over to the car.

| File | What it is | Why it exists |
|---|---|---|
| [run_bridge.py](tools/sim/run_bridge.py) | The bridge: translates simulator ↔ openpilot every tick; also drives the lead vehicle along a scenario | Lets unmodified openpilot drive a simulated car |
| [lib/Scenarios.csv](tools/sim/lib/Scenarios.csv) | The scenario library (one speed profile per column) | Repeatable, comparable runs (Part 2) |
| [launch_openpilot_ego.sh](tools/sim/launch_openpilot_ego.sh) | Starts openpilot configured for simulation | Sets several environment flags nobody should have to remember |
| `launch_openpilot_lead.sh` (on the `hcc-lead` branch) | Same, for the lead instance in two-instance runs | The lead runs as a *separate full openpilot* so its publishing code is also production code; lead-only code lives on its own branch because a real lead device runs that branch |
| [README.md](tools/sim/README.md) | Complete operating instructions | Onboarding without a human in the loop |

The simulation runs in two main configurations: **Mode 1** follows the
lead through the simulated camera/radar pipeline (no V2V — isolates the
controller), and **Mode 2** runs two openpilot instances with the lead
publishing over a local relay (the full V2V path — rung 2). **Why both:**
when a Mode 2 run misbehaves, re-running in Mode 1 immediately separates
"controller problem" from "communication problem."

### V2V components — [`tools/hcc_v2v/`](tools/hcc_v2v/README.md)

The pieces shared by simulation and the real world — the "identical
plumbing" of Part 2.

| File | What it is | Why it exists |
|---|---|---|
| [relay_server.py](tools/hcc_v2v/relay_server.py) | The relay: registers one lead + one ego, forwards lead packets to the ego, logs every packet to a CSV file | [Below](#why-is-there-a-relay-at-all); the log makes every run auditable after the fact |
| [virtual_lead.py](tools/hcc_v2v/virtual_lead.py) | Replays a scenario column as real V2V packets at 50 Hz | The phase-1 virtual lead (Part 2) |
| [bench_ego.py](tools/hcc_v2v/bench_ego.py) | A stand-in receiver that registers as the ego and counts arrivals | On a desk the real subscriber isn't running (it lives inside the driving software, which only runs in a car); this fills the ego's seat so the full path can still be exercised |
| [plot_v2v_run.py](tools/hcc_v2v/plot_v2v_run.py) | Computes packet loss, achieved rate, and timing jitter from a relay log; draws the run | Transport health should be a printed number, not an impression |
| [scripts/setup_v2v_network.sh](tools/hcc_v2v/scripts/setup_v2v_network.sh) | One-time device setup: ego hosts a WiFi hotspot and runs the relay as an always-on service; lead joins the hotspot; both get their configuration | By hand this is ~15 error-prone steps; this is one command per device, safe to re-run |
| [scripts/hcc_monitor.py](tools/hcc_v2v/scripts/hcc_monitor.py) | Live status line on a device — engaged? speed? what acceleration is hCCC commanding? — plus optional recording | During a car test someone must see what the controller is doing *now*; the recording becomes the run's response trace |
| [selfdrive/controls/lib/vendor/hcc_v2v_core.py](selfdrive/controls/lib/vendor/hcc_v2v_core.py) | The packet format and the publisher/subscriber logic | Single source of truth for "what's in a packet," used by everything above |
| [selfdrive/controls/lib/longcontrol.py](selfdrive/controls/lib/longcontrol.py) | Where the received V2V signal meets the longitudinal controller | The integration point — the code that moves the car |

#### Why is there a relay at all?

Why not send packets straight from lead to ego? Three reasons. (1) **One
place to look:** every packet, forwarded or rejected, passes one point
that logs it — after any run we have a complete record of what was sent,
what arrived, and why anything was dropped. (2) **One place to change:**
both ends only ever need the relay's address, so moving between laptop,
in-car, and cloud configurations changes one setting, not the software at
either end. (3) **The target architecture:** the eventual deployment is
over cellular, where two vehicles can't address each other directly but
both can reach a server — the relay *is* that server, present in the
architecture from day one.

#### Why UDP, and the staleness rule

Networking offers two delivery models: TCP (every packet guaranteed,
retried until delivered, in order) and UDP (sent once; occasional loss
possible). We chose UDP deliberately, because for control data **late is
worse than lost**: TCP's guarantee means a delayed packet *blocks* newer
ones behind it, delivering precisely the stale lead state a controller
must not act on. With UDP a lost packet is simply superseded 20 ms later
by the next one. The complementary safety rule: if nothing arrives for
**100 ms**, the subscriber marks the signal stale and hCCC stops
commanding acceleration — the system's response to silence is always "do
less," never extrapolate. (Measured loss on the bench: 0 of 1750 packets;
worst inter-arrival gap 51 ms.)

### Real-world testing — [`tools/real_world_testing/`](tools/real_world_testing/README.md)

The laptop's side of a field test: starting runs remotely, watching them,
and getting every byte of evidence back.

| File | What it is | Why it exists |
|---|---|---|
| [field_test.py](tools/real_world_testing/field_test.py) | The orchestrator: `check` (preflight), `run` (start, watch, collect), `abort` (emergency stop of the data feed), `collect` (recover data), `find-lead` (locate the lead on the network) | Design notes below — nearly every feature encodes a specific failure we hit |
| [field_test_ui.py](tools/real_world_testing/field_test_ui.py) | Point-and-click window over the same commands | In a car, with a driver waiting, typed commands are where mistakes happen |
| [README.md](tools/real_world_testing/README.md) | Procedures: setup, bench rehearsal, the in-car test-day script, safety checklist | The test-day script assumes no internet access (below) and a non-developer operator |

**Design notes on the orchestrator** — each is a decision with a story:

- **A 14-item preflight gate refuses to start a run until everything
  passes**: both devices reachable, relay running, every configuration
  flag correct, scenario file present, log locations writable, and — most
  subtly — the two devices' clocks in agreement. **Why:** each check
  encodes a failure that actually occurred during bring-up, and most fail
  *silently* (packets simply don't arrive; no error appears anywhere).
  The clock check is the flagship case: the subscriber rejects packets
  timestamped more than 500 ms off, and the lead's clock drifts because it
  has no internet to sync from while on the test network — the run would
  record nothing, with every light green. Preflight converts each silent
  failure into a loud, labeled one with its fix printed beside it.
- **Every run lands in one timestamped, never-overwritten folder**: what
  the lead sent, what the relay saw, how the ego responded, console
  output, computed statistics, a plot, and a metadata file with the exact
  configuration and free-text notes. **Why:** a field run costs a road, a
  car, a driver, and an afternoon; its data must survive anything,
  including questions nobody thinks to ask until weeks later.
- **Runs survive a lost laptop connection.** The device-side programs are
  started in a disconnect-proof way; if the laptop's WiFi drops mid-run,
  the run continues and one command (`collect`) recovers the data
  afterwards. **Why:** laptop WiFi in a moving vehicle *will* drop, and
  that must never destroy a run.
- **`abort` exists and is honest about what it is.** It kills the data
  feed; within 100 ms the staleness rule stops hCCC from commanding
  acceleration. It is *not* the emergency stop — the driver's brake is,
  through openpilot's normal instant disengagement. The tooling never
  engages or disengages the vehicle; only the driver does. **Why:**
  layered safety with the human at the top, and documentation that never
  overstates what software guarantees.

#### The constraint that shaped the field tooling: the WiFi island

The devices communicate over a WiFi hotspot the ego device broadcasts.
**Why a hotspot:** the test must work on a closed road with no
infrastructure — no facility WiFi, no assumed cell coverage. The price:
these devices have one WiFi radio each, so a device on the hotspot cannot
simultaneously reach the internet — nor can the laptop while connected to
it. Test days are therefore offline islands: updates, downloads, and clock
synchronization all happen *in advance*, and the test-day procedure is
written to be executed entirely offline. This single constraint explains
the clock-drift problem above and several preflight checks.

---

## Part 4 — The decision log

Every significant engineering choice, the alternative, and the rationale.
(The complete technical record is the version-control history; this is the
readable summary.)

| Decision | Alternative | Why |
|---|---|---|
| Build on openpilot (fork) | Controller from scratch | Production vehicle interfaces, vetted safety model, logging/replay tooling for free; our effort stays on the cooperative controller |
| Identical V2V path at every test rung | Simpler ad-hoc plumbing per environment | Evidence transfers up the ladder (Part 2) |
| Fixed scenario replay | Free driving each test | Repeatability and comparability across weeks and rungs |
| Relay in the middle | Direct lead→ego packets | Auditability, one configuration point, matches the cellular target architecture ([details](#why-is-there-a-relay-at-all)) |
| UDP + 100 ms staleness cutoff | TCP guaranteed delivery | Late data is worse than lost data; silence always degrades toward "do less" ([details](#why-udp-and-the-staleness-rule)) |
| Device-to-device WiFi hotspot | Facility WiFi / cellular | Zero-infrastructure operation anywhere; cellular is a later rung |
| Abandon HIL (USB device↔PC) | Keep debugging it | Hardware fact: the device's USB port doesn't reach its main computer ([record](tools/sim/hil/README.md)) |
| Bench test before any driving | Straight to the car | Caught three real defects at a desk, one of which would have crashed the controller while driving (Part 5) |
| Virtual lead for phase 1 | Start with two cars | Full code path, one car, perfectly repeatable lead (Part 2) |
| Preflight gate before every run | Trust the setup | Each check encodes a real, usually silent, past failure |
| One immutable folder per run | Shared results directory | Field data must survive anything, including late-arriving questions |
| Two-person protocol; driver's brake supreme | Allow solo testing | Layered safety; software is never the last line |

---

## Part 5 — What the bench test caught

In one day of desk testing (2026-06-12, no vehicle involved) the pipeline
went from "fully set up" to "fully validated" in five attempts. The three
defects it surfaced are the best evidence for the ladder methodology:

1. **A crash that would otherwise have occurred while driving.** A
   configuration value was stored as a number where the code expected
   text, crashing the code that reads it — code that also runs inside the
   vehicle control process. Found on the road, it would have shut down the
   controller mid-drive; found on the desk, it cost an afternoon.
2. **Two "failures" that weren't.** A launch step appeared to time out and
   consumed two attempts. The on-device logs later proved both runs had
   executed *perfectly* — only the laptop's method of confirming the
   launch was at fault. The fix is now part of the tooling: launches
   confirm success by reading the started program's ID directly rather
   than trusting the connection's behavior.
3. **A silent permission problem.** A background service had created the
   data-log directory under the administrator account, so the test
   programs (running as a normal user) could not write to it — and they
   failed by *dying instantly while reporting a healthy-looking startup*.
   Also now encoded in the tooling: every launch verifies the program is
   still alive a second later, and preflight checks the directory
   permissions on both devices.

Final bench result: **1750 of 1750 packets delivered at 50 Hz, zero loss,
worst inter-arrival gap 51 ms** against the 100 ms staleness limit. The
complete record, like every run, is one folder:
`tools/real_world_testing/runs/run_20260612_190830_scn48/`.

---

## Part 6 — Status and roadmap

- ✅ Rungs 1–4 pass (both simulation modes, the cloud relay, the bench).
- ⏭️ **Next: the phase-1 in-car test.** The step-by-step procedure,
  preparation list, and safety checklist are in
  [tools/real_world_testing/README.md](tools/real_world_testing/README.md).
- 🔭 Then phase-2 (two vehicles; the real lead-side publisher already
  exists on the `hcc-lead` branch and is exercised in simulation Mode 2),
  and eventually the cellular-relay rung.

### If you only read three documents

1. This one.
2. [tools/real_world_testing/README.md](tools/real_world_testing/README.md)
   — how a car test actually runs, including the safety protocol.
3. [tools/sim/README.md](tools/sim/README.md) — how the simulations run.

### Software terms used in this guide

| Term | Meaning |
|---|---|
| openpilot | the open-source driver-assistance system we build on |
| fork / branch | a modified copy of a codebase / a parallel line of work within it (`hcc-ego` is the main branch; `hcc-lead` carries lead-device-only code) |
| Comma 3X | the windshield-mounted device that runs openpilot |
| panda | the device's internal safety microcontroller |
| relay | the small forwarding program between lead and ego |
| bridge | the program connecting the simulator to openpilot |
| MetaDrive | the open-source driving simulator |
| UDP / TCP | send-once vs. guaranteed-delivery network protocols ([why we use UDP](#why-udp-and-the-staleness-rule)) |
| staleness rule | after 100 ms without data, hCCC stops commanding acceleration |
| preflight | the automated checks that gate every test run |
| bench test | full-pipeline rehearsal on real devices, no vehicle |
| AGNOS | the Comma 3X's operating system |
