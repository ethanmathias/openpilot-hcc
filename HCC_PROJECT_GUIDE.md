# HC3 Project Guide — how this codebase works and why it's built this way

*Written for readers who are not software engineers. Every section starts
with the plain-English version and then adds detail; you can stop reading
any section as soon as you have what you need. Every file mentioned is a
clickable link.*

---

## Part 1 — What we're building, in one page

Standard adaptive cruise control follows the car ahead using **sensors**
(radar and cameras) that measure where that car is *right now*. Our
controller, **hCCC** (the project is "HC3"), instead follows the car ahead
using data that car **transmits**: its speed and acceleration, sent over a
network many times per second. This is called **V2V**
(vehicle-to-vehicle) communication.

Why bother? A sensor can only tell you what the lead car has *already
done* — by the time radar sees the lead braking, it has been braking for a
while. A transmission tells you what the lead is doing *the instant it does
it*, which allows smoother, tighter, more efficient following, and it works
when sensors can't see (curves, weather, a truck between you).

The data path is always the same three hops, in every test we run:

```
lead publisher  ──UDP packets──►  relay  ──UDP packets──►  ego subscriber
(the car ahead,                 (a tiny                  (inside openpilot,
 or something                    forwarding               feeding hCCC the
 pretending to be it)            program)                 lead's speed)
```

- **ego** — the car our software is driving.
- **lead** — the car ahead being followed.
- **relay** — a small program that sits between them and forwards packets.
  (Why it exists at all is explained in [Part 4](#why-is-there-a-relay-at-all).)

Everything else in this guide is about the two environments where we
exercise that path: **simulation** (on a PC, no cars, no risk) and **the
real world** (Comma 3X devices, eventually a real car).

The hardware: a [Comma 3X](https://comma.ai/shop/comma-3x) is a
phone-sized device that mounts on the windshield and runs
[openpilot](https://github.com/commaai/openpilot), an open-source
driver-assistance system that can control a car's gas, brake, and
steering. Our work is a modified copy (a "fork") of openpilot with hCCC
and V2V added.

---

## Part 2 — The testing ladder, and why each rung exists

The guiding principle of this project's structure: **find every bug in the
cheapest, safest place it can possibly be found.** A bug discovered in
simulation costs minutes. The same bug discovered while a real car is
accelerating costs a lot more. So testing climbs a ladder, and nothing
moves up a rung until the rung below passes:

| Rung | What runs | What it can catch | Risk |
|---|---|---|---|
| 1. Simulation, sensor lead | one openpilot on a PC, simulated world | controller logic, tuning | none |
| 2. Simulation, V2V lead | two openpilots + relay on one PC | the entire V2V code path | none |
| 3. Simulation, cloud relay | same, relay on a rented server | network transport over the internet | none |
| 4. Bench test | two real devices on a desk | device-specific surprises | none |
| 5. **Phase-1 in-car** *(next)* | ego drives a real car; lead device replays a recording from inside the same car | the full control loop with real vehicle dynamics | controlled |
| 6. Phase-2 two cars | real lead car transmitting real measurements | everything | controlled |

Two design decisions make this ladder work, and they're worth understanding
because they shaped most of the code:

**Decision: the same three-hop path at every rung.** The lead publisher,
relay, and ego subscriber are the *same code* whether everything runs on
one laptop or across two cars. Only addresses change (`127.0.0.1` →
`10.42.0.1` → a cloud server). **Why:** if each rung used different
plumbing, passing rung 2 would prove nothing about rung 5. Because the
plumbing is identical, every rung climbed is real, transferable evidence.

**Decision: a "virtual lead" for the first car test.** Rather than
starting with two cars, phase 1 puts both devices in *one* car: the ego
device drives, and the lead device — sitting in the cupholder — *replays a
pre-recorded speed profile* as if it were a real car ahead. **Why:** the
ego cannot tell the difference (same packets, same path), so this tests
100% of our code with one car, one driver, and a completely predictable
"lead" that does exactly the same thing every run. The phase-2 step then
changes only one thing: where the lead's numbers come from.

A third decision is about what the lead replays. A **scenario** is a
pre-recorded speed-over-time profile — one column of
[tools/sim/lib/Scenarios.csv](tools/sim/lib/Scenarios.csv). Our standard
test, scenario 48, accelerates to ~32 mph and then varies speed in waves.
**Why scenarios:** repeatability. Every simulation run, bench run, and car
run follows the *same* profile, so results are directly comparable across
rungs and across weeks. ("Did Tuesday's change make following smoother?"
is only answerable if Tuesday and Wednesday drove the same profile.)

### The dead end we kept off the ladder

We attempted a rung between 3 and 4 called **HIL** (hardware-in-the-loop):
the real device drives the *simulated* world over a USB cable to the PC.
It is permanently shelved, and the reason is physics, not software: the
Comma 3X's USB-C port is wired to its internal safety microcontroller (the
"panda"), not to its main computer, so the high-bandwidth PC↔device link
HIL needs cannot exist on this hardware. The investigation and conclusion
are documented in [tools/sim/hil/README.md](tools/sim/hil/README.md).
**Why we're noting a failure here:** the bench test (rung 4) was built as
its replacement, and turned out to catch everything we'd hoped HIL would.

---

## Part 3 — Tour of the code

Two directories matter, one per environment. Each table row is one file:
what it is, and why it exists.

### Simulation — [`tools/sim/`](tools/sim/README.md)

The simulator is [MetaDrive](https://github.com/metadriverse/metadrive),
an open-source driving world. The crucial trick: **openpilot does not know
it is in a simulation.** A "bridge" program feeds it fake camera images and
fake car signals in exactly the format a real car produces, and applies
openpilot's gas/brake decisions to the simulated car. Why this matters:
the code being tested is the *production* code, not a special test version
— so simulation results carry over to the car.

| File | What it is | Why it exists |
|---|---|---|
| [run_bridge.py](tools/sim/run_bridge.py) | The bridge: translates simulator ↔ openpilot every tick | Lets unmodified openpilot drive a simulated car; also drives the lead car along a scenario |
| [lib/Scenarios.csv](tools/sim/lib/Scenarios.csv) | Library of speed profiles (one per column) | Repeatable, comparable runs (see Part 2) |
| [launch_openpilot_ego.sh](tools/sim/launch_openpilot_ego.sh) | Starts openpilot configured for simulation | Sets ~6 environment flags nobody should have to remember |
| `launch_openpilot_lead.sh` (on the `hcc-lead` branch) | Same, for the lead instance in two-instance runs | The lead runs as a separate openpilot so its V2V *publishing* code is also production code; lead-only code lives on its own branch because a real lead device runs that branch |
| [README.md](tools/sim/README.md) | Full instructions, written for newcomers | Onboarding without a human in the loop |

The simulation has two main modes (full commands in the README): **Mode 1**
follows the lead via the simulated camera/radar (no V2V — isolates the
controller), and **Mode 2** runs two openpilot instances with the lead
publishing over a local relay (the full V2V path — rung 2 of the ladder).
**Why two modes:** when a Mode 2 run fails, re-running in Mode 1 instantly
answers "is the problem the controller or the communication?"

### V2V components — [`tools/hcc_v2v/`](tools/hcc_v2v/README.md)

These are the pieces shared by simulation and the real world — the
"identical plumbing" of Part 2.

| File | What it is | Why it exists |
|---|---|---|
| [relay_server.py](tools/hcc_v2v/relay_server.py) | The relay: accepts one lead + one ego, forwards lead packets to the ego, logs every packet to CSV | See [the relay question](#why-is-there-a-relay-at-all) below; the CSV log makes every run auditable after the fact |
| [virtual_lead.py](tools/hcc_v2v/virtual_lead.py) | Replays a scenario column as real V2V packets, 50 per second | The phase-1 "fake lead car" (see Part 2); also useful on a desk |
| [bench_ego.py](tools/hcc_v2v/bench_ego.py) | A stand-in receiver that registers as the ego and counts packets | On a desk there's no car, so openpilot's real receiver isn't running; this fills the seat so the full path can still be exercised |
| [plot_v2v_run.py](tools/hcc_v2v/plot_v2v_run.py) | Turns a relay CSV into statistics (packet loss, rate, delay jitter) and a graph | "Did the network behave?" should be a printed number, not an opinion |
| [scripts/setup_v2v_network.sh](tools/hcc_v2v/scripts/setup_v2v_network.sh) | One-time device setup: ego hosts a WiFi hotspot + runs the relay as a service; lead joins the hotspot; both get configuration | Device setup by hand is ~15 error-prone steps; this is one command per device, safe to re-run |
| [scripts/hcc_monitor.py](tools/hcc_v2v/scripts/hcc_monitor.py) | Live status line on a device (engaged? speed? what is hCCC commanding?) + optional CSV recording | During a car test, someone must be able to see what the controller is doing *right now*; the CSV becomes the run's response record |
| [selfdrive/controls/lib/vendor/hcc_v2v_core.py](selfdrive/controls/lib/vendor/hcc_v2v_core.py) | The packet format and publisher/subscriber logic | Single source of truth for "what's in a packet" — used by every component above |
| [selfdrive/controls/lib/longcontrol.py](selfdrive/controls/lib/longcontrol.py) | Where the received V2V signal meets the acceleration controller | The integration point: this is the code that makes the car move |

#### Why is there a relay at all?

A reasonable question — why doesn't the lead just send packets straight to
the ego? Three reasons. (1) **One place to look:** every packet, forwarded
or rejected, passes one point that logs it to a CSV — after any run, good
or bad, we have a complete record of what was sent, what arrived, and why
anything was dropped. (2) **One place to change:** lead and ego only ever
need the relay's address; moving from laptop to car to cloud server
changes one setting, not the software at either end. (3) **The future
transport:** the eventual deployment uses cellular networking, where two
cars cannot address each other directly but both can reach a server — the
relay *is* that server, already in the architecture from day one.

#### Why UDP, the "unreliable" protocol?

Networks offer two flavors: TCP (every byte guaranteed to arrive, retried
until it does) and UDP (each packet sent once; some may be lost). We chose
UDP, deliberately. For control data, **late is worse than lost**: a speed
reading from 2 seconds ago is not just useless but dangerous, and TCP's
retries would deliver exactly that — old data, late, while newer data
queues behind it. With UDP, a lost packet simply means the next one (20 ms
later) is used instead. The safety net: if no packet arrives for 100 ms,
the ego marks the signal **stale and stops commanding acceleration** — the
system's reaction to silence is always "do less," never "guess."
(Measured loss on the bench: 0 of 1750 packets.)

### Real-world testing — [`tools/real_world_testing/`](tools/real_world_testing/README.md)

This directory is the laptop's side of a car test: starting runs,
watching them, and getting every byte of evidence back.

| File | What it is | Why it exists |
|---|---|---|
| [field_test.py](tools/real_world_testing/field_test.py) | The orchestrator: `check` (preflight), `run` (start everything, watch, collect), `abort` (emergency stop), `collect` (recover data), `find-lead` (find the lead's network address) | See the design notes below — nearly every feature exists because of a specific failure we hit |
| [field_test_ui.py](tools/real_world_testing/field_test_ui.py) | Point-and-click window over the same commands | In a car, with a driver waiting, typing long commands is where mistakes happen |
| [README.md](tools/real_world_testing/README.md) | Procedures: one-time setup, bench rehearsal, the in-car test-day script, safety checklist | The test-day script assumes no internet access (see below) and a non-developer operator |

**Design notes on `field_test.py`** — each of these is a decision with a
story behind it:

- **A preflight check (14 items) refuses to start a run until everything
  passes.** It verifies both devices are reachable, the relay is running,
  every configuration flag is right, the scenario file exists, the log
  directory is writable, and — most subtly — that the two devices' clocks
  agree. **Why:** every one of those checks corresponds to a failure that
  actually happened during bring-up, and most of them fail *silently*
  (packets just don't arrive, with no error anywhere). The clock check is
  the flagship example: the receiver rejects packets whose timestamp is
  more than half a second off, and the lead device's clock drifts because
  it has no internet to sync from — a run would simply record nothing,
  with every light green. The preflight turns each silent failure into a
  loud, labeled one, with the fix printed next to it.
- **Every run lands in one timestamped folder** containing what the lead
  sent, what the relay saw, how the ego responded, all console output,
  computed statistics, and a graph — plus a metadata file recording the
  exact configuration and free-text notes ("parking lot, dry, second
  ramp"). Nothing is ever overwritten. **Why:** a field test costs a
  road, a car, a driver, and an afternoon; the data must survive anything,
  including questions nobody thought to ask until weeks later.
- **The remote programs survive a lost laptop connection.** Test runs are
  started on the devices in a disconnect-proof way; if the laptop's WiFi
  hiccups mid-run, the run continues and the data is recoverable with one
  command (`collect`). **Why:** laptop WiFi in a moving car *will*
  hiccup, and a dropped connection must never destroy a run.
- **`abort` exists, and is honest about what it is.** It kills the data
  feed; within 100 ms the ego stops commanding acceleration (the staleness
  rule above, used on purpose). It is *not* the emergency stop — the
  driver's brake is, which disengages everything instantly by openpilot's
  normal mechanism. The tooling never engages or disengages the car; only
  the driver does. **Why:** layered safety with a human at the top, and
  documentation that never overstates what software guarantees.

#### One physical constraint that shaped everything: the WiFi island

The devices talk over a WiFi hotspot the ego broadcasts. **Why a hotspot:**
the test must work on a closed road in the middle of nowhere — no lab
WiFi, no cell coverage assumed, no infrastructure of any kind. The price:
each device has one WiFi radio, so a device on the hotspot *cannot
simultaneously be on the internet* — and neither can the laptop while it's
connected. Test days are therefore "offline islands": everything is
prepared, updated, and downloaded in advance, and the test-day procedure
in the README is written to be followed without internet access. This
constraint also explains the clock-drift problem above (no internet = no
time sync) and several of the preflight checks.

---

## Part 4 — The decision log

Every significant choice, its alternative, and why we chose as we did.
(The deepest technical record is the git history; this is the readable
version.)

| Decision | Alternative | Why we chose this |
|---|---|---|
| Build on openpilot (fork) | Write a controller from scratch | Production-grade car interfaces, safety model, and tooling for free; our work stays focused on the cooperative controller itself |
| Same 3-hop V2V path at every test rung | Simpler ad-hoc plumbing per environment | Evidence transfers up the ladder (Part 2) |
| Scenario CSV replay | Drive freely each test | Repeatability — comparisons across weeks and across rungs |
| Relay in the middle | Direct lead→ego packets | Auditability, one configuration point, matches future cellular deployment ([details](#why-is-there-a-relay-at-all)) |
| UDP + 100 ms staleness cutoff | TCP "reliable" delivery | Late data is worse than lost data; silence always degrades to "do less" ([details](#why-udp-the-unreliable-protocol)) |
| WiFi hotspot between devices | Lab WiFi / cellular | Works anywhere with zero infrastructure; cellular is a later rung |
| Abandon HIL (USB device↔PC) | Keep debugging it | Hardware fact: the device's USB port doesn't reach its main computer ([details](tools/sim/hil/README.md)) |
| Bench test before any driving | Go straight to the car | Caught 3 real bugs at a desk, incl. one that would have crashed the controller *while driving* (Part 5) |
| Virtual lead for phase 1 | Start with two cars | Full code path, half the cars, perfectly repeatable "lead" (Part 2) |
| Preflight gate before every run | Trust the setup | Every check encodes a real, usually *silent*, past failure |
| One immutable folder per run | A shared results directory | Field data must survive anything, incl. questions asked weeks later |
| Two-person car protocol, driver's brake supreme | Allow solo testing | Layered safety; software never the last line |

---

## Part 5 — What the bench test caught (why the ladder works)

In one day of desk testing (2026-06-12, no car involved), the pipeline
went from "fully set up" to "fully working" in 5 attempts, and the
failures it surfaced are the best argument for the methodology:

1. **A crash that would have happened while driving.** A configuration
   value was stored as a number where the code expected text; the code
   that reads it crashed. That code also runs inside the car's control
   process — discovered on the road, it would have shut down the
   controller mid-drive. Cost at the desk: one afternoon. *(Fixed in
   [selfdrive/controls/lib/hcc_v2v.py](selfdrive/controls/lib/hcc_v2v.py).)*
2. **Two "failures" that weren't.** The launch step appeared to time out;
   two takes were spent on it. The leftover logs later proved the test had
   *run perfectly both times* — only the laptop's way of waiting for
   confirmation was at fault. Lesson encoded into the tooling: launches
   now confirm success by reading the started program's ID directly
   rather than trusting the connection to close.
3. **A silent permission problem.** A system service had created the log
   directory as the administrator user, so the test programs (a normal
   user) couldn't write their logs — and the failure mode was *programs
   dying instantly while reporting a healthy-looking startup*. Lesson
   encoded: launches now verify the program is still alive a second later,
   and the preflight checks directory permissions on both devices.

Final bench result: **1750 of 1750 packets delivered at 50 per second,
zero loss, worst delay gap 51 ms** (limit: 100 ms). The full record, like
every run, is a folder: `tools/real_world_testing/runs/run_20260612_190830_scn48/`.

---

## Part 6 — Where the project stands

- ✅ Rungs 1–4 of the ladder pass (simulation modes, cloud relay, bench).
- ⏭️ **Next: phase-1 in-car test** — the step-by-step script, packing
  list, and safety checklist are in
  [tools/real_world_testing/README.md](tools/real_world_testing/README.md).
- 🔭 Then: phase-2 (two cars, real lead measurements — the lead-side
  publisher already exists on the `hcc-lead` branch and is exercised in
  simulation Mode 2), and eventually the cellular relay rung.

### If you only read three documents

1. This one.
2. [tools/real_world_testing/README.md](tools/real_world_testing/README.md)
   — how a car test actually runs, including safety.
3. [tools/sim/README.md](tools/sim/README.md) — how the simulations run.

### Glossary

| Term | Meaning |
|---|---|
| ego / lead | the following car / the car being followed |
| hCCC, HC3 | our cooperative cruise controller / this project |
| V2V | vehicle-to-vehicle data transmission |
| openpilot | the open-source driver-assistance system we build on |
| Comma 3X | the windshield-mounted device that runs openpilot |
| panda | the device's internal safety microcontroller |
| relay | the small program that forwards lead packets to the ego |
| scenario | a pre-recorded speed profile used for repeatable tests |
| bridge | the program connecting the simulator to openpilot |
| MetaDrive | the driving simulator |
| bench test | full pipeline rehearsal with real devices, no car |
| preflight | automated checks that gate every test run |
| staleness | the 100 ms no-data cutoff after which the ego stops commanding acceleration |
| UDP | the send-and-forget network protocol we use (see Part 3) |
| AGNOS | the Comma 3X's operating system |
