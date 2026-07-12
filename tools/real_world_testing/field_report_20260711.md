# HC3 field report — 2026-07-11 (Tucson, phase-1 COMPLETE)

**Vehicle:** 2024 Hyundai Tucson (rental, radar-SCC, fingerprints as
`HYUNDAI_TUCSON_4TH_GEN` after a one-string DB fix).
**Setup:** ego on the harness driving the car; virtual lead replaying
scenarios (both from the lead device over WiFi and locally on the ego over
localhost during debugging); laptop orchestrating on the `hcc-v2v` hotspot.

## Headline

**Phase 1 is complete.** The ego drove the full scenario-48 profile under
HC3 control: engaged via SET, feet off, tracking the transmitted V2V speed
profile through acceleration, a mid-profile slowdown, re-acceleration, and
the final braking — peaking at **14.66 m/s (32.8 mph)** with commanded vs.
achieved acceleration matching nearly 1:1 (`cmd +1.33 → aEgo +1.31`,
`cmd −1.33 → aEgo −1.27`). Scenarios 49 and 50 were also run and starred.
The canonical good-run dataset is in
[`runs/scn48_THIS_WORKED/`](runs/scn48_THIS_WORKED/).

Getting there consumed the day finding and fixing four real bugs and
diagnosing two more. None of them were in the controller: the cascade hCCC
itself behaved correctly whenever its input signal was valid.

## Bugs found today

### 1. Fingerprint database gap (fixed on device; upstream-worthy)
The 2024 Tucson's camera firmware (`99211-CW020 14Z`) was catalogued under
the Santa Cruz in our pinned opendbc, while its radar FW was under the
Tucson — no single candidate matched both ECUs, so openpilot booted MOCK/
dashcam. Fix: appended the camera string to the Tucson entry in the
**device's** `opendbc_repo/.../fingerprints.py` (backup `.tucson_bak`).
Current upstream master also lacks this string — worth a PR to comma.

### 2. Engagement / SET button (resolved in the field; root cause to verify)
Pressing SET initially did nothing: `mainCruise` button events decoded but
SET/RES never appeared, on either the standard or ALT buttons message.
Diagnostic watchers early in the day were too short-lived to be conclusive
(lesson recorded below). Engagement later worked. **Before the next run:
verify which button configuration is active on the ego** (`values.py` vs
`values.py.btnbak` in the device's opendbc) and re-run the button watcher
to document the working configuration.

### 3. Staleness-reset thrash (mitigated on device; permanent fix pending)
The V2V subscriber thread inside controlsd gets starved ~100 ms about once
per second (OS scheduling; the packet stream itself was proven clean at
every hop). The staleness threshold was exactly 100 ms, and each trip
called the controller's `reset()` — wiping the PID integral and
feedforward once per second. The controller could never build authority:
this was the "car crawls at 3–5 mph" symptom, and it also explains the
sluggishness of the first successful 15:05 run on 07-11's morning session.
Mitigation: `export HCC_V2V_STALE_THRESHOLD_MS=300` appended to the ego's
`launch_env.sh` (**device-only edit — a `git checkout -- .` erases it**).
Permanent fix pending: raise the committed default and separate "output
zero on brief dropout" from "reset controller state" (reset only after
sustained loss).

### 4. Seq replay-guard killed every second run (FIXED in git, `5a7d9a543`)
`V2VLeadBuffer` dropped any packet with `seq <= last_seq` and never reset —
but every publisher launch renumbers from 0. After a session ending at seq
N, the next run's first N packets (≈70 s at 50 Hz) were silently discarded:
the first run after every controlsd restart worked, every back-to-back run
was deaf. Misdiagnosed twice as clock skew before the swaglog mode history
exposed it. Fixed: a seq regression larger than 50 packets is treated as a
new publisher session and adopted; small regressions are still dropped as
replays. Tests added (10/10).

### 5. SCC standstill latch (diagnosed; code fix pending)
If the car reaches a complete stop while engaged, the Hyundai drivetrain
latches its standstill hold and **ignores positive acceleration requests**
(`cmd/out +2.00` transmitted for 30 s, `aEgo ≈ 0`) while still executing
negative ones. Stock openpilot releases the hold via its
`stopping`/`starting` longitudinal states (`stopReq` handshake); our
V2V-only path forces `long_state = pid` unconditionally and never performs
the handshake. **Driving workaround:** engage while rolling; if the car
fully stops while engaged, CANCEL, get rolling, re-engage. **Code fix
pending:** derive `should_stop` from the V2V target and run the standard
state machine instead of forcing `pid`.

### 6. Gas press disengages (diagnosed in data; fix pending)
Run `171310_scn48_gasblendattempt`: gas rising edge at 19.65 s → disengage
at 20.15 s, brake untouched. Ruled out: panda firmware (gas-disengage no
longer exists in this safety version), fork software paths (removed).
Prime suspect: the car's TCS reports `ACCEnable != 0` during a driver gas
override → openpilot reads `accFaulted` → IMMEDIATE_DISABLE. To confirm
with the accFaulted watcher, then suppress that event while `gasPressed`
(one-line filter in `selfdrived.py` next to the existing FCW/AEB filter).
Until fixed: gas-blending experiments will disengage the system.

## Tooling gained today

- **Injection probe** (now folded into `hcc_monitor.py`, commit
  `617b16fcc`): every run records the full causal chain per sample —
  `hccc_accel` (controller) → `cmd_accel` (decision) → `out_accel` (wire)
  → `a_ego_mps2` (physics) — plus `gas_pressed`/`brake_pressed`
  (driver-intervention markers) and `standstill`. Use `--monitor_hz 20`
  for research runs.
- Swaglog mode-transition history
  (`grep hcc_v2v_mode_transition /data/log/swaglog*`) proved decisive
  twice; it is the subscriber's ground truth.
- Diagnostic lesson, twice paid: **SubMaster.update(t) returns on every
  message, not after t ms** — early watchers ran for ~1–2 s instead of 30
  and produced misleading conclusions (the button mystery, the "SET is
  unparseable" verdict). All later watchers are wall-clock timed.

## Ego device state ledger (as of end of day)

| Item | State | In git? |
|---|---|---|
| `hcc_v2v_core.py` (seq fix) | scp'd, matches `5a7d9a543` | yes — reconciles on next pull |
| `hcc_monitor.py` (research columns) | scp'd, matches `617b16fcc` | yes — reconciles |
| `fingerprints.py` camera FW | device-only edit (+ `.tucson_bak`) | **no — re-apply after any opendbc reset** |
| `values.py` button flag | **uncertain** (`.btnbak` exists) — verify | no |
| `launch_env.sh` stale-threshold export | device-only | **no — wiped by `git checkout -- .`** |
| Lead→ego ssh key | not working (sshd authorized-keys path) | — |

## Before the next run (in order)

1. Land the permanent staleness fix (committed default + no-reset-on-blip).
2. Implement the standstill `stopping`/`starting` handshake for V2V-only.
3. Confirm and fix the gas-press `accFaulted` disengage.
4. Verify the ego's button configuration and document it.
5. Fix the lead→ego ssh key (find sshd's `AuthorizedKeysFile`) so clock
   sync is one reliable local hop instead of the laptop-hop guesswork.
6. Reconcile the ego with git (`git checkout -- . && git pull`), then
   **re-apply the two device-only edits** (fingerprints camera FW,
   launch_env threshold) — or land both in git first so nothing needs
   re-applying.
7. Upstream candidates for comma: Tucson camera FW string; possibly the
   seq-session behavior.

## Data inventory (all committed under `runs/`)

- `scn48_THIS_WORKED/` — **the phase-1 result**: full-profile probe CSV +
  transmitted profile.
- `run_20260711_*_star/` — operator-starred good runs, incl. scenario 49
  (`172015`, `172644`) and scenario 50 (`172418`).
- `run_20260711_171310_scn48_gasblendattempt/` — the gas-disengage dataset
  (bug 6).
- `inject_probe/` — all probe CSVs from the debugging arc, including the
  before/after pairs for bugs 3 and 4 (`194157` = thrash, `195318` =
  post-threshold-fix ramp, `202132` = THIS_WORKED source, `202753` =
  standstill-latch dataset).
- `local_scn48/` — relay-level forensics for the seq bug.
- Morning session (`1426xx`–`1513xx`): the staleness/skew-era runs.
