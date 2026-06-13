# HC3 / hCCC openpilot — Work Log & Timeline

*Reconstructed from the git history (`git log --author=ethanmathias@gmail.com`).*

## Summary

- **288 commits** authored by Ethan Mathias across all branches (242 on the main `hcc-ego` branch; the remainder are lead-device commits on `hcc-lead`)
- **Span:** 2026-01-25 → 2026-06-12 (~4.5 months)
- **Cadence (all branches):** Jan 6 · Feb 121 · Mar 101 · Apr 25 · May 11 · Jun 24
- **Outcome:** test rungs 1–4 passed (both sim modes, cloud relay, and the on-device bench test at 0% packet loss); phase-1 in-car test is next.

## Phase totals

| Phase | Milestones | Commits |
|---|---|---|
| A · Research & Setup | 1 | 6 |
| B · Sim & hCCC Controller | 4 | 79 |
| C · Manual Control | 3 | 39 |
| D · Smoothing & Scenarios | 6 | 100 |
| E · V2V Transport | 7 | 29 |
| F · HIL & Device Work | 4 | 13 |
| G · Field Testing & Bench | 5 | 22 |

## A · Research & Setup

### 1. Research, openpilot architecture study & integration plan
*2026-01-25 → 2026-01-27 · 6 commits*

Surveyed the openpilot codebase, captured a written architecture/integration plan for adding the hCCC cooperative controller, and imported the BeamNG reference material the controller was being ported from.

Representative commits:
- `1ce0470 — added beamng code`
- `60ec017 — integration plan`
- `9d6f606 — openpilot arch`

## B · Sim & hCCC Controller

### 2. MetaDrive simulator installation & bring-up
*2026-02-05 · 2 commits*

Stood up the MetaDrive driving simulator as the rung-1 test environment so unmodified openpilot could be driven in a synthetic world.

Representative commits:
- `124821b — working metadrive installation`

### 3. hCCC controller + openpilot integration, first traffic scenarios
*2026-02-10 · 24 commits*

Wrote the hCCC controller and wired it into openpilot's longitudinal path; built the first deterministic MetaDrive traffic/lead scenarios and iterated on lead spawning.

Representative commits:
- `9cc74e7 — hcc_controller code (openpilot integration)`
- `026595f — simple traffic scenario`
- `0c60535 — hccintegration test1`

### 4. MetaDrive bridge & hCCC acceleration override
*2026-02-11 · 27 commits*

Made the sim↔openpilot bridge feed synthetic camera/vehicle signals and apply actuation back; implemented hCCC's hard acceleration override when a lead exists, with delayed-start and accel-clip smoothing.

Representative commits:
- `991043624 — HCC hard-overrides accel when a lead exists; deterministic lead scenario`
- `c05a445 — accel clip`
- `f2fa60c — updates smoothing`

### 5. Lead-vehicle IDM policy & live-lead integration
*2026-02-12 · 26 commits*

Gave the simulated lead an IDM (intelligent-driver-model) policy and a live-lead feed; added auto-enable of hCCC, throttle bypass, and override-stop handling for the acceleration path.

Representative commits:
- `5abf5f1 — added idm policy to lead car`
- `69791e3 — livelead`
- `a8be85c — autoenablehcc`
- `fe51e58 — changes for hcc acceleration`

## C · Manual Control

### 6. Auto-launcher & multi-terminal sim workflow
*2026-02-23 · 9 commits*

Built sim_terminals.sh and an auto-launcher so a full sim session (world + openpilot + lead) starts from one command; added steering damping and quality fixes.

Representative commits:
- `0e26984 — sim_terminals.sh`
- `d2a97e9 — updates for auto launcher script`

### 7. Human override + manual steering/throttle input
*2026-02-24 · 14 commits*

Added human-override and manual steering/acceleration input to the sim, blended steering, alternate car models, and an out-of-road recovery patch.

Representative commits:
- `85010fd — add human override and manual input for steering and acceleration`
- `ad6c39b — blend steering in sim`

### 8. Logitech wheel & pedal control, lane-centering lead
*2026-02-26 · 16 commits*

Integrated a Logitech racing wheel and pedals for manual driving, added pedal-update logic, and made the lead vehicle center itself in the lane.

Representative commits:
- `2b667ff — add logic for logitech wheel`
- `98b430e — pedal update logic`
- `241c2c1 — lead vehicle to center the lane`

## D · Smoothing & Scenarios

### 9. Acceleration smoothing to match BeamNG blending profile
*2026-02-27 → 2026-03-17 · 14 commits*

Replaced the old acceleration-blending technique so the sim's acceleration profile matches the BeamNG reference; tuned accel ratios for weak pedals and added analysis graphs.

Representative commits:
- `913b4c6 — startup works beautifully in the smoothing push`
- `5e0b45d — remove old blending technique — now matches BEAMNG blending profile`
- `37134a5 — accel ratio`

### 10. Scenario system (-scn) & IDM removal
*2026-03-01 → 2026-03-10 · 10 commits*

Added a -scn scenario-selection argument and the CSV-driven scenario system, removed the IDM policy in favor of replayed profiles, and switched the CSV logic to m/s with two-lane support.

Representative commits:
- `362a06e — added scenarios -scn`
- `cfabaa1 — remove IDM policy`
- `28ad1d6 — switched logic of csv to m/s from m`

### 11. Output CSV logging, sampling rate & road-length tuning
*2026-03-12 → 2026-03-13 · 12 commits*

Added output-CSV logging of every run, fixed the 10 Hz sampling so runs are comparable, tuned road length, and introduced the HCC_CHANGE_NOTE tag marking every edit to upstream openpilot code.

Representative commits:
- `c04931b — output csv added, scenario end when scn ends`
- `7cc1661 — add HCC_CHANGE_NOTE tag for updates to org code`
- `b9e1203 — reverse the 10hz logic, samples outputcsv as much as possible`

### 12. Trajectory & steering-smoothing experiments
*2026-03-19 · 12 commits*

Iterated on lane/road length and smoother acceleration-from-lead behavior, with several try/revert cycles converging on a stable time-step.

Representative commits:
- `5e6130f — smoother acceleration from lead?`
- `7ac2ed4 — working again time step`

### 13. Simulation rewrite for smoothness & blending fixes
*2026-03-23 → 2026-03-25 · 29 commits*

Rewrote the sim/bridge path for smoothness, fixed the blending, and removed model-based trajectory projection in favor of wheel-pointing direction; added output graphs.

Representative commits:
- `d99781a — codex sim code`
- `31c0d22 — fixed blending`
- `f4d0e5f — remove model based trajectory projection with where wheels are pointing`

### 14. Radar handling & BeamNG scenario data
*2026-03-26 → 2026-03-27 · 23 commits*

Added radar-blocking so the controller follows the V2V/scenario lead rather than sensed radar, imported BeamNG CSV scenario data, and validated against smooth-graph reference tests (through test 21).

Representative commits:
- `a0fd655 — block radar`
- `6ab0c8b — added beamng`
- `f4446bc — smooth graph test 21 (return here if need be)`

## E · V2V Transport

### 15. V2V transport scaffolding
*2026-03-27 · 1 commits*

Laid down the first scaffolding for the vehicle-to-vehicle (V2V) data path that would later carry the lead's state to the ego.

Representative commits:
- `67c054f — first commit of scaffolding`

### 16. Shared V2V transport base + lead/ego paths
*2026-03-31 · 3 commits*

Built the shared HCC V2V transport base (packet format, publisher/subscriber) and the matching lead-side publisher and ego-side control paths — the 'identical plumbing' reused at every test rung.

Representative commits:
- `3ef225a — Add shared HCC V2V transport base`
- `1861cf7 — Add lead-side V2V publisher branch`
- `802be63 — Add ego-side V2V HC3 control path`

### 17. V2V sim launchers (ego/lead) & manager preload fixes
*2026-04-06 → 2026-04-07 · 11 commits*

Added ego/lead V2V sim launchers, made openpilot's manager skip blocked process preloads in the two-instance configuration, and stabilized the HC3 V2V sim replay and lead startup.

Representative commits:
- `d583e32 — Add ego V2V sim launcher`
- `cb864d7 — Skip blocked process preloads in manager`
- `ed78413 — Stabilize HC3 V2V sim replay`

### 18. HC3 OpenPilot integration progress report (LaTeX)
*2026-04-10 · 1 commits*

Wrote a formal LaTeX progress report documenting the HC3/openpilot integration to date.

Representative commits:
- `0ff6916 — docs: add HC3 OpenPilot integration progress report (LaTeX)`

### 19. V2V-only HC3 transport + Lightsail cloud relay
*2026-04-15 · 3 commits*

Completed the V2V-only HC3 transport on both lead and ego sides and documented the AWS Lightsail cloud-relay workflow — proving the relay path over a real wide-area network (rung 3).

Representative commits:
- `f7298a3 — Add lead-side V2V transport integration`
- `e4cad05 — Add V2V-only HC3 transport integration`
- `815042c — Document Lightsail relay sim workflow`

### 20. V2V UI toggles & param defaults
*2026-04-23 · 2 commits*

Added on-device UI toggles for V2V, fixed a param default, and updated the install README to use direct file writes for params.

Representative commits:
- `0573568 — Add V2V UI toggles, fix param default, update install README`

### 21. Git LFS fixes & install / relay-setup documentation
*2026-04-25 · 8 commits*

Redirected Git LFS fetches from GitLab to GitHub, fixed the LFS smudge/install steps, brought the hcc-lead install guide to parity with hcc-ego, and documented the HCC V2V relay setup.

Representative commits:
- `7ab907a — lfsconfig: redirect LFS fetch from GitLab to GitHub`
- `c1bfb8e — README: document HCC V2V relay setup`

## F · HIL & Device Work

### 22. HIL two-device MetaDrive bridge
*2026-05-04 · 2 commits*

Built a hardware-in-the-loop MetaDrive bridge to drive the simulated world from two real Comma 3X devices (ego + lead) — later shelved for a hardware reason (the device's USB-C port is wired to the panda, not the main computer), documented in tools/sim/hil/README.md.

Representative commits:
- `82748d7 — add hardware-in-the-loop (HIL) MetaDrive bridge for two Comma 3X devices`
- `1945178 — add device-side HIL support for lead Comma 3X`

### 23. Sim V2V param fixes & three operational modes
*2026-05-07 → 2026-05-08 · 8 commits*

Fixed the HCCV2VRelayPort param typing (INT vs str) and the OPENPILOT_PREFIX mismatch in hc3 mode, defaulted V2V off for single-device sim, and documented the three operational modes; removed stale LFS test files.

Representative commits:
- `757c28d — sim: write hCCC V2V params to Params store at launch`
- `eb032a4 — docs: add three operational modes to sim README`

### 24. SDE cleanup refactor across hCCC, sim & HIL
*2026-05-28 · 1 commits*

Software-engineering cleanup pass across the hCCC controller, the MetaDrive sim, and the HIL code.

Representative commits:
- `5714490 — refactor: apply SDE cleanup across hCCC, MetaDrive sim, and HIL code`

### 25. Block AGNOS auto-downgrade daemon on device
*2026-06-07 · 2 commits*

Blocked the openpilot updater daemon on the devices so AGNOS (the Comma 3X OS) would not auto-downgrade and undo the test setup.

Representative commits:
- `bbefeb1 — hil: block updated daemon to prevent AGNOS downgrade on device`

## G · Field Testing & Bench

### 26. Two-device bridge fixes, preflight & manual-input cleanup
*2026-06-11 → 2026-06-12 · 4 commits*

Fixed the two-device bridge port collision, added a preflight script and device monitor, made the sim lead follow a CSV profile by default, and removed the gas/brake-pressed floor in manual longitudinal input.

Representative commits:
- `84873be — hil: fix two-device bridge port collision, add preflight script`
- `45139c4 — hil: lead vehicle follows CSV profile by default; add device monitor`
- `93d9f5f — controls: remove gasPressed/brakePressed floor in manual longitudinal input`

### 27. Real-world field-test orchestration + virtual lead
*2026-06-12 · 1 commits*

Built the real-world testing toolkit: the field_test orchestrator (check / run / abort / collect / find-lead), the point-and-click UI, the virtual_lead (replays a scenario as 50 Hz V2V packets), bench receiver, and run analysis/plotting — with a 14-item preflight gate and one immutable folder per run.

Representative commits:
- `5beb1d4 — tools: real-world HC3 testing — virtual lead, field test orchestration, docs`

### 28. V2V network setup-script hardening (AGNOS / systemd / params)
*2026-06-12 · 12 commits*

Hardened scripts/setup_v2v_network.sh and the relay unit against the Comma 3X's quirks: remount the read-only AGNOS rootfs for installs, normalize the systemd WorkingDirectory, run set_param as the right user with the repo venv python, write INT-typed params as int, set PYTHONPATH, switch networks last so an SSH drop can't kill setup, and resolve the device venv python for non-interactive SSH.

Representative commits:
- `045260e — remount AGNOS rootfs rw for the relay unit install`
- `5fef9d3 — switch networks last so WiFi SSH drop can't kill setup midway`
- `6a2da83 — hcc_v2v: tolerate int/bytes param values (HCCV2VRelayPort is INT-typed; crashed load_v2v_config)`

### 29. Field-launch robustness + BENCH TEST PASSED (1750/1750, 0% loss)
*2026-06-12 · 3 commits*

Made remote launches disconnect-proof (read the PID then close the ssh client; detect launches that die instantly; fix the root-owned log dir) and ran the full-pipeline bench test on the two real devices: 1750 of 1750 packets delivered at 50 Hz, 0% loss, worst inter-arrival gap 51 ms vs the 100 ms staleness limit. Rung 4 passed.

Representative commits:
- `14e3b2e — ssh_launch: read the PID then close the ssh client ourselves`
- `b57d0c5 — detect launches that die instantly; fix root-owned log dir`
- `efbbf4e — bench test passed — 1750/1750 packets, 0% loss, 50 Hz`

### 30. Docs: in-car test-day procedure + HCC_PROJECT_GUIDE
*2026-06-12 · 2 commits*

Wrote the offline in-car test-day procedure and safety protocol, and the HCC_PROJECT_GUIDE explaining the architecture, the testing ladder, and the decision log — setting up the next rung, the phase-1 in-car test.

Representative commits:
- `c2e099a — docs: in-car test-day procedure + HCC_PROJECT_GUIDE`
- `b88a7da — docs: HCC_PROJECT_GUIDE — assume domain expertise`

