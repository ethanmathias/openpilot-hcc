# HC3 / hCCC openpilot — Work Log & Timeline

*Reconstructed from the git history (`git log --all --author=ethanmathias@gmail.com`). Every figure below — commit counts, date spans, and lines changed — is computed from the live history by [build_work_log.py](tools/work_log/build_work_log.py).*

## Summary

- **288 commits** authored by Ethan Mathias across all branches (242 on the main `hcc-ego` branch; the remainder are lead-device commits on `hcc-lead`)
- **Span:** 2026-01-25 → 2026-06-12 (~4.5 months)
- **Cadence (all branches):** Jan 6 · Feb 121 · Mar 101 · Apr 25 · May 11 · Jun 24
- **Outcome:** test rungs 1–4 passed (both sim modes, cloud relay, and the on-device bench test at 0% packet loss); phase-1 in-car test is next.

## Phase totals

| Phase | Milestones | Commits | Lines (+/−) |
|---|---|---|---|
| A · Research & Setup | 1 | 6 | +3,141 / −119 |
| B · Sim & hCCC Controller | 4 | 79 | +3,376 / −2,360 |
| C · Manual Control | 3 | 39 | +1,066 / −640 |
| D · Smoothing & Scenarios | 6 | 100 | +86,384 / −76,200 |
| E · V2V Transport | 7 | 29 | +5,580 / −5,734 |
| F · HIL & Device Work | 4 | 13 | +3,324 / −507 |
| G · Field Testing & Bench | 5 | 22 | +5,286 / −1,242 |

## A · Research & Setup

### 1. Research, openpilot architecture study & integration plan
*2026-01-25 → 2026-01-27 · 6 commits · +3,141/−119 lines, 11 files*

Surveyed the openpilot codebase, captured a written architecture / integration plan for adding the hCCC cooperative controller, and imported the BeamNG reference material the controller was being ported from.

<details><summary>6 commits</summary>

- `1ce047079` 2026-01-25 — added beamng code
- `41442f543` 2026-01-25 — readme
- `477ea43ff` 2026-01-25 — readme
- `6b2c54525` 2026-01-25 — reaeme
- `60ec017c1` 2026-01-27 — integration plan
- `9d6f6064c` 2026-01-27 — openpilot arch

</details>

## B · Sim & hCCC Controller

### 2. MetaDrive simulator installation & bring-up
*2026-02-05 · 2 commits · +9/−3 lines, 3 files*

Stood up the MetaDrive driving simulator as the rung-1 test environment so unmodified openpilot could be driven in a synthetic world.

<details><summary>2 commits</summary>

- `124821bd6` 2026-02-05 — working metadrive installation
- `f04e315d0` 2026-02-05 — readme

</details>

### 3. hCCC controller + openpilot integration, first traffic scenarios
*2026-02-10 · 24 commits · +788/−523 lines, 14 files*

Wrote the hCCC controller and wired it into openpilot's longitudinal path; built the first deterministic MetaDrive traffic/lead scenarios and iterated on asset-safe lead spawning.

<details><summary>24 commits</summary>

- `026595f69` 2026-02-10 — simple traffic scenario
- `08f6cbdcb` 2026-02-10 — hccintegration
- `0928038db` 2026-02-10 — commit
- `0c6053572` 2026-02-10 — hccintegration test1
- `0cfa829c3` 2026-02-10 — updates
- `18b4604b8` 2026-02-10 — update
- `2920a4262` 2026-02-10 — org metadrive
- `32ed94d79` 2026-02-10 — test
- `4523b308f` 2026-02-10 — m
- `4d1d4f5f9` 2026-02-10 — scenario update
- `4fd267174` 2026-02-10 — update
- `53d01c324` 2026-02-10 — orig
- `5b1ad39bb` 2026-02-10 — original
- `5f4af6b2f` 2026-02-10 — text
- `728b414ba` 2026-02-10 — update
- `8c9f5d5f8` 2026-02-10 — upda
- `98e56c465` 2026-02-10 — comments
- `9cc74e718` 2026-02-10 — hcc_controller code (openpilot integration) still need to work on integration with current acceleration methods
- `beaefa560` 2026-02-10 — metadrive simulation files need to be tested
- `bf2c8d01c` 2026-02-10 — output
- `c43dfe8ca` 2026-02-10 — update
- `d70c10fd0` 2026-02-10 — update
- `e85da8dce` 2026-02-10 — changes
- `f106244ff` 2026-02-10 — 3 updated

</details>

### 4. MetaDrive bridge & hCCC acceleration override
*2026-02-11 · 27 commits · +1,777/−1,299 lines, 9 files*

Made the sim↔openpilot bridge feed synthetic camera/vehicle signals and apply actuation back; implemented hCCC's hard acceleration override when a lead exists, with delayed-start and accel-clip smoothing.

<details><summary>27 commits</summary>

- `0b775979e` 2026-02-11 — car glitch fix (just a dot now)
- `124830546` 2026-02-11 — patch autoupdate
- `19a850ad4` 2026-02-11 — updates
- `2b69359f9` 2026-02-11 — chat
- `3163ad746` 2026-02-11 — panda
- `3a9d7bfc1` 2026-02-11 — updates
- `478e54962` 2026-02-11 — sim
- `4fa2aeda6` 2026-02-11 — changes
- `55165a39c` 2026-02-11 — updates
- `5c8719020` 2026-02-11 — updates
- `6df7363a8` 2026-02-11 — error
- `7677ae60e` 2026-02-11 — updates
- `7b14487d6` 2026-02-11 — chat
- `8910bb777` 2026-02-11 — manual
- `94c0bd21a` 2026-02-11 — pre ferra going back
- `991043624` 2026-02-11 — Implemented the MetaDrive side to match your goal: HCC still hard-overrides accel when a lead exists, and now there is a deterministic lead scenario with asset-safe spawning.
- `aaf3aea3b` 2026-02-11 — minor
- `bca5a76b9` 2026-02-11 — delay start
- `bf4df439f` 2026-02-11 — chat
- `c05a44571` 2026-02-11 — accel clip
- `ce06641b3` 2026-02-11 — patch ferra
- `e3867a2d2` 2026-02-11 — drel
- `e66f0824d` 2026-02-11 — mesh
- `ede5ef425` 2026-02-11 — looping
- `ee4e672aa` 2026-02-11 — errors
- `f11b50870` 2026-02-11 — updates
- `f2fa60cb8` 2026-02-11 — updates smoothing

</details>

### 5. Lead-vehicle IDM policy & live-lead integration
*2026-02-12 · 26 commits · +802/−535 lines, 15 files*

Gave the simulated lead an IDM (intelligent-driver-model) policy and a live-lead feed; added auto-enable of hCCC, throttle bypass, and override-stop handling for the acceleration path.

<details><summary>26 commits</summary>

- `04c14f5f3` 2026-02-12 — bypassthrottle
- `0c187b7d8` 2026-02-12 — bugs
- `312daa581` 2026-02-12 — lead
- `46f6c517d` 2026-02-12 — edits to lead car speed
- `50337f4a2` 2026-02-12 — manuallead
- `522389844` 2026-02-12 — updates
- `5abf5f1a6` 2026-02-12 — added idm policy to lead car
- `692da74f2` 2026-02-12 — delay at start
- `69791e39d` 2026-02-12 — livelead
- `97cbf4e23` 2026-02-12 — override stop
- `97f54b01b` 2026-02-12 — bypass paramkey
- `9eba94700` 2026-02-12 — edits
- `9f67a8619` 2026-02-12 — chat
- `a8be85cfe` 2026-02-12 — autoenablehcc
- `a9d6d9c50` 2026-02-12 — remove pid accel
- `b1a3c7561` 2026-02-12 — reverted back to idm
- `b5d61c1a6` 2026-02-12 — updates
- `c9ace7555` 2026-02-12 — EVERYTHING worked before this commit
- `d06e401c5` 2026-02-12 — socket issues
- `e22a8ab7e` 2026-02-12 — straight lead car
- `e26c87a28` 2026-02-12 — switched idm policy to expert
- `e321fe875` 2026-02-12 — random car movements
- `eccc61019` 2026-02-12 — update to lead code
- `f77814020` 2026-02-12 — updates
- `fafa931dd` 2026-02-12 — car moves idm
- `fe51e5865` 2026-02-12 — changes for hcc acceleration

</details>

## C · Manual Control

### 6. Auto-launcher & multi-terminal sim workflow
*2026-02-23 · 9 commits · +150/−70 lines, 4 files*

Built sim_terminals.sh and an auto-launcher so a full sim session (world + openpilot + lead) starts from one command; added steering damping and quality fixes.

<details><summary>9 commits</summary>

- `0e26984bb` 2026-02-23 — sim_terminals.sh
- `7f8acd410` 2026-02-23 — chat
- `838728877` 2026-02-23 — quality changes
- `8d5740aad` 2026-02-23 — steering dampining
- `a4dc90dfe` 2026-02-23 — reverted changes
- `a5adea413` 2026-02-23 — revert changes fully
- `bacb03efd` 2026-02-23 — big changes
- `d2a97e924` 2026-02-23 — updates for auto laucnher script
- `d9ef05b64` 2026-02-23 — updates

</details>

### 7. Human override + manual steering/throttle input
*2026-02-24 · 14 commits · +67/−112 lines, 7 files*

Added human-override and manual steering/acceleration input to the sim, blended steering, alternate car models, and an out-of-road recovery patch.

<details><summary>14 commits</summary>

- `2375d63d9` 2026-02-24 — medium car sim
- `3e2309922` 2026-02-24 — revert
- `4c04c5ad4` 2026-02-24 — revert
- `5ac7eccb1` 2026-02-24 — idm policy
- `72041a4aa` 2026-02-24 — remove pedal disengage logic
- `78de57b66` 2026-02-24 — steering
- `85010fd8a` 2026-02-24 — add human override and manual input for steering and acceleration
- `ad6c39bea` 2026-02-24 — blend steering in sim
- `bcef21524` 2026-02-24 — out of road patch
- `c98853783` 2026-02-24 — revert
- `d36a8e8c7` 2026-02-24 — fix 10hz patch
- `f27113651` 2026-02-24 — car model
- `f9f0994f1` 2026-02-24 — resolution
- `fb1f5cf6f` 2026-02-24 — road length inc

</details>

### 8. Logitech wheel & pedal control, lane-centering lead
*2026-02-26 · 16 commits · +849/−458 lines, 9 files*

Integrated a Logitech racing wheel and pedals for manual driving, added pedal-update logic, made the lead vehicle center itself in the lane, and began the acceleration-smoothing work.

<details><summary>16 commits</summary>

- `1f93c5e93` 2026-02-26 — fix
- `241c2c141` 2026-02-26 — lead vehicle to center the lane
- `2496da72b` 2026-02-26 — wheel updates
- `2b667ffbe` 2026-02-26 — add logic for logitech wheel
- `32f5f0b13` 2026-02-26 — remove out of road error
- `3ed074697` 2026-02-26 — removed lane restriction
- `3f627f7fe` 2026-02-26 — default lead car
- `695a7ebc8` 2026-02-26 — removed left right automatic steering
- `7a1d8fe54` 2026-02-26 — smoothing
- `7ba2e3946` 2026-02-26 — no steering input from openpilot thru sim
- `8d57554b5` 2026-02-26 — updates to beamng logic
- `98b430eda` 2026-02-26 — pedal update logic
- `ae8fe251c` 2026-02-26 — updates lead car
- `c8d7785f2` 2026-02-26 — cleaned up bridge code
- `d52e640e6` 2026-02-26 — lane steering
- `f9e5082f2` 2026-02-26 — updates

</details>

## D · Smoothing & Scenarios

### 9. Acceleration smoothing to match BeamNG blending profile
*2026-02-27 → 2026-03-17 · 14 commits · +27,359/−27,068 lines, 20 files*

Replaced the old acceleration-blending technique so the sim's acceleration profile matches the BeamNG reference; tuned accel ratios for weak pedals and added analysis graphs.

<details><summary>14 commits</summary>

- `219e27cda` 2026-02-27 — patched clearing error
- `913b4c6d6` 2026-02-27 — update README and change tags for startup works beautifully in the smoothing push
- `99964e7e7` 2026-02-27 — removed lead from default scenario
- `3082c11e7` 2026-03-17 — add graph
- `3210bf92a` 2026-03-17 — patch for import
- `37134a541` 2026-03-17 — accel ratio
- `528f55c23` 2026-03-17 — savved dattta
- `5e0b45dce` 2026-03-17 — remove old blending technique - now matches BEAMNG blending profile
- `737369cc0` 2026-03-17 — naming scheme + graph
- `956bbbb67` 2026-03-17 — change output
- `995e9bc53` 2026-03-17 — didnt need that patch
- `be5b7848a` 2026-03-17 — data
- `c013a6d60` 2026-03-17 — patch for weak pedals.
- `e9bd2d770` 2026-03-17 — reduce accel ratio

</details>

### 10. Scenario system (-scn) & IDM removal
*2026-03-01 → 2026-03-10 · 10 commits · +2,870/−74 lines, 10 files*

Added a -scn scenario-selection argument and the CSV-driven scenario system, removed the IDM policy in favor of replayed profiles, and switched the CSV logic to m/s with two-lane support.

<details><summary>10 commits</summary>

- `362a06e46` 2026-03-01 — added scenarios -scn
- `cfabaa1c3` 2026-03-01 — remove IDM policy
- `28ad1d693` 2026-03-10 — switched logic of csv to m/s from m
- `49186934b` 2026-03-10 — out of road patch
- `708883155` 2026-03-10 — added scn arg
- `714d0bd50` 2026-03-10 — add curr location
- `752ae9196` 2026-03-10 — debug
- `d1104e3b4` 2026-03-10 — debug args
- `d9b30bd6f` 2026-03-10 — 2 lanes
- `fa7d1531f` 2026-03-10 — remove brake error

</details>

### 11. Output CSV logging, sampling rate & road-length tuning
*2026-03-12 → 2026-03-13 · 12 commits · +628/−356 lines, 19 files*

Added output-CSV logging of every run, fixed the 10 Hz sampling so runs are comparable, tuned road length, and introduced the HCC_CHANGE_NOTE tag marking every edit to upstream openpilot code.

<details><summary>12 commits</summary>

- `1aca7cc79` 2026-03-12 — inc length of road
- `3fc01f560` 2026-03-12 — made road even longer
- `5615189e6` 2026-03-12 — sample every 0.1 seconds for output csv
- `7cc1661c5` 2026-03-12 — add HCC_CHANGE_NOTE tag for updates to org code
- `7d5215c8a` 2026-03-12 — longer road
- `8e135830b` 2026-03-12 — added fixed for on road vs sim instructions which includes a toggle in dev settings (untested branch) - prev out of road branch works perfect.
- `b9e1203ac` 2026-03-12 — reverse the 10hz logic, samples outputcsv as much as possible
- `c04931b1d` 2026-03-12 — output csv added, scenario end when scn ends
- `fbbf6011e` 2026-03-12 — readability of code (rewritten for simplicity)
- `60ab5e2e2` 2026-03-13 — dec length
- `7c9e5a1f6` 2026-03-13 — default wheel, and output csv directory. Also removed the throttle that existed before
- `fe1068085` 2026-03-13 — dec road length

</details>

### 12. Trajectory & steering-smoothing experiments
*2026-03-19 · 12 commits · +139/−116 lines, 5 files*

Iterated on lane/road length and smoother acceleration-from-lead behavior, with several try/revert cycles converging on a stable time-step.

<details><summary>12 commits</summary>

- `04179de2d` 2026-03-19 — undo
- `1c33f7f0c` 2026-03-19 — new try
- `2ec995c28` 2026-03-19 — inc road length
- `39484352f` 2026-03-19 — rever changes
- `431dcfcd7` 2026-03-19 — revert changes
- `45151638b` 2026-03-19 — changes
- `5766f085c` 2026-03-19 — lane length
- `5e6130f22` 2026-03-19 — smoother acceleration from lead?
- `673317f0d` 2026-03-19 — updates
- `7ac2ed48a` 2026-03-19 — wokring again time step
- `c0d120e37` 2026-03-19 — changes
- `f79673fee` 2026-03-19 — new method

</details>

### 13. Simulation rewrite for smoothness & blending fixes
*2026-03-23 → 2026-03-25 · 29 commits · +47,389/−2,179 lines, 50 files*

Rewrote the sim/bridge path for smoothness, fixed the blending, and removed model-based trajectory projection in favor of wheel-pointing direction; added output graphs.

<details><summary>29 commits</summary>

- `06a2431f1` 2026-03-23 — graphs
- `1dea2892a` 2026-03-23 — more debugging
- `229848b81` 2026-03-23 — test again more debug
- `23ffa2f2f` 2026-03-23 — smoothness
- `2a4717f35` 2026-03-23 — links
- `2d8d782cb` 2026-03-23 — readme updates
- `31c0d22e9` 2026-03-23 — fixed blending
- `3f39707ad` 2026-03-23 — debug
- `430d599d2` 2026-03-23 — graphs
- `54860e740` 2026-03-23 — debug fixed?
- `54b9812ae` 2026-03-23 — remove old code still left behind.
- `620bd6039` 2026-03-23 — new graph
- `650b7155a` 2026-03-23 — read me
- `69055bf7e` 2026-03-23 — graphs
- `6b0eeb0cd` 2026-03-23 — readme sim update
- `7201fa6ff` 2026-03-23 — graph
- `9466d71fb` 2026-03-23 — graph
- `a7407f5d0` 2026-03-23 — simple graph
- `ab669223a` 2026-03-23 — graphs
- `adb1b839e` 2026-03-23 — graphs
- `adc795910` 2026-03-23 — removeo unnesary files
- `bfb661863` 2026-03-23 — graphs
- `d99781a78` 2026-03-23 — codex sim code
- `de5e36672` 2026-03-23 — removed those changes as they caused issues
- `ebdc80472` 2026-03-23 — data test run
- `f4d0e5f14` 2026-03-23 — remove model based trajectory projection with where wheels are pointing.
- `fce2f965c` 2026-03-23 — technical
- `31d578cf7` 2026-03-25 — graphs
- `c46552ab7` 2026-03-25 — graph

</details>

### 14. Radar handling & BeamNG scenario data
*2026-03-26 → 2026-03-27 · 23 commits · +7,999/−46,407 lines, 73 files*

Added radar-blocking so the controller follows the V2V/scenario lead rather than sensed radar, imported BeamNG CSV scenario data, and validated against smooth-graph reference tests (through test 21).

<details><summary>23 commits</summary>

- `118ba14d4` 2026-03-26 — graphs
- `19536d5fc` 2026-03-26 — graphs
- `3258915e3` 2026-03-26 — graphs
- `3a892ba35` 2026-03-26 — fix for radar
- `42315740b` 2026-03-26 — graphs
- `6ab0c8b03` 2026-03-26 — added beamng
- `70c1299c3` 2026-03-26 — patch 0.1
- `85e3d1bd9` 2026-03-26 — chagnes
- `a0fd65512` 2026-03-26 — block radar
- `b88c35a60` 2026-03-26 — graphs
- `b97117373` 2026-03-26 — graphs
- `bb77f0e9b` 2026-03-26 — csv data
- `e40d1cb1f` 2026-03-26 — track csvs in correct spot.
- `0310cba82` 2026-03-27 — debug
- `075fcc937` 2026-03-27 — revent scnv2 to be more scn
- `13ec7c49a` 2026-03-27 — radar
- `43500dcc5` 2026-03-27 — graphs
- `6b2b70785` 2026-03-27 — graphs
- `7a60cf7bb` 2026-03-27 — grraph
- `9a681d647` 2026-03-27 — debug
- `a0604c610` 2026-03-27 — graph
- `bcfe69761` 2026-03-27 — graphs
- `f4446bc17` 2026-03-27 — smooth graph test 21 (return here if need be)

</details>

## E · V2V Transport

### 15. V2V transport scaffolding
*2026-03-27 · 1 commits · +464/−3 lines, 9 files*

Laid down the first scaffolding for the vehicle-to-vehicle (V2V) data path that would later carry the lead's state to the ego.

<details><summary>1 commits</summary>

- `67c054fe3` 2026-03-27 — first commit of scaffolding

</details>

### 16. Shared V2V transport base + lead/ego paths
*2026-03-31 · 3 commits · +942/−24 lines, 20 files*

Built the shared HCC V2V transport base (packet format, publisher/subscriber) and the matching lead-side publisher and ego-side control paths — the 'identical plumbing' reused at every test rung.

<details><summary>3 commits</summary>

- `1861cf78a` 2026-03-31 — Add lead-side V2V publisher branch
- `3ef225af2` 2026-03-31 — Add shared HCC V2V transport base
- `802be6317` 2026-03-31 — Add ego-side V2V HC3 control path

</details>

### 17. V2V sim launchers (ego/lead) & manager preload fixes
*2026-04-06 → 2026-04-07 · 11 commits · +1,799/−4,651 lines, 17 files*

Added ego/lead V2V sim launchers, made openpilot's manager skip blocked process preloads in the two-instance configuration, and stabilized the HC3 V2V sim replay and lead startup.

<details><summary>11 commits</summary>

- `1cc9d772e` 2026-04-06 — Add ego V2V sim launcher
- `27edd8235` 2026-04-06 — first commit
- `5c2a6aeae` 2026-04-06 — edits
- `68d80ca60` 2026-04-06 — updated readme
- `7f82ff646` 2026-04-06 — updated readme
- `8f1a0b82c` 2026-04-06 — launcher
- `cb864d79d` 2026-04-06 — Skip blocked process preloads in manager
- `d583e324d` 2026-04-06 — Add ego V2V sim launcher
- `dce6f4bb1` 2026-04-06 — launcher
- `389b91d70` 2026-04-07 — Stabilize lead HC3 sim startup
- `ed78413e2` 2026-04-07 — Stabilize HC3 V2V sim replay

</details>

### 18. HC3 OpenPilot integration progress report (LaTeX)
*2026-04-10 · 1 commits · +576/−0 lines, 1 files*

Wrote a formal LaTeX progress report documenting the HC3/openpilot integration to date.

<details><summary>1 commits</summary>

- `0ff69161a` 2026-04-10 — docs: add HC3 OpenPilot integration progress report (LaTeX)

</details>

### 19. V2V-only HC3 transport + Lightsail cloud relay
*2026-04-15 · 3 commits · +1,538/−1,014 lines, 16 files*

Completed the V2V-only HC3 transport on both lead and ego sides and documented the AWS Lightsail cloud-relay workflow — proving the relay path over a real wide-area network (rung 3).

<details><summary>3 commits</summary>

- `815042ca6` 2026-04-15 — Document Lightsail relay sim workflow
- `e4cad059e` 2026-04-15 — Add V2V-only HC3 transport integration
- `f7298a385` 2026-04-15 — Add lead-side V2V transport integration

</details>

### 20. V2V UI toggles & param defaults
*2026-04-23 · 2 commits · +61/−6 lines, 4 files*

Added on-device UI toggles for V2V, fixed a param default, and updated the install README to use direct file writes for params.

<details><summary>2 commits</summary>

- `05735688c` 2026-04-23 — Add V2V UI toggles, fix param default, update install README
- `bb195c88a` 2026-04-23 — README: use direct file writes for params instead of Python

</details>

### 21. Git LFS fixes & install / relay-setup documentation
*2026-04-25 · 8 commits · +200/−36 lines, 2 files*

Redirected Git LFS fetches from GitLab to GitHub, fixed the LFS smudge/install steps, brought the hcc-lead install guide to parity with hcc-ego, and documented the HCC V2V relay setup.

<details><summary>8 commits</summary>

- `48ac44013` 2026-04-25 — lfsconfig: redirect LFS fetch from GitLab to GitHub
- `4a53e8503` 2026-04-25 — README: document HCC V2V relay setup
- `5fe2e2eee` 2026-04-25 — README: simplify step 11 now that LFS is fixed on GitHub
- `6e92539c2` 2026-04-25 — README: document LFS smudge workaround and safe_staging umount requirement
- `7ab907a38` 2026-04-25 — lfsconfig: redirect LFS fetch from GitLab to GitHub
- `c1bfb8e6a` 2026-04-25 — README: document HCC V2V relay setup
- `c25442721` 2026-04-25 — README: bring hcc-lead install guide to parity with hcc-ego
- `d207a7b1c` 2026-04-25 — README: fix step 11 — submodule init + selective LFS pull, not skip-smudge

</details>

## F · HIL & Device Work

### 22. HIL two-device MetaDrive bridge
*2026-05-04 · 2 commits · +2,620/−14 lines, 21 files*

Built a hardware-in-the-loop MetaDrive bridge to drive the simulated world from two real Comma 3X devices (ego + lead) — later shelved for a hardware reason (the device's USB-C port is wired to the panda, not the main computer), documented in tools/sim/hil/README.md.

<details><summary>2 commits</summary>

- `194517825` 2026-05-04 — sim/hil: add device-side HIL support for lead Comma 3X
- `82748d784` 2026-05-04 — sim/hil: add hardware-in-the-loop (HIL) MetaDrive bridge for two Comma 3X devices

</details>

### 23. Sim V2V param fixes & three operational modes
*2026-05-07 → 2026-05-08 · 8 commits · +169/−144 lines, 12 files*

Fixed the HCCV2VRelayPort param typing (INT vs str) and the OPENPILOT_PREFIX mismatch in hc3 mode, defaulted V2V off for single-device sim, documented the three operational modes, and removed stale LFS test files.

<details><summary>8 commits</summary>

- `6cb18ccd5` 2026-05-07 — Remove missing LFS files (Test14-Test22)
- `36d035e4c` 2026-05-08 — sim: fix OPENPILOT_PREFIX mismatch in hc3 mode
- `57516db41` 2026-05-08 — sim: default V2V off for single-device sim
- `5c7de9863` 2026-05-08 — docs: clarify hcc-lead branch requirement in Mode 2
- `757c28dc8` 2026-05-08 — sim: write hCCC V2V params to Params store at launch
- `895c5614c` 2026-05-08 — sim: fix HCCV2VRelayPort type — use put_int for INT param
- `e0f389f38` 2026-05-08 — sim: fix HCCV2VRelayPort — pass int not str to Params.put
- `eb032a41e` 2026-05-08 — docs: add three operational modes to sim README

</details>

### 24. SDE cleanup refactor across hCCC, sim & HIL
*2026-05-28 · 1 commits · +533/−347 lines, 18 files*

Software-engineering cleanup pass across the hCCC controller, the MetaDrive sim, and the HIL code.

<details><summary>1 commits</summary>

- `57144905e` 2026-05-28 — refactor: apply SDE cleanup across hCCC, MetaDrive sim, and HIL code

</details>

### 25. Block AGNOS auto-downgrade daemon on device
*2026-06-07 · 2 commits · +2/−2 lines, 1 files*

Blocked the openpilot updater daemon on the devices so AGNOS (the Comma 3X OS) would not auto-downgrade and undo the test setup.

<details><summary>2 commits</summary>

- `b863fb47c` 2026-06-07 — hil: block updated daemon to prevent AGNOS downgrade on device
- `bbefeb199` 2026-06-07 — hil: block updated daemon to prevent AGNOS downgrade on device

</details>

## G · Field Testing & Bench

### 26. Two-device bridge fixes, preflight & manual-input cleanup
*2026-06-11 → 2026-06-12 · 4 commits · +2,106/−48 lines, 14 files*

Fixed the two-device bridge port collision, added a preflight script and device monitor, made the sim lead follow a CSV profile by default, and removed the gas/brake-pressed floor in manual longitudinal input.

<details><summary>4 commits</summary>

- `cac41e386` 2026-06-11 — docs: checkpoint before HIL cleanup and connection setup
- `45139c4f7` 2026-06-12 — hil: lead vehicle follows CSV profile by default; add device monitor
- `84873be94` 2026-06-12 — hil: fix two-device bridge port collision, add preflight script
- `93d9f5f55` 2026-06-12 — controls: remove gasPressed/brakePressed floor in manual longitudinal input

</details>

### 27. Real-world field-test orchestration + virtual lead
*2026-06-12 · 1 commits · +2,213/−815 lines, 19 files*

Built the real-world testing toolkit: the field_test orchestrator (check / run / abort / collect / find-lead), the point-and-click UI, the virtual_lead (replays a scenario as 50 Hz V2V packets), bench receiver, and run analysis/plotting — with a 14-item preflight gate and one immutable folder per run.

<details><summary>1 commits</summary>

- `5beb1d4e9` 2026-06-12 — tools: real-world HC3 testing — virtual lead, field test orchestration, docs

</details>

### 28. V2V network setup-script hardening (AGNOS / systemd / params)
*2026-06-12 · 12 commits · +192/−70 lines, 5 files*

Hardened scripts/setup_v2v_network.sh and the relay unit against the Comma 3X's quirks: remount the read-only AGNOS rootfs for installs, normalize the systemd WorkingDirectory, run set_param as the right user with the repo venv python, write INT-typed params as int, set PYTHONPATH, switch networks last so an SSH drop can't kill setup, and resolve the device venv python for non-interactive SSH.

<details><summary>12 commits</summary>

- `0403b543c` 2026-06-12 — hcc_v2v: setup script — write integer params as int (HCCV2VRelayPort is INT-typed)
- `045260e14` 2026-06-12 — hcc_v2v: setup script — remount AGNOS rootfs rw for the relay unit install
- `19751ba2c` 2026-06-12 — hcc_v2v: setup script — use repo venv python for set_param (sudo shells don't see the interactive env)
- `27b330e7e` 2026-06-12 — hcc_v2v: setup script — normalize OPENPILOT_DIR; systemd rejects non-normalized WorkingDirectory
- `5fef9d31d` 2026-06-12 — hcc_v2v: setup script — switch networks last so WiFi SSH drop can't kill setup midway; ego role also sets EnableHCCC/AlphaLongitudinalEnabled
- `6a2da835c` 2026-06-12 — controls: hcc_v2v — tolerate int/bytes param values in _read_str_param (HCCV2VRelayPort is INT-typed; crashed load_v2v_config)
- `80d727f32` 2026-06-12 — real_world_testing: document device gotchas and bench-test status/resume point
- `92c2506d8` 2026-06-12 — hcc_v2v: relay unit — set PYTHONPATH to the repo root; running the script by path leaves openpilot unimportable
- `aa276898f` 2026-06-12 — hcc_v2v: setup script — skip relay unit install when already up to date (avoids rootfs remount on re-runs)
- `bbe705aa8` 2026-06-12 — real_world_testing: close stdin on remote nohup launches, longer launch timeouts, cleanup bench on failed start; hcc_monitor: open CSV first, flush prints
- `cd7751bd6` 2026-06-12 — hcc_v2v: setup script — run set_param as the invoking user; root lacks openpilot's python deps
- `d90c472e7` 2026-06-12 — real_world_testing: resolve the device venv python for remote commands; non-interactive ssh lacks the login env (zmq import failures)

</details>

### 29. Field-launch robustness + BENCH TEST PASSED (1750/1750, 0% loss)
*2026-06-12 · 3 commits · +112/−56 lines, 3 files*

Made remote launches disconnect-proof (read the PID then close the ssh client; detect launches that die instantly; fix the root-owned log dir) and ran the full-pipeline bench test on the two real devices: 1750 of 1750 packets delivered at 50 Hz, 0% loss, worst inter-arrival gap 51 ms vs the 100 ms staleness limit. Rung 4 passed.

<details><summary>3 commits</summary>

- `14e3b2eaf` 2026-06-12 — real_world_testing: ssh_launch — read the PID then close the ssh client ourselves
- `b57d0c5b2` 2026-06-12 — real_world_testing: detect launches that die instantly; fix root-owned log dir
- `efbbf4e4d` 2026-06-12 — real_world_testing: bench test passed — 1750/1750 packets, 0% loss, 50 Hz; next: phase-1 in-car

</details>

### 30. Docs: in-car test-day procedure + HCC_PROJECT_GUIDE
*2026-06-12 · 2 commits · +663/−253 lines, 4 files*

Wrote the offline in-car test-day procedure and safety protocol, and the HCC_PROJECT_GUIDE explaining the architecture, the testing ladder, and the decision log — setting up the next rung, the phase-1 in-car test.

<details><summary>2 commits</summary>

- `b88a7da7c` 2026-06-12 — docs: HCC_PROJECT_GUIDE — assume domain expertise, explain only the engineering
- `c2e099aa0` 2026-06-12 — docs: in-car test-day procedure + HCC_PROJECT_GUIDE for non-technical readers

</details>

