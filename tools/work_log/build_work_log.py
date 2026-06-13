#!/usr/bin/env python3
"""Reconstructs Ethan Mathias's work on the HC3 / hCCC openpilot project from
the git history and (a) writes a human-readable timeline (WORK_TIMELINE.md) and
(b) optionally creates + populates a GitHub Project (v2) board.

Single source of truth: the MILESTONES list below. Each milestone is a cluster
of commits grouped by theme and date, with the commit count and representative
hashes taken from `git log --author=ethanmathias@gmail.com`.

Usage:
    python3 tools/work_log/build_work_log.py            # write WORK_TIMELINE.md
    python3 tools/work_log/build_work_log.py --project  # also build GH Project

Building the GitHub Project requires the gh CLI authenticated with the
`project` scope:  gh auth refresh -s project -h github.com
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass, field

OWNER = "ethanmathias"
REPO = "openpilot-hcc"
PROJECT_TITLE = "HC3 / hCCC openpilot — Work Log"


@dataclass
class Milestone:
    n: int
    phase: str
    title: str
    start: str          # YYYY-MM-DD
    end: str            # YYYY-MM-DD
    commits: int
    summary: str
    key: list[str] = field(default_factory=list)  # "hash — subject"


# ---------------------------------------------------------------------------
# The reconstructed work, grouped into milestones. Commit counts are exact
# per-day totals from the git history; date ranges are the span of the cluster.
# ---------------------------------------------------------------------------
MILESTONES: list[Milestone] = [
    # ---- Phase A — Research & setup --------------------------------------
    Milestone(
        1, "A · Research & Setup",
        "Research, openpilot architecture study & integration plan",
        "2026-01-25", "2026-01-27", 6,
        "Surveyed the openpilot codebase, captured a written architecture/"
        "integration plan for adding the hCCC cooperative controller, and "
        "imported the BeamNG reference material the controller was being "
        "ported from.",
        ["1ce0470 — added beamng code", "60ec017 — integration plan",
         "9d6f606 — openpilot arch"],
    ),
    # ---- Phase B — Simulation & hCCC controller --------------------------
    Milestone(
        2, "B · Sim & hCCC Controller",
        "MetaDrive simulator installation & bring-up",
        "2026-02-05", "2026-02-05", 2,
        "Stood up the MetaDrive driving simulator as the rung-1 test "
        "environment so unmodified openpilot could be driven in a synthetic "
        "world.",
        ["124821b — working metadrive installation"],
    ),
    Milestone(
        3, "B · Sim & hCCC Controller",
        "hCCC controller + openpilot integration, first traffic scenarios",
        "2026-02-10", "2026-02-10", 24,
        "Wrote the hCCC controller and wired it into openpilot's longitudinal "
        "path; built the first deterministic MetaDrive traffic/lead scenarios "
        "and iterated on lead spawning.",
        ["9cc74e7 — hcc_controller code (openpilot integration)",
         "026595f — simple traffic scenario",
         "0c60535 — hccintegration test1"],
    ),
    Milestone(
        4, "B · Sim & hCCC Controller",
        "MetaDrive bridge & hCCC acceleration override",
        "2026-02-11", "2026-02-11", 27,
        "Made the sim↔openpilot bridge feed synthetic camera/vehicle signals "
        "and apply actuation back; implemented hCCC's hard acceleration "
        "override when a lead exists, with delayed-start and accel-clip "
        "smoothing.",
        ["991043624 — HCC hard-overrides accel when a lead exists; "
         "deterministic lead scenario", "c05a445 — accel clip",
         "f2fa60c — updates smoothing"],
    ),
    Milestone(
        5, "B · Sim & hCCC Controller",
        "Lead-vehicle IDM policy & live-lead integration",
        "2026-02-12", "2026-02-12", 26,
        "Gave the simulated lead an IDM (intelligent-driver-model) policy and "
        "a live-lead feed; added auto-enable of hCCC, throttle bypass, and "
        "override-stop handling for the acceleration path.",
        ["5abf5f1 — added idm policy to lead car", "69791e3 — livelead",
         "a8be85c — autoenablehcc", "fe51e58 — changes for hcc acceleration"],
    ),
    # ---- Phase C — Manual control & lead behavior ------------------------
    Milestone(
        6, "C · Manual Control",
        "Auto-launcher & multi-terminal sim workflow",
        "2026-02-23", "2026-02-23", 9,
        "Built sim_terminals.sh and an auto-launcher so a full sim session "
        "(world + openpilot + lead) starts from one command; added steering "
        "damping and quality fixes.",
        ["0e26984 — sim_terminals.sh", "d2a97e9 — updates for auto launcher script"],
    ),
    Milestone(
        7, "C · Manual Control",
        "Human override + manual steering/throttle input",
        "2026-02-24", "2026-02-24", 14,
        "Added human-override and manual steering/acceleration input to the "
        "sim, blended steering, alternate car models, and an out-of-road "
        "recovery patch.",
        ["85010fd — add human override and manual input for steering and "
         "acceleration", "ad6c39b — blend steering in sim"],
    ),
    Milestone(
        8, "C · Manual Control",
        "Logitech wheel & pedal control, lane-centering lead",
        "2026-02-26", "2026-02-26", 16,
        "Integrated a Logitech racing wheel and pedals for manual driving, "
        "added pedal-update logic, and made the lead vehicle center itself in "
        "the lane.",
        ["2b667ff — add logic for logitech wheel", "98b430e — pedal update logic",
         "241c2c1 — lead vehicle to center the lane"],
    ),
    # ---- Phase D — Smoothing & scenario system ---------------------------
    Milestone(
        9, "D · Smoothing & Scenarios",
        "Acceleration smoothing to match BeamNG blending profile",
        "2026-02-27", "2026-03-17", 14,
        "Replaced the old acceleration-blending technique so the sim's "
        "acceleration profile matches the BeamNG reference; tuned accel "
        "ratios for weak pedals and added analysis graphs.",
        ["913b4c6 — startup works beautifully in the smoothing push",
         "5e0b45d — remove old blending technique — now matches BEAMNG blending "
         "profile", "37134a5 — accel ratio"],
    ),
    Milestone(
        10, "D · Smoothing & Scenarios",
        "Scenario system (-scn) & IDM removal",
        "2026-03-01", "2026-03-10", 10,
        "Added a -scn scenario-selection argument and the CSV-driven scenario "
        "system, removed the IDM policy in favor of replayed profiles, and "
        "switched the CSV logic to m/s with two-lane support.",
        ["362a06e — added scenarios -scn", "cfabaa1 — remove IDM policy",
         "28ad1d6 — switched logic of csv to m/s from m"],
    ),
    Milestone(
        11, "D · Smoothing & Scenarios",
        "Output CSV logging, sampling rate & road-length tuning",
        "2026-03-12", "2026-03-13", 12,
        "Added output-CSV logging of every run, fixed the 10 Hz sampling so "
        "runs are comparable, tuned road length, and introduced the "
        "HCC_CHANGE_NOTE tag marking every edit to upstream openpilot code.",
        ["c04931b — output csv added, scenario end when scn ends",
         "7cc1661 — add HCC_CHANGE_NOTE tag for updates to org code",
         "b9e1203 — reverse the 10hz logic, samples outputcsv as much as possible"],
    ),
    Milestone(
        12, "D · Smoothing & Scenarios",
        "Trajectory & steering-smoothing experiments",
        "2026-03-19", "2026-03-19", 12,
        "Iterated on lane/road length and smoother acceleration-from-lead "
        "behavior, with several try/revert cycles converging on a stable "
        "time-step.",
        ["5e6130f — smoother acceleration from lead?", "7ac2ed4 — working again time step"],
    ),
    Milestone(
        13, "D · Smoothing & Scenarios",
        "Simulation rewrite for smoothness & blending fixes",
        "2026-03-23", "2026-03-25", 29,
        "Rewrote the sim/bridge path for smoothness, fixed the blending, and "
        "removed model-based trajectory projection in favor of wheel-pointing "
        "direction; added output graphs.",
        ["d99781a — codex sim code", "31c0d22 — fixed blending",
         "f4d0e5f — remove model based trajectory projection with where wheels "
         "are pointing"],
    ),
    Milestone(
        14, "D · Smoothing & Scenarios",
        "Radar handling & BeamNG scenario data",
        "2026-03-26", "2026-03-27", 23,
        "Added radar-blocking so the controller follows the V2V/scenario lead "
        "rather than sensed radar, imported BeamNG CSV scenario data, and "
        "validated against smooth-graph reference tests (through test 21).",
        ["a0fd655 — block radar", "6ab0c8b — added beamng",
         "f4446bc — smooth graph test 21 (return here if need be)"],
    ),
    # ---- Phase E — V2V transport architecture ----------------------------
    Milestone(
        15, "E · V2V Transport",
        "V2V transport scaffolding",
        "2026-03-27", "2026-03-27", 1,
        "Laid down the first scaffolding for the vehicle-to-vehicle (V2V) data "
        "path that would later carry the lead's state to the ego.",
        ["67c054f — first commit of scaffolding"],
    ),
    Milestone(
        16, "E · V2V Transport",
        "Shared V2V transport base + lead/ego paths",
        "2026-03-31", "2026-03-31", 3,
        "Built the shared HCC V2V transport base (packet format, "
        "publisher/subscriber) and the matching lead-side publisher and "
        "ego-side control paths — the 'identical plumbing' reused at every "
        "test rung.",
        ["3ef225a — Add shared HCC V2V transport base",
         "1861cf7 — Add lead-side V2V publisher branch",
         "802be63 — Add ego-side V2V HC3 control path"],
    ),
    Milestone(
        17, "E · V2V Transport",
        "V2V sim launchers (ego/lead) & manager preload fixes",
        "2026-04-06", "2026-04-07", 11,
        "Added ego/lead V2V sim launchers, made openpilot's manager skip "
        "blocked process preloads in the two-instance configuration, and "
        "stabilized the HC3 V2V sim replay and lead startup.",
        ["d583e32 — Add ego V2V sim launcher", "cb864d7 — Skip blocked process "
         "preloads in manager", "ed78413 — Stabilize HC3 V2V sim replay"],
    ),
    Milestone(
        18, "E · V2V Transport",
        "HC3 OpenPilot integration progress report (LaTeX)",
        "2026-04-10", "2026-04-10", 1,
        "Wrote a formal LaTeX progress report documenting the HC3/openpilot "
        "integration to date.",
        ["0ff6916 — docs: add HC3 OpenPilot integration progress report (LaTeX)"],
    ),
    Milestone(
        19, "E · V2V Transport",
        "V2V-only HC3 transport + Lightsail cloud relay",
        "2026-04-15", "2026-04-15", 3,
        "Completed the V2V-only HC3 transport on both lead and ego sides and "
        "documented the AWS Lightsail cloud-relay workflow — proving the "
        "relay path over a real wide-area network (rung 3).",
        ["f7298a3 — Add lead-side V2V transport integration",
         "e4cad05 — Add V2V-only HC3 transport integration",
         "815042c — Document Lightsail relay sim workflow"],
    ),
    Milestone(
        20, "E · V2V Transport",
        "V2V UI toggles & param defaults",
        "2026-04-23", "2026-04-23", 2,
        "Added on-device UI toggles for V2V, fixed a param default, and "
        "updated the install README to use direct file writes for params.",
        ["0573568 — Add V2V UI toggles, fix param default, update install README"],
    ),
    Milestone(
        21, "E · V2V Transport",
        "Git LFS fixes & install / relay-setup documentation",
        "2026-04-25", "2026-04-25", 8,
        "Redirected Git LFS fetches from GitLab to GitHub, fixed the LFS "
        "smudge/install steps, brought the hcc-lead install guide to parity "
        "with hcc-ego, and documented the HCC V2V relay setup.",
        ["7ab907a — lfsconfig: redirect LFS fetch from GitLab to GitHub",
         "c1bfb8e — README: document HCC V2V relay setup"],
    ),
    # ---- Phase F — Hardware-in-the-loop & device work --------------------
    Milestone(
        22, "F · HIL & Device Work",
        "HIL two-device MetaDrive bridge",
        "2026-05-04", "2026-05-04", 2,
        "Built a hardware-in-the-loop MetaDrive bridge to drive the simulated "
        "world from two real Comma 3X devices (ego + lead) — later shelved "
        "for a hardware reason (the device's USB-C port is wired to the panda, "
        "not the main computer), documented in tools/sim/hil/README.md.",
        ["82748d7 — add hardware-in-the-loop (HIL) MetaDrive bridge for two "
         "Comma 3X devices", "1945178 — add device-side HIL support for lead Comma 3X"],
    ),
    Milestone(
        23, "F · HIL & Device Work",
        "Sim V2V param fixes & three operational modes",
        "2026-05-07", "2026-05-08", 8,
        "Fixed the HCCV2VRelayPort param typing (INT vs str) and the "
        "OPENPILOT_PREFIX mismatch in hc3 mode, defaulted V2V off for "
        "single-device sim, and documented the three operational modes; "
        "removed stale LFS test files.",
        ["757c28d — sim: write hCCC V2V params to Params store at launch",
         "eb032a4 — docs: add three operational modes to sim README"],
    ),
    Milestone(
        24, "F · HIL & Device Work",
        "SDE cleanup refactor across hCCC, sim & HIL",
        "2026-05-28", "2026-05-28", 1,
        "Software-engineering cleanup pass across the hCCC controller, the "
        "MetaDrive sim, and the HIL code.",
        ["5714490 — refactor: apply SDE cleanup across hCCC, MetaDrive sim, and HIL code"],
    ),
    Milestone(
        25, "F · HIL & Device Work",
        "Block AGNOS auto-downgrade daemon on device",
        "2026-06-07", "2026-06-07", 2,
        "Blocked the openpilot updater daemon on the devices so AGNOS (the "
        "Comma 3X OS) would not auto-downgrade and undo the test setup.",
        ["bbefeb1 — hil: block updated daemon to prevent AGNOS downgrade on device"],
    ),
    # ---- Phase G — Real-world field testing & bench ----------------------
    Milestone(
        26, "G · Field Testing & Bench",
        "Two-device bridge fixes, preflight & manual-input cleanup",
        "2026-06-11", "2026-06-12", 4,
        "Fixed the two-device bridge port collision, added a preflight script "
        "and device monitor, made the sim lead follow a CSV profile by "
        "default, and removed the gas/brake-pressed floor in manual "
        "longitudinal input.",
        ["84873be — hil: fix two-device bridge port collision, add preflight script",
         "45139c4 — hil: lead vehicle follows CSV profile by default; add device monitor",
         "93d9f5f — controls: remove gasPressed/brakePressed floor in manual "
         "longitudinal input"],
    ),
    Milestone(
        27, "G · Field Testing & Bench",
        "Real-world field-test orchestration + virtual lead",
        "2026-06-12", "2026-06-12", 1,
        "Built the real-world testing toolkit: the field_test orchestrator "
        "(check / run / abort / collect / find-lead), the point-and-click UI, "
        "the virtual_lead (replays a scenario as 50 Hz V2V packets), bench "
        "receiver, and run analysis/plotting — with a 14-item preflight gate "
        "and one immutable folder per run.",
        ["5beb1d4 — tools: real-world HC3 testing — virtual lead, field test "
         "orchestration, docs"],
    ),
    Milestone(
        28, "G · Field Testing & Bench",
        "V2V network setup-script hardening (AGNOS / systemd / params)",
        "2026-06-12", "2026-06-12", 12,
        "Hardened scripts/setup_v2v_network.sh and the relay unit against the "
        "Comma 3X's quirks: remount the read-only AGNOS rootfs for installs, "
        "normalize the systemd WorkingDirectory, run set_param as the right "
        "user with the repo venv python, write INT-typed params as int, set "
        "PYTHONPATH, switch networks last so an SSH drop can't kill setup, and "
        "resolve the device venv python for non-interactive SSH.",
        ["045260e — remount AGNOS rootfs rw for the relay unit install",
         "5fef9d3 — switch networks last so WiFi SSH drop can't kill setup midway",
         "6a2da83 — hcc_v2v: tolerate int/bytes param values (HCCV2VRelayPort is "
         "INT-typed; crashed load_v2v_config)"],
    ),
    Milestone(
        29, "G · Field Testing & Bench",
        "Field-launch robustness + BENCH TEST PASSED (1750/1750, 0% loss)",
        "2026-06-12", "2026-06-12", 3,
        "Made remote launches disconnect-proof (read the PID then close the "
        "ssh client; detect launches that die instantly; fix the root-owned "
        "log dir) and ran the full-pipeline bench test on the two real "
        "devices: 1750 of 1750 packets delivered at 50 Hz, 0% loss, worst "
        "inter-arrival gap 51 ms vs the 100 ms staleness limit. Rung 4 passed.",
        ["14e3b2e — ssh_launch: read the PID then close the ssh client ourselves",
         "b57d0c5 — detect launches that die instantly; fix root-owned log dir",
         "efbbf4e — bench test passed — 1750/1750 packets, 0% loss, 50 Hz"],
    ),
    Milestone(
        30, "G · Field Testing & Bench",
        "Docs: in-car test-day procedure + HCC_PROJECT_GUIDE",
        "2026-06-12", "2026-06-12", 2,
        "Wrote the offline in-car test-day procedure and safety protocol, and "
        "the HCC_PROJECT_GUIDE explaining the architecture, the testing "
        "ladder, and the decision log — setting up the next rung, the phase-1 "
        "in-car test.",
        ["c2e099a — docs: in-car test-day procedure + HCC_PROJECT_GUIDE",
         "b88a7da — docs: HCC_PROJECT_GUIDE — assume domain expertise"],
    ),
]


def sh(cmd: list[str]) -> str:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{res.stderr}\n")
        res.check_returncode()
    return res.stdout.strip()


# ---------------------------------------------------------------------------
# Markdown timeline
# ---------------------------------------------------------------------------
def write_markdown(path: str) -> None:
    total_commits = sum(m.commits for m in MILESTONES)
    phases: dict[str, list[Milestone]] = {}
    for m in MILESTONES:
        phases.setdefault(m.phase, []).append(m)

    lines: list[str] = []
    lines.append("# HC3 / hCCC openpilot — Work Log & Timeline")
    lines.append("")
    lines.append("*Reconstructed from the git history "
                 "(`git log --author=ethanmathias@gmail.com`).*")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- **{total_commits} commits** authored by Ethan Mathias "
                 "across all branches (242 on the main `hcc-ego` branch; the "
                 "remainder are lead-device commits on `hcc-lead`)")
    lines.append("- **Span:** 2026-01-25 → 2026-06-12 (~4.5 months)")
    lines.append("- **Cadence (all branches):** Jan 6 · Feb 121 · Mar 101 · "
                 "Apr 25 · May 11 · Jun 24")
    lines.append("- **Outcome:** test rungs 1–4 passed (both sim modes, cloud "
                 "relay, and the on-device bench test at 0% packet loss); "
                 "phase-1 in-car test is next.")
    lines.append("")
    lines.append("## Phase totals")
    lines.append("")
    lines.append("| Phase | Milestones | Commits |")
    lines.append("|---|---|---|")
    for phase, ms in phases.items():
        lines.append(f"| {phase} | {len(ms)} | {sum(m.commits for m in ms)} |")
    lines.append("")
    for phase, ms in phases.items():
        lines.append(f"## {phase}")
        lines.append("")
        for m in ms:
            span = m.start if m.start == m.end else f"{m.start} → {m.end}"
            lines.append(f"### {m.n}. {m.title}")
            lines.append(f"*{span} · {m.commits} commits*")
            lines.append("")
            lines.append(m.summary)
            lines.append("")
            if m.key:
                lines.append("Representative commits:")
                for k in m.key:
                    lines.append(f"- `{k}`")
                lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {path}  ({total_commits} commits across {len(MILESTONES)} milestones)")


# ---------------------------------------------------------------------------
# GitHub Project (v2)
# ---------------------------------------------------------------------------
def build_project() -> None:
    # Verify the project scope is present.
    status = subprocess.run(["gh", "auth", "status"], capture_output=True, text=True)
    if "project" not in status.stderr and "project" not in status.stdout:
        sys.exit("gh token lacks the 'project' scope. Run:\n"
                 "  gh auth refresh -s project -h github.com")

    print("Creating project…")
    proj = json.loads(sh(["gh", "project", "create", "--owner", OWNER,
                          "--title", PROJECT_TITLE, "--format", "json"]))
    number = str(proj["number"])
    pid = proj["id"]
    print(f"  project #{number}  ({proj.get('url','')})")

    # Link the project to the repo so it appears on the repo's Projects tab.
    try:
        sh(["gh", "project", "link", number, "--owner", OWNER,
            "--repo", f"{OWNER}/{REPO}"])
        print(f"  linked to {OWNER}/{REPO}")
    except subprocess.CalledProcessError:
        print("  (could not auto-link to repo; link manually from the "
              "project's ⋯ menu)")

    # Custom fields.
    def field_create(name: str, dtype: str, opts: list[str] | None = None) -> dict:
        cmd = ["gh", "project", "field-create", number, "--owner", OWNER,
               "--name", name, "--data-type", dtype, "--format", "json"]
        if opts:
            cmd += ["--single-select-options", ",".join(opts)]
        return json.loads(sh(cmd))

    phase_names = list({m.phase: None for m in MILESTONES})
    f_phase = field_create("Phase", "SINGLE_SELECT", phase_names)
    f_start = field_create("Start", "DATE")
    f_end = field_create("End", "DATE")
    f_commits = field_create("Commits", "NUMBER")

    phase_opt = {o["name"]: o["id"] for o in f_phase["options"]}

    for m in MILESTONES:
        body_lines = [m.summary, "", f"**Span:** {m.start} → {m.end}  ",
                      f"**Commits:** {m.commits}"]
        if m.key:
            body_lines += ["", "**Representative commits:**"]
            body_lines += [f"- `{k}`" for k in m.key]
        body = "\n".join(body_lines)
        title = f"{m.n:02d}. {m.title}"
        item = json.loads(sh(["gh", "project", "item-create", number,
                              "--owner", OWNER, "--title", title,
                              "--body", body, "--format", "json"]))
        iid = item["id"]

        def edit(field_id: str, key: str, val: str) -> None:
            sh(["gh", "project", "item-edit", "--id", iid, "--project-id", pid,
                "--field-id", field_id, key, val])

        edit(f_start["id"], "--date", m.start)
        edit(f_end["id"], "--date", m.end)
        edit(f_commits["id"], "--number", str(m.commits))
        sh(["gh", "project", "item-edit", "--id", iid, "--project-id", pid,
            "--field-id", f_phase["id"], "--single-select-option-id",
            phase_opt[m.phase]])
        print(f"  + {title}")

    print(f"\nDone. Open: {proj.get('url','(see gh project list)')}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", action="store_true",
                    help="also create + populate the GitHub Project")
    ap.add_argument("--md", default="WORK_TIMELINE.md")
    args = ap.parse_args()
    write_markdown(args.md)
    if args.project:
        build_project()


if __name__ == "__main__":
    main()
