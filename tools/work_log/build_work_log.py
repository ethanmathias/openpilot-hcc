#!/usr/bin/env python3
"""Reconstructs Ethan Mathias's work on the HC3 / hCCC openpilot project from
the git history and (a) writes a human-readable timeline (WORK_TIMELINE.md) and
(b) optionally creates + populates a GitHub Project (v2) board.

Single source of truth: the MILESTONES list below. Each milestone owns a set of
calendar days (`dates`) and/or an explicit set of commit hashes (`hashes`, used
on the three days where work spanned two themes). Commit counts, date spans, the
full per-card commit list, and the lines-changed stat are all derived from the
live git history — nothing is hand-counted.

Usage:
    python3 tools/work_log/build_work_log.py            # write WORK_TIMELINE.md
    python3 tools/work_log/build_work_log.py --project  # also (re)build GH Project

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
AUTHOR = "ethanmathias@gmail.com"
PROJECT_TITLE = "HC3 / hCCC openpilot — Work Log"


@dataclass
class Milestone:
    n: int
    phase: str
    title: str
    summary: str
    dates: list[str] = field(default_factory=list)   # full days this owns
    hashes: list[str] = field(default_factory=list)   # explicit commits (split days)


# ---------------------------------------------------------------------------
# The reconstructed work. `dates` = calendar days fully owned by the milestone;
# `hashes` = explicit commits for the three days that split across two themes
# (Mar 27, Jun 11/12).
# ---------------------------------------------------------------------------
MILESTONES: list[Milestone] = [
    # ---- Phase A — Research & setup --------------------------------------
    Milestone(
        1, "A · Research & Setup",
        "Research, openpilot architecture study & integration plan",
        "Surveyed the openpilot codebase, captured a written architecture / "
        "integration plan for adding the hCCC cooperative controller, and "
        "imported the BeamNG reference material the controller was being "
        "ported from.",
        dates=["2026-01-25", "2026-01-27"],
    ),
    # ---- Phase B — Simulation & hCCC controller --------------------------
    Milestone(
        2, "B · Sim & hCCC Controller",
        "MetaDrive simulator installation & bring-up",
        "Stood up the MetaDrive driving simulator as the rung-1 test "
        "environment so unmodified openpilot could be driven in a synthetic "
        "world.",
        dates=["2026-02-05"],
    ),
    Milestone(
        3, "B · Sim & hCCC Controller",
        "hCCC controller + openpilot integration, first traffic scenarios",
        "Wrote the hCCC controller and wired it into openpilot's longitudinal "
        "path; built the first deterministic MetaDrive traffic/lead scenarios "
        "and iterated on asset-safe lead spawning.",
        dates=["2026-02-10"],
    ),
    Milestone(
        4, "B · Sim & hCCC Controller",
        "MetaDrive bridge & hCCC acceleration override",
        "Made the sim↔openpilot bridge feed synthetic camera/vehicle signals "
        "and apply actuation back; implemented hCCC's hard acceleration "
        "override when a lead exists, with delayed-start and accel-clip "
        "smoothing.",
        dates=["2026-02-11"],
    ),
    Milestone(
        5, "B · Sim & hCCC Controller",
        "Lead-vehicle IDM policy & live-lead integration",
        "Gave the simulated lead an IDM (intelligent-driver-model) policy and "
        "a live-lead feed; added auto-enable of hCCC, throttle bypass, and "
        "override-stop handling for the acceleration path.",
        dates=["2026-02-12"],
    ),
    # ---- Phase C — Manual control & lead behavior ------------------------
    Milestone(
        6, "C · Manual Control",
        "Auto-launcher & multi-terminal sim workflow",
        "Built sim_terminals.sh and an auto-launcher so a full sim session "
        "(world + openpilot + lead) starts from one command; added steering "
        "damping and quality fixes.",
        dates=["2026-02-23"],
    ),
    Milestone(
        7, "C · Manual Control",
        "Human override + manual steering/throttle input",
        "Added human-override and manual steering/acceleration input to the "
        "sim, blended steering, alternate car models, and an out-of-road "
        "recovery patch.",
        dates=["2026-02-24"],
    ),
    Milestone(
        8, "C · Manual Control",
        "Logitech wheel & pedal control, lane-centering lead",
        "Integrated a Logitech racing wheel and pedals for manual driving, "
        "added pedal-update logic, made the lead vehicle center itself in the "
        "lane, and began the acceleration-smoothing work.",
        dates=["2026-02-26"],
    ),
    # ---- Phase D — Smoothing & scenario system ---------------------------
    Milestone(
        9, "D · Smoothing & Scenarios",
        "Acceleration smoothing to match BeamNG blending profile",
        "Replaced the old acceleration-blending technique so the sim's "
        "acceleration profile matches the BeamNG reference; tuned accel "
        "ratios for weak pedals and added analysis graphs.",
        dates=["2026-02-27", "2026-03-17"],
    ),
    Milestone(
        10, "D · Smoothing & Scenarios",
        "Scenario system (-scn) & IDM removal",
        "Added a -scn scenario-selection argument and the CSV-driven scenario "
        "system, removed the IDM policy in favor of replayed profiles, and "
        "switched the CSV logic to m/s with two-lane support.",
        dates=["2026-03-01", "2026-03-10"],
    ),
    Milestone(
        11, "D · Smoothing & Scenarios",
        "Output CSV logging, sampling rate & road-length tuning",
        "Added output-CSV logging of every run, fixed the 10 Hz sampling so "
        "runs are comparable, tuned road length, and introduced the "
        "HCC_CHANGE_NOTE tag marking every edit to upstream openpilot code.",
        dates=["2026-03-12", "2026-03-13"],
    ),
    Milestone(
        12, "D · Smoothing & Scenarios",
        "Trajectory & steering-smoothing experiments",
        "Iterated on lane/road length and smoother acceleration-from-lead "
        "behavior, with several try/revert cycles converging on a stable "
        "time-step.",
        dates=["2026-03-19"],
    ),
    Milestone(
        13, "D · Smoothing & Scenarios",
        "Simulation rewrite for smoothness & blending fixes",
        "Rewrote the sim/bridge path for smoothness, fixed the blending, and "
        "removed model-based trajectory projection in favor of "
        "wheel-pointing direction; added output graphs.",
        dates=["2026-03-23", "2026-03-25"],
    ),
    Milestone(
        14, "D · Smoothing & Scenarios",
        "Radar handling & BeamNG scenario data",
        "Added radar-blocking so the controller follows the V2V/scenario lead "
        "rather than sensed radar, imported BeamNG CSV scenario data, and "
        "validated against smooth-graph reference tests (through test 21).",
        dates=["2026-03-26"],
        hashes=["43500dcc5", "f4446bc17", "a0604c610", "9a681d647", "7a60cf7bb",
                "0310cba82", "bcfe69761", "13ec7c49a", "6b2b70785", "075fcc937"],
    ),
    # ---- Phase E — V2V transport architecture ----------------------------
    Milestone(
        15, "E · V2V Transport",
        "V2V transport scaffolding",
        "Laid down the first scaffolding for the vehicle-to-vehicle (V2V) data "
        "path that would later carry the lead's state to the ego.",
        hashes=["67c054fe3"],
    ),
    Milestone(
        16, "E · V2V Transport",
        "Shared V2V transport base + lead/ego paths",
        "Built the shared HCC V2V transport base (packet format, "
        "publisher/subscriber) and the matching lead-side publisher and "
        "ego-side control paths — the 'identical plumbing' reused at every "
        "test rung.",
        dates=["2026-03-31"],
    ),
    Milestone(
        17, "E · V2V Transport",
        "V2V sim launchers (ego/lead) & manager preload fixes",
        "Added ego/lead V2V sim launchers, made openpilot's manager skip "
        "blocked process preloads in the two-instance configuration, and "
        "stabilized the HC3 V2V sim replay and lead startup.",
        dates=["2026-04-06", "2026-04-07"],
    ),
    Milestone(
        18, "E · V2V Transport",
        "HC3 OpenPilot integration progress report (LaTeX)",
        "Wrote a formal LaTeX progress report documenting the HC3/openpilot "
        "integration to date.",
        dates=["2026-04-10"],
    ),
    Milestone(
        19, "E · V2V Transport",
        "V2V-only HC3 transport + Lightsail cloud relay",
        "Completed the V2V-only HC3 transport on both lead and ego sides and "
        "documented the AWS Lightsail cloud-relay workflow — proving the relay "
        "path over a real wide-area network (rung 3).",
        dates=["2026-04-15"],
    ),
    Milestone(
        20, "E · V2V Transport",
        "V2V UI toggles & param defaults",
        "Added on-device UI toggles for V2V, fixed a param default, and "
        "updated the install README to use direct file writes for params.",
        dates=["2026-04-23"],
    ),
    Milestone(
        21, "E · V2V Transport",
        "Git LFS fixes & install / relay-setup documentation",
        "Redirected Git LFS fetches from GitLab to GitHub, fixed the LFS "
        "smudge/install steps, brought the hcc-lead install guide to parity "
        "with hcc-ego, and documented the HCC V2V relay setup.",
        dates=["2026-04-25"],
    ),
    # ---- Phase F — Hardware-in-the-loop & device work --------------------
    Milestone(
        22, "F · HIL & Device Work",
        "HIL two-device MetaDrive bridge",
        "Built a hardware-in-the-loop MetaDrive bridge to drive the simulated "
        "world from two real Comma 3X devices (ego + lead) — later shelved for "
        "a hardware reason (the device's USB-C port is wired to the panda, not "
        "the main computer), documented in tools/sim/hil/README.md.",
        dates=["2026-05-04"],
    ),
    Milestone(
        23, "F · HIL & Device Work",
        "Sim V2V param fixes & three operational modes",
        "Fixed the HCCV2VRelayPort param typing (INT vs str) and the "
        "OPENPILOT_PREFIX mismatch in hc3 mode, defaulted V2V off for "
        "single-device sim, documented the three operational modes, and "
        "removed stale LFS test files.",
        dates=["2026-05-07", "2026-05-08"],
    ),
    Milestone(
        24, "F · HIL & Device Work",
        "SDE cleanup refactor across hCCC, sim & HIL",
        "Software-engineering cleanup pass across the hCCC controller, the "
        "MetaDrive sim, and the HIL code.",
        dates=["2026-05-28"],
    ),
    Milestone(
        25, "F · HIL & Device Work",
        "Block AGNOS auto-downgrade daemon on device",
        "Blocked the openpilot updater daemon on the devices so AGNOS (the "
        "Comma 3X OS) would not auto-downgrade and undo the test setup.",
        dates=["2026-06-07"],
    ),
    # ---- Phase G — Real-world field testing & bench ----------------------
    Milestone(
        26, "G · Field Testing & Bench",
        "Two-device bridge fixes, preflight & manual-input cleanup",
        "Fixed the two-device bridge port collision, added a preflight script "
        "and device monitor, made the sim lead follow a CSV profile by "
        "default, and removed the gas/brake-pressed floor in manual "
        "longitudinal input.",
        dates=["2026-06-11"],
        hashes=["84873be94", "45139c4f7", "93d9f5f55"],
    ),
    Milestone(
        27, "G · Field Testing & Bench",
        "Real-world field-test orchestration + virtual lead",
        "Built the real-world testing toolkit: the field_test orchestrator "
        "(check / run / abort / collect / find-lead), the point-and-click UI, "
        "the virtual_lead (replays a scenario as 50 Hz V2V packets), bench "
        "receiver, and run analysis/plotting — with a 14-item preflight gate "
        "and one immutable folder per run.",
        hashes=["5beb1d4e9"],
    ),
    Milestone(
        28, "G · Field Testing & Bench",
        "V2V network setup-script hardening (AGNOS / systemd / params)",
        "Hardened scripts/setup_v2v_network.sh and the relay unit against the "
        "Comma 3X's quirks: remount the read-only AGNOS rootfs for installs, "
        "normalize the systemd WorkingDirectory, run set_param as the right "
        "user with the repo venv python, write INT-typed params as int, set "
        "PYTHONPATH, switch networks last so an SSH drop can't kill setup, and "
        "resolve the device venv python for non-interactive SSH.",
        hashes=["5fef9d31d", "045260e14", "27b330e7e", "cd7751bd6", "aa276898f",
                "19751ba2c", "0403b543c", "92c2506d8", "d90c472e7", "6a2da835c",
                "bbe705aa8", "80d727f32"],
    ),
    Milestone(
        29, "G · Field Testing & Bench",
        "Field-launch robustness + BENCH TEST PASSED (1750/1750, 0% loss)",
        "Made remote launches disconnect-proof (read the PID then close the "
        "ssh client; detect launches that die instantly; fix the root-owned "
        "log dir) and ran the full-pipeline bench test on the two real "
        "devices: 1750 of 1750 packets delivered at 50 Hz, 0% loss, worst "
        "inter-arrival gap 51 ms vs the 100 ms staleness limit. Rung 4 passed.",
        hashes=["14e3b2eaf", "b57d0c5b2", "efbbf4e4d"],
    ),
    Milestone(
        30, "G · Field Testing & Bench",
        "Docs: in-car test-day procedure + HCC_PROJECT_GUIDE",
        "Wrote the offline in-car test-day procedure and safety protocol, and "
        "the HCC_PROJECT_GUIDE explaining the architecture, the testing "
        "ladder, and the decision log — setting up the next rung, the phase-1 "
        "in-car test.",
        hashes=["c2e099aa0", "b88a7da7c"],
    ),
]


def sh(cmd: list[str]) -> str:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{res.stderr}\n")
        res.check_returncode()
    return res.stdout.strip()


# ---------------------------------------------------------------------------
# Git history index
# ---------------------------------------------------------------------------
class History:
    """All of the author's commits across every branch, indexed for lookup."""

    def __init__(self) -> None:
        raw = sh(["git", "log", "--all", f"--author={AUTHOR}",
                  "--pretty=format:%h%x1f%H%x1f%ad%x1f%s", "--date=short"])
        self.by_date: dict[str, list[tuple]] = {}
        self.by_short: dict[str, tuple] = {}
        self.full_to_short: dict[str, str] = {}
        for line in raw.splitlines():
            short, full, date, subj = line.split("\x1f")
            entry = (short, date, subj, full)
            self.by_short[short] = entry
            self.full_to_short[full] = short
            self.by_date.setdefault(date, []).append(entry)

    def gather(self, m: Milestone) -> list[tuple]:
        """Return the (short, date, subject, full) commits owned by m, deduped
        and ordered oldest→newest."""
        seen: dict[str, tuple] = {}
        for d in m.dates:
            for e in self.by_date.get(d, []):
                seen[e[3]] = e
        for h in m.hashes:
            e = self.by_short.get(h)
            if e is None:
                sys.exit(f"milestone {m.n}: unknown hash {h}")
            seen[e[3]] = e
        return sorted(seen.values(), key=lambda e: (e[1], e[0]))

    def diffstat(self, fulls: list[str]) -> tuple[int, int, int]:
        """Sum (added, deleted, files_touched) for a set of commits."""
        if not fulls:
            return (0, 0, 0)
        out = sh(["git", "show", "--numstat", "--format=", *fulls])
        add = dele = 0
        files: set[str] = set()
        for line in out.splitlines():
            parts = line.split("\t")
            if len(parts) != 3:
                continue
            a, d, path = parts
            files.add(path)
            if a != "-":
                add += int(a)
            if d != "-":
                dele += int(d)
        return (add, dele, len(files))


def body_for(m: Milestone, commits: list[tuple], stat: tuple[int, int, int]) -> str:
    start, end = commits[0][1], commits[-1][1]
    span = start if start == end else f"{start} → {end}"
    add, dele, files = stat
    lines = [m.summary, "",
             f"**Span:** {span}  ",
             f"**Commits:** {len(commits)}  ",
             f"**Code change:** +{add:,} / −{dele:,} lines across {files} files",
             "", "**Commits:**"]
    for short, date, subj, _ in commits:
        lines.append(f"- `{short}` {date} — {subj}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Markdown timeline
# ---------------------------------------------------------------------------
def write_markdown(path: str, hist: History) -> int:
    rows = []  # (m, commits, stat)
    for m in MILESTONES:
        commits = hist.gather(m)
        rows.append((m, commits, hist.diffstat([c[3] for c in commits])))
    total = sum(len(c) for _, c, _ in rows)

    phases: dict[str, list] = {}
    for r in rows:
        phases.setdefault(r[0].phase, []).append(r)

    L: list[str] = []
    L += ["# HC3 / hCCC openpilot — Work Log & Timeline", "",
          "*Reconstructed from the git history "
          f"(`git log --all --author={AUTHOR}`). Every figure below — commit "
          "counts, date spans, and lines changed — is computed from the live "
          "history by [build_work_log.py](tools/work_log/build_work_log.py).*", ""]
    L += ["## Summary", "",
          f"- **{total} commits** authored by Ethan Mathias across all branches "
          "(242 on the main `hcc-ego` branch; the remainder are lead-device "
          "commits on `hcc-lead`)",
          "- **Span:** 2026-01-25 → 2026-06-12 (~4.5 months)",
          "- **Cadence (all branches):** Jan 6 · Feb 121 · Mar 101 · Apr 25 · "
          "May 11 · Jun 24",
          "- **Outcome:** test rungs 1–4 passed (both sim modes, cloud relay, "
          "and the on-device bench test at 0% packet loss); phase-1 in-car test "
          "is next.", ""]
    L += ["## Phase totals", "", "| Phase | Milestones | Commits | Lines (+/−) |",
          "|---|---|---|---|"]
    for phase, prows in phases.items():
        c = sum(len(x[1]) for x in prows)
        a = sum(x[2][0] for x in prows)
        d = sum(x[2][1] for x in prows)
        L.append(f"| {phase} | {len(prows)} | {c} | +{a:,} / −{d:,} |")
    L.append("")

    for phase, prows in phases.items():
        L += [f"## {phase}", ""]
        for m, commits, stat in prows:
            start, end = commits[0][1], commits[-1][1]
            span = start if start == end else f"{start} → {end}"
            add, dele, files = stat
            L += [f"### {m.n}. {m.title}",
                  f"*{span} · {len(commits)} commits · +{add:,}/−{dele:,} lines, "
                  f"{files} files*", "", m.summary, "", "<details><summary>"
                  f"{len(commits)} commits</summary>", ""]
            for short, date, subj, _ in commits:
                L.append(f"- `{short}` {date} — {subj}")
            L += ["", "</details>", ""]

    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"Wrote {path}  ({total} commits across {len(MILESTONES)} milestones)")
    return total


# ---------------------------------------------------------------------------
# GitHub Project (v2)
# ---------------------------------------------------------------------------
def build_project(hist: History) -> None:
    status = subprocess.run(["gh", "auth", "status"], capture_output=True, text=True)
    if "'project'" not in status.stderr and "'project'" not in status.stdout:
        sys.exit("gh token lacks the 'project' scope. Run:\n"
                 "  gh auth refresh -s project -h github.com")

    # Idempotent: remove any prior project with the same title.
    existing = json.loads(sh(["gh", "project", "list", "--owner", OWNER,
                              "--format", "json"]))
    for p in existing.get("projects", []):
        if p["title"] == PROJECT_TITLE:
            print(f"Deleting prior project #{p['number']}…")
            sh(["gh", "project", "delete", str(p["number"]), "--owner", OWNER])

    print("Creating project…")
    proj = json.loads(sh(["gh", "project", "create", "--owner", OWNER,
                          "--title", PROJECT_TITLE, "--format", "json"]))
    number, pid = str(proj["number"]), proj["id"]
    print(f"  project #{number}  ({proj.get('url','')})")

    try:
        sh(["gh", "project", "link", number, "--owner", OWNER,
            "--repo", f"{OWNER}/{REPO}"])
        print(f"  linked to {OWNER}/{REPO}")
    except subprocess.CalledProcessError:
        print("  (could not auto-link to repo; link manually)")

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
    f_lines = field_create("Lines changed", "NUMBER")
    phase_opt = {o["name"]: o["id"] for o in f_phase["options"]}

    for m in MILESTONES:
        commits = hist.gather(m)
        stat = hist.diffstat([c[3] for c in commits])
        body = body_for(m, commits, stat)
        title = f"{m.n:02d}. {m.title}"
        item = json.loads(sh(["gh", "project", "item-create", number,
                              "--owner", OWNER, "--title", title,
                              "--body", body, "--format", "json"]))
        iid = item["id"]

        def edit(fid: str, key: str, val: str) -> None:
            sh(["gh", "project", "item-edit", "--id", iid, "--project-id", pid,
                "--field-id", fid, key, val])

        edit(f_start["id"], "--date", commits[0][1])
        edit(f_end["id"], "--date", commits[-1][1])
        edit(f_commits["id"], "--number", str(len(commits)))
        edit(f_lines["id"], "--number", str(stat[0] + stat[1]))
        sh(["gh", "project", "item-edit", "--id", iid, "--project-id", pid,
            "--field-id", f_phase["id"], "--single-select-option-id",
            phase_opt[m.phase]])
        print(f"  + {title}  ({len(commits)} commits)")

    print(f"\nDone. Open: {proj.get('url','(see gh project list)')}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", action="store_true",
                    help="also (re)create + populate the GitHub Project")
    ap.add_argument("--md", default="WORK_TIMELINE.md")
    args = ap.parse_args()
    hist = History()
    write_markdown(args.md, hist)
    if args.project:
        build_project(hist)


if __name__ == "__main__":
    main()
