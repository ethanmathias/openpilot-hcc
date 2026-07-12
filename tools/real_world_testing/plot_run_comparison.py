#!/usr/bin/env python3
"""Plot lead (transmitted) vs ego (achieved) acceleration for in-car HC3 runs.

For every run folder under tools/real_world_testing/runs/, this writes an
`accel_comparison.png` showing the lead car's transmitted acceleration against
the ego car's actual measured acceleration on a shared time axis, in the style
of the simulator's speed-vs-time graphs. A combined montage of all runs is also
written to runs/accel_comparison_all.png.

Data sources per run folder (first match wins):

  lead acceleration:
    relay.csv     column a_lead        (ego clock, best aligned) -- standard runs
    *sent*.csv    column a_lead_mps2   (lead clock)              -- e.g. scn48_THIS_WORKED

  ego acceleration:
    ego_monitor.csv  column a_ego_mps2   (ego clock)             -- standard runs
    *probe*.csv      column a_ego        (ego clock)             -- e.g. scn48_THIS_WORKED

When both series come from the ego clock they share one origin, so the traces
line up on the real timeline (the lead trace starts later by the start-delay).
When the lead series is on the lead clock, each series is zeroed to its own
first sample (approximate alignment); this is noted in the printed summary.

Usage:
  python3 tools/real_world_testing/plot_run_comparison.py               # all runs
  python3 tools/real_world_testing/plot_run_comparison.py runs/run_XXXX # one run
  python3 tools/real_world_testing/plot_run_comparison.py --with_cmd    # overlay commanded accel
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

RUNS_DIR = Path(__file__).resolve().parent / "runs"
STALE_LABEL = "commanded (out_accel)"


class Series:
  """A time series in seconds with its acceleration values and clock origin."""

  def __init__(self, t_us: list[int], a: list[float], clock: str, source: str):
    self.t_us = t_us
    self.a = a
    self.clock = clock          # "ego" or "lead"
    self.source = source        # filename it came from

  def __len__(self) -> int:
    return len(self.t_us)


def _read_column(path: Path, t_key: str, a_key: str) -> tuple[list[int], list[float]] | None:
  t_us: list[int] = []
  a: list[float] = []
  with open(path, newline="") as f:
    reader = csv.DictReader(f)
    if reader.fieldnames is None or t_key not in reader.fieldnames or a_key not in reader.fieldnames:
      return None
    for row in reader:
      raw_t, raw_a = row.get(t_key), row.get(a_key)
      if not raw_t or raw_a in (None, "", "nan", "NaN"):
        continue
      try:
        av = float(raw_a)
      except ValueError:
        continue
      if not math.isfinite(av):
        continue
      t_us.append(int(raw_t))
      a.append(av)
  return (t_us, a) if t_us else None


def load_lead_accel(folder: Path) -> Series | None:
  relay = folder / "relay.csv"
  if relay.exists():
    t_us, a = [], []
    with open(relay, newline="") as f:
      for row in csv.DictReader(f):
        if row.get("event_type") != "data" or not row.get("a_lead"):
          continue
        try:
          av = float(row["a_lead"])
        except ValueError:
          continue
        if math.isfinite(av):
          t_us.append(int(row["recv_wall_time_us"]))
          a.append(av)
    if t_us:
      order = sorted(range(len(t_us)), key=lambda i: t_us[i])
      return Series([t_us[i] for i in order], [a[i] for i in order], "ego", "relay.csv")

  for sent in sorted(folder.glob("*sent*.csv")):
    got = _read_column(sent, "wall_time_us", "a_lead_mps2")
    if got:
      return Series(got[0], got[1], "lead", sent.name)
  return None


def load_ego_accel(folder: Path) -> Series | None:
  monitor = folder / "ego_monitor.csv"
  if monitor.exists():
    got = _read_column(monitor, "wall_time_us", "a_ego_mps2")
    if got:
      return Series(got[0], got[1], "ego", "ego_monitor.csv")
  for probe in sorted(folder.glob("*probe*.csv")):
    got = _read_column(probe, "wall_time_us", "a_ego")
    if got:
      return Series(got[0], got[1], "ego", probe.name)
  return None


def load_cmd_accel(folder: Path) -> Series | None:
  monitor = folder / "ego_monitor.csv"
  if monitor.exists():
    got = _read_column(monitor, "wall_time_us", "out_accel")
    if got:
      return Series(got[0], got[1], "ego", "ego_monitor.csv")
  for probe in sorted(folder.glob("*probe*.csv")):
    got = _read_column(probe, "wall_time_us", "out_accel")
    if got:
      return Series(got[0], got[1], "ego", probe.name)
  return None


def align(lead: Series, ego: Series, extra: Series | None):
  """Return (t0_us for each series to subtract, shared_clock: bool)."""
  shared = lead.clock == ego.clock
  if shared:
    origin = min(lead.t_us[0], ego.t_us[0])
    if extra is not None:
      origin = min(origin, extra.t_us[0])
    return {"lead": origin, "ego": origin, "cmd": origin}, True
  # Different clocks: zero each series to its own start.
  return {"lead": lead.t_us[0], "ego": ego.t_us[0],
          "cmd": extra.t_us[0] if extra is not None else 0}, False


def seconds(series: Series, origin_us: int) -> list[float]:
  return [(t - origin_us) / 1e6 for t in series.t_us]


def correlation(lead: Series, ego: Series, origin: dict, shared: bool) -> float | None:
  """Pearson correlation of ego accel resampled onto lead timestamps (shared clock only)."""
  if not shared:
    return None
  try:
    import numpy as np
  except ImportError:
    return None
  lt = np.array(seconds(lead, origin["lead"]))
  la = np.array(lead.a)
  et = np.array(seconds(ego, origin["ego"]))
  ea = np.array(ego.a)
  lo, hi = max(lt[0], et[0]), min(lt[-1], et[-1])
  mask = (lt >= lo) & (lt <= hi)
  if mask.sum() < 5:
    return None
  ego_on_lead = np.interp(lt[mask], et, ea)
  la_win = la[mask]
  if la_win.std() < 1e-6 or ego_on_lead.std() < 1e-6:
    return None
  return float(np.corrcoef(la_win, ego_on_lead)[0, 1])


def plot_single(ax, run_id, lead, ego, cmd, origin, shared) -> None:
  ax.plot(seconds(lead, origin["lead"]), lead.a, lw=1.0, color="tab:blue",
          label="Lead (transmitted)")
  ax.plot(seconds(ego, origin["ego"]), ego.a, lw=1.0, color="tab:orange",
          label="Ego (achieved)")
  if cmd is not None:
    ax.plot(seconds(cmd, origin["cmd"]), cmd.a, lw=0.8, color="tab:green",
            ls="--", alpha=0.7, label=STALE_LABEL)
  ax.axhline(0.0, color="k", lw=0.6, alpha=0.4)
  ax.set_ylabel("Acceleration [m/s^2]")
  ax.grid(True, alpha=0.3)
  title = run_id if shared else f"{run_id} (approx. time alignment)"
  ax.set_title(title, fontsize=9)


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("runs", nargs="*", help="Run folders (default: every folder under runs/)")
  parser.add_argument("--runs_dir", default=str(RUNS_DIR), help="Base runs directory")
  parser.add_argument("--with_cmd", action="store_true",
                      help="Also overlay the commanded acceleration (out_accel)")
  parser.add_argument("--out_name", default="accel_comparison.png",
                      help="Per-run PNG filename written into each run folder")
  parser.add_argument("--no_summary", action="store_true", help="Skip the combined montage")
  args = parser.parse_args(argv)

  if args.runs:
    folders = [Path(p) for p in args.runs]
  else:
    base = Path(args.runs_dir)
    folders = sorted(p for p in base.iterdir() if p.is_dir())

  try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
  except ImportError:
    print("matplotlib not installed (pip install matplotlib)")
    return 1

  plotted = []
  skipped = []
  for folder in folders:
    if not folder.is_dir():
      skipped.append((folder.name, "not a directory"))
      continue
    lead = load_lead_accel(folder)
    ego = load_ego_accel(folder)
    if lead is None or ego is None:
      missing = ", ".join(m for m, present in (("lead accel", lead), ("ego accel", ego)) if present is None)
      skipped.append((folder.name, f"no {missing}"))
      continue
    cmd = load_cmd_accel(folder) if args.with_cmd else None
    origin, shared = align(lead, ego, cmd)

    fig, ax = plt.subplots(figsize=(11, 5))
    plot_single(ax, folder.name, lead, ego, cmd, origin, shared)
    ax.set_xlabel("Time [s]")
    ax.legend(loc="best", fontsize=8)
    fig.suptitle("Lead vs ego acceleration")
    fig.tight_layout()
    out_path = folder / args.out_name
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    r = correlation(lead, ego, origin, shared)
    plotted.append((folder.name, lead, ego, cmd, origin, shared, r))
    r_txt = f"corr={r:+.2f}" if r is not None else "corr=n/a"
    print(f"{folder.name:44s}  lead[{lead.source}] {len(lead):5d}  "
          f"ego[{ego.source}] {len(ego):5d}  {r_txt}  -> {out_path.name}")

  for name, why in skipped:
    print(f"  skip {name}: {why}")

  if plotted and not args.no_summary:
    n = len(plotted)
    cols = 3 if n > 4 else max(1, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5.2, rows * 3.0), squeeze=False)
    for idx, (name, lead, ego, cmd, origin, shared, _r) in enumerate(plotted):
      ax = axes[idx // cols][idx % cols]
      plot_single(ax, name, lead, ego, cmd, origin, shared)
      if idx // cols == rows - 1:
        ax.set_xlabel("Time [s]")
    for j in range(n, rows * cols):
      axes[j // cols][j % cols].axis("off")
    axes[0][0].legend(loc="best", fontsize=7)
    fig.suptitle("Lead vs ego acceleration -- all runs", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    summary_path = Path(args.runs_dir) / "accel_comparison_all.png"
    fig.savefig(summary_path, dpi=120)
    plt.close(fig)
    print(f"\nmontage of {n} runs -> {summary_path}")

  print(f"\nplotted {len(plotted)} run(s), skipped {len(skipped)}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
