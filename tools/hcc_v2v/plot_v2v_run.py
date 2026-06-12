#!/usr/bin/env python3
"""Summarize and plot one V2V run from the relay's CSV log.

The relay (tools/hcc_v2v/relay_server.py) logs every packet it sees when run
with --log_csv — the on-ego systemd unit installed by setup_v2v_network.sh
does this automatically into /data/hcc_v2v_logs/. Pull a log off the ego and
run:

    python3 tools/hcc_v2v/plot_v2v_run.py relay_20260612_141500.csv

Prints packet/loss/rate statistics and writes a PNG next to the CSV with
v_lead and a_lead over time plus inter-arrival gaps. Optionally overlay what
the virtual lead actually sent (its --log_csv output) to spot relay drops:

    python3 tools/hcc_v2v/plot_v2v_run.py relay.csv --sent virtual_lead.csv --out run.png
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def read_relay_csv(path: Path):
  """Return forwarded data rows as (t_s, seq, v_lead, a_lead), t relative to first row."""
  rows = []
  rejects: dict[str, int] = {}
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      if row.get("event_type") != "data":
        continue
      if row.get("forwarded") == "True":
        rows.append((int(row["recv_wall_time_us"]), int(row["seq"]),
                     float(row["v_lead"]), float(row["a_lead"])))
      else:
        reason = row.get("reason", "unknown")
        rejects[reason] = rejects.get(reason, 0) + 1
  if not rows:
    raise SystemExit(f"no forwarded data packets in {path} (rejections: {rejects or 'none'})")
  t0 = rows[0][0]
  return [((t - t0) / 1e6, seq, v, a) for t, seq, v, a in rows], rejects


def read_sent_csv(path: Path):
  """Return (t_s, v, a) rows from a virtual_lead --log_csv file, t relative to first row."""
  rows = []
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      rows.append((int(row["wall_time_us"]), float(row["v_lead_mps"]), float(row["a_lead_mps2"])))
  if not rows:
    return []
  t0 = rows[0][0]
  return [((t - t0) / 1e6, v, a) for t, v, a in rows]


def summarize(rows, rejects) -> None:
  ts = [r[0] for r in rows]
  seqs = [r[1] for r in rows]
  duration = ts[-1] - ts[0]
  expected = seqs[-1] - seqs[0] + 1
  lost = expected - len(rows)
  gaps = [b - a for a, b in zip(ts, ts[1:])]
  gaps_sorted = sorted(gaps)
  median_gap = gaps_sorted[len(gaps_sorted) // 2] if gaps_sorted else 0.0
  max_gap = max(gaps) if gaps else 0.0

  print(f"forwarded packets : {len(rows)}")
  print(f"duration          : {duration:.1f} s ({len(rows) / duration:.1f} Hz)" if duration > 0 else "duration          : 0 s")
  print(f"seq range         : {seqs[0]} .. {seqs[-1]}")
  print(f"lost in transit   : {lost} ({100.0 * lost / expected:.2f}%)")
  print(f"inter-arrival gap : median {median_gap * 1000:.1f} ms, max {max_gap * 1000:.1f} ms")
  if rejects:
    print(f"relay rejections  : {rejects}")


def plot(rows, sent, out_path: Path) -> None:
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt

  ts = [r[0] for r in rows]
  vs = [r[2] for r in rows]
  accels = [r[3] for r in rows]
  gaps_t = ts[1:]
  gaps_ms = [(b - a) * 1000.0 for a, b in zip(ts, ts[1:])]

  fig, (ax_v, ax_a, ax_gap) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
  ax_v.plot(ts, vs, lw=0.9, label="v_lead (relayed)")
  if sent:
    ax_v.plot([s[0] for s in sent], [s[1] for s in sent], lw=0.9, ls="--", alpha=0.7, label="v_lead (sent)")
  ax_v.set_ylabel("speed (m/s)")
  ax_v.legend(loc="best", fontsize=8)
  ax_v.grid(alpha=0.3)

  ax_a.plot(ts, accels, lw=0.9, color="tab:orange", label="a_lead (relayed)")
  if sent:
    ax_a.plot([s[0] for s in sent], [s[2] for s in sent], lw=0.9, ls="--", alpha=0.7, label="a_lead (sent)")
  ax_a.set_ylabel("accel (m/s²)")
  ax_a.ticklabel_format(useOffset=False, axis="y")
  ax_a.legend(loc="best", fontsize=8)
  ax_a.grid(alpha=0.3)

  ax_gap.plot(gaps_t, gaps_ms, lw=0.6, color="tab:red")
  ax_gap.axhline(100.0, ls=":", color="k", lw=0.8)  # ego staleness threshold
  ax_gap.set_ylabel("inter-arrival (ms)")
  ax_gap.set_xlabel("time since first packet (s)")
  ax_gap.grid(alpha=0.3)

  fig.suptitle("V2V relay run")
  fig.tight_layout()
  fig.savefig(out_path, dpi=130)
  print(f"plot written to {out_path}")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("relay_csv", help="Relay --log_csv output (from /data/hcc_v2v_logs/ on the ego)")
  parser.add_argument("--sent", default=None, help="Optional virtual_lead --log_csv to overlay")
  parser.add_argument("--out", default=None, help="Output PNG path (default: next to the relay CSV)")
  parser.add_argument("--no_plot", action="store_true", help="Stats only, skip the PNG")
  args = parser.parse_args(argv)

  relay_path = Path(args.relay_csv).expanduser()
  rows, rejects = read_relay_csv(relay_path)
  summarize(rows, rejects)

  if not args.no_plot:
    sent = read_sent_csv(Path(args.sent).expanduser()) if args.sent else []
    out_path = Path(args.out).expanduser() if args.out else relay_path.with_suffix(".png")
    try:
      plot(rows, sent, out_path)
    except ImportError:
      print("matplotlib not installed — stats only (pip install matplotlib)", file=sys.stderr)
      return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
