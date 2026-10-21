#!/usr/bin/env python3
"""Link health for one run: packet statistics and a plot, from the relay CSV.

The relay logs every datagram it sees, so this is the file that answers "did
the data get there" -- separately for each stream, since the Fall-26 setup has
two publishers on one relay and they fail independently.

    python3 tools/hcc/analysis/plot_run.py runs/<run_id>/relay.csv

`hcc run` calls this automatically at the end of a run and saves the output as
stats.txt next to the plot.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  DEFAULT_PEDAL_STALE_THRESHOLD_MS,
  DEFAULT_STALE_THRESHOLD_MS,
)
from openpilot.tools.hcc.analysis.load import (
  EVENT_LEAD,
  EVENT_PEDAL,
  Stream,
  read_pedal_sent_csv,
  read_relay_csv,
  read_sent_csv,
)

STALE_MS = {EVENT_LEAD: DEFAULT_STALE_THRESHOLD_MS, EVENT_PEDAL: DEFAULT_PEDAL_STALE_THRESHOLD_MS}
LABELS = {
  EVENT_LEAD: ("v_lead (m/s)", "a_lead (m/s2)"),
  EVENT_PEDAL: ("throttle", "brake"),
}


def summarize(stream: Stream, stale_ms: float) -> None:
  print(f"--- {stream.name} stream ---")
  if not len(stream):
    print(f"  no forwarded packets" + (f" (rejections: {stream.rejects})" if stream.rejects else ""))
    return

  median_gap, max_gap = stream.gap_stats_ms()
  over = sum(1 for gap in stream.gaps_ms() if gap > stale_ms)
  print(f"  forwarded packets : {len(stream)}")
  print(f"  duration          : {stream.duration_s:.1f} s ({stream.rate_hz:.1f} Hz)")
  print(f"  seq range         : {stream.seq[0]} .. {stream.seq[-1]}")
  print(f"  lost in transit   : {stream.lost} ({stream.loss_percent:.2f}%)")
  print(f"  inter-arrival gap : median {median_gap:.1f} ms, max {max_gap:.1f} ms")
  # A gap past the staleness threshold is a window where the ego treated this
  # stream as faulted, which matters more than the average rate.
  print(f"  gaps over {stale_ms:.0f} ms   : {over}"
        + ("  <-- the ego saw this stream as stale here" if over else ""))
  if stream.rejects:
    print(f"  relay rejections  : {stream.rejects}")


def plot(streams: dict[str, Stream], sent_lead, sent_pedal, out_path: Path) -> None:
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt

  present = [key for key, stream in streams.items() if len(stream)]
  if not present:
    raise SystemExit("nothing to plot")

  # Two value rows per present stream, plus one shared inter-arrival row.
  n_rows = 2 * len(present) + 1
  fig, axes = plt.subplots(n_rows, 1, figsize=(11, 2.3 * n_rows), sharex=True)
  axes = list(axes) if n_rows > 1 else [axes]

  row = 0
  for key in present:
    stream = streams[key]
    primary_label, secondary_label = LABELS[key]
    sent = sent_lead if key == EVENT_LEAD else sent_pedal

    for values, label, color, sent_index in ((stream.primary, primary_label, "tab:blue", 1),
                                             (stream.secondary, secondary_label, "tab:orange", 2)):
      ax = axes[row]
      ax.plot(stream.t_s, values, lw=0.9, color=color, label=f"{label} (relayed)")
      if sent:
        ax.plot([s[0] for s in sent], [s[sent_index] for s in sent],
                lw=0.9, ls="--", alpha=0.7, color="k", label=f"{label} (sent)")
      ax.set_ylabel(label)
      ax.ticklabel_format(useOffset=False, axis="y")
      # A constant trace otherwise gets an axis of floating-point noise
      # (0.5 +/- 4e-13), which reads as if the signal were jittering.
      if values and (max(values) - min(values)) < 1e-6:
        centre = values[0]
        ax.set_ylim(centre - 0.5, centre + 0.5)
      ax.legend(loc="best", fontsize=8)
      ax.grid(alpha=0.3)
      row += 1

  ax_gap = axes[row]
  for key in present:
    stream = streams[key]
    ax_gap.plot(stream.t_s[1:], stream.gaps_ms(), lw=0.6, label=f"{stream.name} inter-arrival")
    ax_gap.axhline(STALE_MS[key], ls=":", lw=0.8,
                   label=f"{stream.name} staleness ({STALE_MS[key]:.0f} ms)")
  ax_gap.set_ylabel("inter-arrival (ms)")
  ax_gap.set_xlabel("time since first packet (s)")
  ax_gap.legend(loc="best", fontsize=8)
  ax_gap.grid(alpha=0.3)

  fig.suptitle("V2V link health")
  fig.tight_layout()
  fig.savefig(out_path, dpi=130)
  print(f"plot written to {out_path}")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("relay_csv", help="Relay CSV from a run folder (or /data/hcc_v2v_logs on the ego)")
  parser.add_argument("--sent", default=None, help="virtual_lead --log_csv to overlay")
  parser.add_argument("--pedal_sent", default=None, help="pedal --log_csv to overlay")
  parser.add_argument("--out", default=None, help="Output PNG (default: next to the relay CSV)")
  parser.add_argument("--no_plot", action="store_true", help="Statistics only")
  args = parser.parse_args(argv)

  relay_path = Path(args.relay_csv).expanduser()
  streams = read_relay_csv(relay_path)

  if not any(len(stream) for stream in streams.values()):
    rejects = {key: stream.rejects for key, stream in streams.items() if stream.rejects}
    raise SystemExit(f"no forwarded packets in {relay_path} (rejections: {rejects or 'none'})")

  for key, stream in streams.items():
    if len(stream) or stream.rejects:
      summarize(stream, STALE_MS[key])

  if args.no_plot:
    return 0

  sent_lead = read_sent_csv(Path(args.sent).expanduser()) if args.sent else []
  sent_pedal = read_pedal_sent_csv(Path(args.pedal_sent).expanduser()) if args.pedal_sent else []
  out_path = Path(args.out).expanduser() if args.out else relay_path.with_suffix(".png")
  try:
    plot(streams, sent_lead, sent_pedal, out_path)
  except ImportError:
    print("matplotlib not installed — statistics only (uv pip install matplotlib)", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
