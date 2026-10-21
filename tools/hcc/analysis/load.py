"""Reading the CSVs a run produces.

Shared by both plotters, which previously had their own copies of this and
disagreed about edge cases.

A run folder holds:
  relay.csv        one row per datagram the relay saw, ego clock, both streams
  sent.csv         what the virtual lead actually published, lead clock
  pedal_sent.csv   what the pedal publisher published, laptop clock
  ego_monitor.csv  the ego's own response, ego clock
  metadata.json    run bounds and flags

Three clocks, so anything comparing two files has to say which it trusts.
Within relay.csv everything is already on the ego's clock, which is why the
relay log is the spine of the analysis.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

EVENT_LEAD = "data"
EVENT_PEDAL = "pedal"


@dataclass
class Stream:
  """One peer's forwarded packets, times relative to the first row seen."""
  name: str
  t_s: list[float] = field(default_factory=list)
  seq: list[int] = field(default_factory=list)
  primary: list[float] = field(default_factory=list)    # v_lead, or throttle
  secondary: list[float] = field(default_factory=list)  # a_lead, or brake
  rejects: dict[str, int] = field(default_factory=dict)

  def __len__(self) -> int:
    return len(self.t_s)

  @property
  def duration_s(self) -> float:
    return (self.t_s[-1] - self.t_s[0]) if len(self) > 1 else 0.0

  @property
  def expected(self) -> int:
    return (self.seq[-1] - self.seq[0] + 1) if len(self) else 0

  @property
  def lost(self) -> int:
    return max(0, self.expected - len(self))

  @property
  def loss_percent(self) -> float:
    return 100.0 * self.lost / self.expected if self.expected else 0.0

  @property
  def rate_hz(self) -> float:
    return len(self) / self.duration_s if self.duration_s > 0 else 0.0

  def gaps_ms(self) -> list[float]:
    return [(b - a) * 1000.0 for a, b in zip(self.t_s, self.t_s[1:], strict=False)]

  def gap_stats_ms(self) -> tuple[float, float]:
    gaps = sorted(self.gaps_ms())
    if not gaps:
      return 0.0, 0.0
    return gaps[len(gaps) // 2], gaps[-1]


def read_relay_csv(path: str | Path) -> dict[str, Stream]:
  """Split a relay CSV into one Stream per packet type, with reject tallies.

  Rows are kept on the ego's clock but rebased to the first row of each stream,
  so a stream that starts late does not read as a huge initial gap.
  """
  streams = {
    EVENT_LEAD: Stream(name="lead"),
    EVENT_PEDAL: Stream(name="pedal"),
  }
  raw: dict[str, list[tuple[int, int, float, float]]] = {EVENT_LEAD: [], EVENT_PEDAL: []}

  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      event = (row.get("event_type") or "").strip()
      if event not in streams:
        continue
      stream = streams[event]
      if (row.get("forwarded") or "").strip() == "True":
        try:
          if event == EVENT_LEAD:
            values = (float(row["v_lead"]), float(row["a_lead"]))
          else:
            values = (float(row["throttle"]), float(row["brake"]))
          raw[event].append((int(row["recv_wall_time_us"]), int(row["seq"]), *values))
        except (ValueError, KeyError, TypeError):
          continue
      else:
        reason = (row.get("reason") or "unknown").strip()
        stream.rejects[reason] = stream.rejects.get(reason, 0) + 1

  for event, rows in raw.items():
    if not rows:
      continue
    t0 = rows[0][0]
    stream = streams[event]
    for t_us, seq, primary, secondary in rows:
      stream.t_s.append((t_us - t0) / 1e6)
      stream.seq.append(seq)
      stream.primary.append(primary)
      stream.secondary.append(secondary)
  return streams


def read_sent_csv(path: str | Path) -> list[tuple[float, float, float]]:
  """(t_s, v_lead, a_lead) from a virtual_lead --log_csv file."""
  rows: list[tuple[int, float, float]] = []
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      try:
        rows.append((int(row["wall_time_us"]), float(row["v_lead_mps"]), float(row["a_lead_mps2"])))
      except (ValueError, KeyError):
        continue
  if not rows:
    return []
  t0 = rows[0][0]
  return [((t - t0) / 1e6, v, a) for t, v, a in rows]


def read_pedal_sent_csv(path: str | Path) -> list[tuple[float, float, float]]:
  """(t_s, throttle, brake) from a pedal --log_csv file."""
  rows: list[tuple[int, float, float]] = []
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      try:
        rows.append((int(row["wall_time_us"]), float(row["throttle"]), float(row["brake"])))
      except (ValueError, KeyError):
        continue
  if not rows:
    return []
  t0 = rows[0][0]
  return [((t - t0) / 1e6, throttle, brake) for t, throttle, brake in rows]


def read_monitor_csv(path: str | Path) -> dict[str, list[float]]:
  """Every numeric column of an ego_monitor CSV, by name, plus t_s."""
  columns: dict[str, list[float]] = {}
  times: list[int] = []
  with open(path, newline="") as f:
    for row in csv.DictReader(f):
      try:
        times.append(int(row["wall_time_us"]))
      except (ValueError, KeyError):
        continue
      for key, value in row.items():
        if key == "wall_time_us":
          continue
        try:
          columns.setdefault(key, []).append(float(value))
        except (TypeError, ValueError):
          columns.setdefault(key, []).append(float("nan"))
  if times:
    t0 = times[0]
    columns["t_s"] = [(t - t0) / 1e6 for t in times]
  return columns


def read_metadata(run_dir: str | Path) -> dict:
  path = Path(run_dir) / "metadata.json"
  if not path.is_file():
    return {}
  try:
    return json.loads(path.read_text())
  except json.JSONDecodeError:
    return {}
