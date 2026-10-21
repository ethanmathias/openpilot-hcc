"""Live V2V link health, read from the relay's own CSV.

The relay runs on the ego and already writes one row per datagram with a
timestamp, the peer's role, its seq and the forward decision. That is exactly
the age / loss / last-seq a monitor needs, so the monitor tails that file
instead of opening a V2V socket -- which it must never do, because a second
subscriber claiming the ego role would steal the relay's registration from
controlsd and silently take the real controller off the air.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import wall_time_us


@dataclass
class PeerHealth:
  role: str
  last_seq: int = -1
  last_wall_time_us: int = 0
  forwarded: int = 0
  rejected: int = 0
  last_reason: str = ""

  def age_ms(self, now_us: int | None = None) -> float:
    if self.last_wall_time_us <= 0:
      return float("inf")
    now = wall_time_us() if now_us is None else int(now_us)
    return max(0.0, (now - self.last_wall_time_us) / 1000.0)

  @property
  def loss_fraction(self) -> float:
    """Fraction of this peer's expected packets that never arrived.

    seq is dense per publisher session, so the gap between the first and last
    seq is the expected count.
    """
    total = self.forwarded + self.rejected
    if self.last_seq < 0 or total == 0:
      return 0.0
    expected = self.last_seq + 1
    return max(0.0, 1.0 - (self.forwarded / expected)) if expected > 0 else 0.0


class RelayLogTailer:
  """Incrementally reads new rows from a relay CSV as it is written."""

  def __init__(self, path: str | Path):
    self.path = Path(path)
    self._offset = 0
    self._header: list[str] | None = None
    self.peers: dict[str, PeerHealth] = {}

  def poll(self) -> dict[str, PeerHealth]:
    if not self.path.exists():
      return self.peers
    try:
      with self.path.open("r", newline="") as f:
        if self._header is None:
          header_line = f.readline()
          if not header_line:
            return self.peers
          self._header = next(csv.reader([header_line]))
          self._offset = f.tell()
        f.seek(self._offset)
        rows = f.read()
        self._offset = f.tell()
    except OSError:
      return self.peers

    if not rows:
      return self.peers

    index = {name: i for i, name in enumerate(self._header or [])}
    for row in csv.reader(rows.splitlines()):
      if not row or len(row) < len(index):
        continue
      self._ingest(row, index)
    return self.peers

  def _ingest(self, row: list[str], index: dict[str, int]) -> None:
    def field(name: str) -> str:
      position = index.get(name)
      return row[position] if position is not None and position < len(row) else ""

    role = field("role")
    if role in ("", "?"):
      return
    health = self.peers.setdefault(role, PeerHealth(role=role))
    health.last_reason = field("reason")

    forwarded = field("forwarded").strip().lower() == "true"
    if field("event_type") == "hello":
      return
    if forwarded:
      health.forwarded += 1
    else:
      health.rejected += 1

    try:
      health.last_wall_time_us = int(field("recv_wall_time_us"))
    except ValueError:
      pass
    try:
      health.last_seq = max(health.last_seq, int(field("seq")))
    except ValueError:
      pass
