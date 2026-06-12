import csv
from pathlib import Path

from tools.real_world_testing.field_test import _extract_relay_slice

HEADER = ["recv_wall_time_us", "event_type", "role", "device_id", "source_host", "source_port",
          "dest_host", "dest_port", "seq", "a_lead", "v_lead", "forwarded", "reason"]


def _write_relay_csv(path: Path, times_us):
  with open(path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(HEADER)
    for i, t in enumerate(times_us):
      writer.writerow([t, "data", "lead", "hcc-lead", "10.42.0.34", 5000,
                       "10.42.0.1", 6000, i, 0.5, 1.0, "True", "forwarded"])


def test_extract_relay_slice_window(tmp_path):
  src = tmp_path / "full.csv"
  out = tmp_path / "slice.csv"
  _write_relay_csv(src, [1_000_000, 2_000_000, 3_000_000, 4_000_000])

  count = _extract_relay_slice(src, out, 2_000_000, 3_000_000)
  assert count == 2
  with open(out, newline="") as f:
    rows = list(csv.DictReader(f))
  assert [r["seq"] for r in rows] == ["1", "2"]


def test_extract_relay_slice_keeps_header_and_skips_garbage(tmp_path):
  src = tmp_path / "full.csv"
  out = tmp_path / "slice.csv"
  with open(src, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(HEADER)
    writer.writerow(["not-a-number", "data"])
    writer.writerow([5_000_000, "data", "lead", "d", "h", 1, "h2", 2, 0, 0.0, 0.0, "True", "forwarded"])

  count = _extract_relay_slice(src, out, 0, 10_000_000)
  assert count == 1
  with open(out, newline="") as f:
    assert next(csv.reader(f)) == HEADER


def test_extract_relay_slice_empty_window(tmp_path):
  src = tmp_path / "full.csv"
  out = tmp_path / "slice.csv"
  _write_relay_csv(src, [1_000_000])
  assert _extract_relay_slice(src, out, 5_000_000, 6_000_000) == 0
