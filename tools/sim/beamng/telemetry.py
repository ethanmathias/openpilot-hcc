import csv
import time
from pathlib import Path


OUTPUT_CSV_COLUMNS = [
  "time[s]",
  "bridge_mode",
  "hc3_variant",
  "target_speed_pre[m/s]",
  "carstate_v_ego[m/s]",
  "carstate_a_ego[m/s2]",
  "lead_d_rel[m]",
  "lead_v_rel[m/s]",
  "lead_a_rel[m/s2]",
  "planner_a_target[m/s2]",
  "hccc_accel[m/s2]",
  "manual_accel[m/s2]",
  "final_accel_cmd[m/s2]",
  "hccc_active",
  "driver_gas",
  "driver_brake",
  "controller_throttle",
  "controller_brake",
]


class TelemetryLogger:
  def __init__(self, output_csv: str | None, bridge_mode: str, hc3_variant: str | None):
    self._output_file = None
    self._csv_writer = None
    self._start_time = time.monotonic()
    self._bridge_mode = bridge_mode
    self._hc3_variant = hc3_variant or ""

    if output_csv is None:
      return

    output_path = Path(output_csv).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    self._output_file = output_path.open("w", newline="")
    self._csv_writer = csv.writer(self._output_file)
    self._csv_writer.writerow(OUTPUT_CSV_COLUMNS)

  def log(self, telemetry: dict[str, float | bool | str]):
    if self._csv_writer is None:
      return

    self._csv_writer.writerow([
      round(time.monotonic() - self._start_time, 6),
      self._bridge_mode,
      self._hc3_variant,
      float(telemetry.get("target_speed_pre", 0.0)),
      float(telemetry.get("carstate_v_ego", 0.0)),
      float(telemetry.get("carstate_a_ego", 0.0)),
      float(telemetry.get("lead_d_rel", 0.0)),
      float(telemetry.get("lead_v_rel", 0.0)),
      float(telemetry.get("lead_a_rel", 0.0)),
      float(telemetry.get("planner_a_target", 0.0)),
      float(telemetry.get("hccc_accel", 0.0)),
      float(telemetry.get("manual_accel", 0.0)),
      float(telemetry.get("final_accel", 0.0)),
      bool(telemetry.get("hccc_active", False)),
      float(telemetry.get("driver_gas", 0.0)),
      float(telemetry.get("driver_brake", 0.0)),
      float(telemetry.get("controller_throttle", 0.0)),
      float(telemetry.get("controller_brake", 0.0)),
    ])
    self._output_file.flush()

  def close(self):
    if self._output_file is not None:
      self._output_file.close()
      self._output_file = None
      self._csv_writer = None
