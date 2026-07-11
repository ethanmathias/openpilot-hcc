#!/usr/bin/env python3
"""Live HC3/V2V status line, printed once per second. Run ON a device:

    cd /data/openpilot && python3 tools/hcc_v2v/scripts/hcc_monitor.py

Answers the bring-up questions in one place: is the device onroad, is it
engaged, what accel is HC3 commanding, and (on the ego) is the V2V signal
fresh. Reads passively from local msgq + the V2V mode transitions that
longcontrol writes to cloudlog — it never opens a V2V socket itself, so it
cannot steal the relay's ego registration from the real subscriber.

With --log_csv it also records every sample to a CSV (at --hz, default 10)
so the ego's actual response can be reviewed after a test run — this is what
tools/real_world_testing/field_test.py runs on the ego during a field test.

controlsState debug field mapping (see controlsd.py):
  upAccelCmd = planner accel, uiAccelCmd = HC3 accel, ufAccelCmd = manual accel
"""
import argparse
import csv
import os
import time

import cereal.messaging as messaging
from openpilot.common.params import Params

CSV_COLUMNS = ["wall_time_us", "v_ego_mps", "a_ego_mps2", "engaged", "long_state",
               "hccc_accel", "manual_accel", "planner_accel",
               "cmd_accel", "out_accel",
               "gas_pressed", "brake_pressed", "standstill",
               "carstate_valid", "controlsstate_valid", "selfdrivestate_valid"]


def main() -> None:
  parser = argparse.ArgumentParser(description="Live HC3/V2V status monitor.")
  parser.add_argument("--log_csv", default=None, help="Also record every sample to this CSV")
  parser.add_argument("--hz", type=float, default=10.0, help="CSV sample rate (status line stays at 1 Hz)")
  args = parser.parse_args()

  # Open the CSV before anything that could block or crash (SubMaster, params),
  # so a run always leaves at least a header behind for collection.
  csv_file = None
  csv_writer = None
  if args.log_csv:
    os.makedirs(os.path.dirname(os.path.abspath(args.log_csv)), exist_ok=True)
    csv_file = open(args.log_csv, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_COLUMNS)
    csv_file.flush()
    print(f"hcc_monitor: logging {args.hz:.0f} Hz samples to {args.log_csv}", flush=True)

  sm = messaging.SubMaster(['carState', 'controlsState', 'selfdriveState', 'carControl', 'carOutput'])
  params = Params()
  v2v_only = params.get_bool("HCCV2VOnly") if params.check_key("HCCV2VOnly") else False
  role = "ego" if v2v_only else "?"
  print(f"hcc_monitor: role guess={role} (HCCV2VOnly={v2v_only}); Ctrl-C to stop", flush=True)

  interval = 1.0 / max(args.hz, 0.1)
  next_sample = time.monotonic()
  last_print = 0.0

  try:
    while True:
      sm.update(100)
      now = time.monotonic()
      if now < next_sample:
        continue
      next_sample += interval
      if next_sample < now:  # fell behind; don't burst
        next_sample = now + interval

      cs_ok = sm.valid['carState']
      ctrl_ok = sm.valid['controlsState']
      sds_ok = sm.valid['selfdriveState']

      cs = sm['carState']
      v_ego = cs.vEgo if cs_ok else float("nan")
      a_ego = cs.aEgo if cs_ok else float("nan")
      gas_pressed = bool(cs.gasPressed) if cs_ok else False
      brake_pressed = bool(cs.brakePressed) if cs_ok else False
      standstill = bool(cs.standstill) if cs_ok else False
      engaged = sm['selfdriveState'].active if sds_ok else False
      state = sm['controlsState'].longControlState if ctrl_ok else "?"
      planner = sm['controlsState'].upAccelCmd if ctrl_ok else float("nan")
      hccc = sm['controlsState'].uiAccelCmd if ctrl_ok else float("nan")
      manual = sm['controlsState'].ufAccelCmd if ctrl_ok else float("nan")
      cmd_accel = sm['carControl'].actuators.accel
      out_accel = sm['carOutput'].actuatorsOutput.accel

      if csv_writer is not None:
        csv_writer.writerow([time.time_ns() // 1000, f"{v_ego:.4f}", f"{a_ego:.4f}", int(bool(engaged)), state,
                             f"{hccc:.4f}", f"{manual:.4f}", f"{planner:.4f}",
                             f"{cmd_accel:.4f}", f"{out_accel:.4f}",
                             int(gas_pressed), int(brake_pressed), int(standstill),
                             int(cs_ok), int(ctrl_ok), int(sds_ok)])
        csv_file.flush()

      if now - last_print >= 1.0:
        last_print = now
        alive = "".join("+" if ok else "-" for ok in (cs_ok, ctrl_ok, sds_ok))
        print(f"[{time.strftime('%H:%M:%S')}] alive={alive} engaged={engaged!s:5} "
              f"v={v_ego:5.2f} m/s aEgo={a_ego:+.2f}  long={state}  "
              f"accel: hccc={hccc:+.2f} cmd={cmd_accel:+.2f} out={out_accel:+.2f} planner={planner:+.2f}",
              flush=True)
  finally:
    if csv_file is not None:
      csv_file.close()


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
