#!/usr/bin/env python3
"""Live HC3 status line, printed once per second. Runs ON a device.

    cd /data/openpilot && python3 tools/hcc/device/monitor.py

Answers the bring-up questions in one place: is the device onroad, is it
engaged, what accel is the hCCC controller commanding, is the human blend term
actually moving, and are the V2V links fresh.

Reads passively from local msgq. It never opens a V2V socket, because a second
subscriber claiming the ego role would steal the relay's registration from
controlsd. Link health comes from the relay's own CSV instead -- pass
--relay_csv to enable it.

With --log_csv it records every sample so the ego's response can be reviewed
after a run; this is what `hcc run` starts on the ego.

controlsState debug field mapping (see controlsd.py): while the hCCC
controller is driving, upAccelCmd = planner accel, uiAccelCmd = hCCC accel,
ufAccelCmd = manual (human) accel.

The vision-lead columns are recorded but NOT used for control in V2V-only
mode. They are here for the future three-vehicle configuration, where the
unconnected middle vehicle is tracked by onboard sensing. On a Tesla
radarUnavailable is True, so these are vision-model estimates, not radar
measurements -- lead_is_radar records which.
"""
from __future__ import annotations

import argparse
import csv
import os
import time

import cereal.messaging as messaging
from openpilot.tools.hcc.device.link_health import RelayLogTailer

CSV_COLUMNS = [
  "wall_time_us", "v_ego_mps", "a_ego_mps2", "engaged", "long_state",
  "hccc_accel", "manual_accel", "planner_accel",
  "cmd_accel", "out_accel",
  "gas_pressed", "brake_pressed", "standstill",
  # Vision lead (radarState.leadOne) -- logged only, never used for control here.
  "lead_status", "lead_d_rel", "lead_v_rel", "lead_v_lead", "lead_a_lead_k",
  "lead_model_prob", "lead_is_radar",
  "carstate_valid", "controlsstate_valid", "selfdrivestate_valid", "radarstate_valid",
]


def _parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Live HC3 status monitor.")
  parser.add_argument("--log_csv", default=None, help="Also record every sample to this CSV")
  parser.add_argument("--hz", type=float, default=10.0, help="CSV sample rate (status line stays at 1 Hz)")
  parser.add_argument("--relay_csv", default=None,
                      help="Relay CSV to tail for live V2V/pedal link health")
  return parser.parse_args()


def main() -> None:
  args = _parse_args()

  # Open the CSV before anything that could block or crash, so a run always
  # leaves at least a header behind for collection.
  csv_file = None
  csv_writer = None
  if args.log_csv:
    os.makedirs(os.path.dirname(os.path.abspath(args.log_csv)), exist_ok=True)
    csv_file = open(args.log_csv, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_COLUMNS)
    csv_file.flush()
    print(f"monitor: logging {args.hz:.0f} Hz samples to {args.log_csv}", flush=True)

  tailer = RelayLogTailer(args.relay_csv) if args.relay_csv else None

  sm = messaging.SubMaster(['carState', 'controlsState', 'selfdriveState',
                            'carControl', 'carOutput', 'radarState'])
  print("monitor: started; Ctrl-C to stop", flush=True)

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
      radar_ok = sm.valid['radarState']

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

      lead = sm['radarState'].leadOne
      lead_status = bool(lead.status) if radar_ok else False

      if csv_writer is not None:
        csv_writer.writerow([
          time.time_ns() // 1000, f"{v_ego:.4f}", f"{a_ego:.4f}", int(bool(engaged)), state,
          f"{hccc:.4f}", f"{manual:.4f}", f"{planner:.4f}",
          f"{cmd_accel:.4f}", f"{out_accel:.4f}",
          int(gas_pressed), int(brake_pressed), int(standstill),
          int(lead_status), f"{lead.dRel:.3f}", f"{lead.vRel:.3f}", f"{lead.vLead:.3f}",
          f"{lead.aLeadK:.3f}", f"{lead.modelProb:.3f}", int(bool(lead.radar)),
          int(cs_ok), int(ctrl_ok), int(sds_ok), int(radar_ok),
        ])
        csv_file.flush()

      if now - last_print >= 1.0:
        last_print = now
        alive = "".join("+" if ok else "-" for ok in (cs_ok, ctrl_ok, sds_ok, radar_ok))
        line = (f"[{time.strftime('%H:%M:%S')}] alive={alive} engaged={engaged!s:5} "
                f"v={v_ego:5.2f} m/s aEgo={a_ego:+.2f}  long={state}  "
                f"accel: hccc={hccc:+.2f} manual={manual:+.2f} cmd={cmd_accel:+.2f} out={out_accel:+.2f}")
        if tailer is not None:
          for role, health in sorted(tailer.poll().items()):
            if role == "ego":
              continue
            line += f"  {role}: age={health.age_ms():6.1f}ms seq={health.last_seq}"
            if health.rejected:
              line += f" rej={health.rejected}({health.last_reason})"
        print(line, flush=True)
  finally:
    if csv_file is not None:
      csv_file.close()


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
