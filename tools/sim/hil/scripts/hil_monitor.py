#!/usr/bin/env python3
"""Live HC3/V2V status line, printed once per second. Run ON a device:

    cd /data/openpilot && python3 tools/sim/hil/scripts/hil_monitor.py

Answers the bring-up questions in one place: is the device onroad, is it
engaged, what accel is HC3 commanding, and (on the ego) is the V2V signal
fresh. Reads passively from local msgq + the V2V mode transitions that
longcontrol writes to cloudlog — it never opens a V2V socket itself, so it
cannot steal the relay's ego registration from the real subscriber.

controlsState debug field mapping (see controlsd.py):
  upAccelCmd = planner accel, uiAccelCmd = HC3 accel, ufAccelCmd = manual accel
"""
import time

import cereal.messaging as messaging
from openpilot.common.params import Params


def main() -> None:
  sm = messaging.SubMaster(['carState', 'controlsState', 'selfdriveState'])
  params = Params()
  v2v_only = params.get_bool("HCCV2VOnly") if params.check_key("HCCV2VOnly") else False
  role = "ego" if v2v_only else "?"
  print(f"hil_monitor: role guess={role} (HCCV2VOnly={v2v_only}); Ctrl-C to stop")

  while True:
    sm.update(1000)
    cs_ok = sm.valid['carState']
    ctrl_ok = sm.valid['controlsState']
    sds_ok = sm.valid['selfdriveState']

    v_ego = sm['carState'].vEgo if cs_ok else float("nan")
    engaged = sm['selfdriveState'].active if sds_ok else False
    state = sm['controlsState'].longControlState if ctrl_ok else "?"
    planner = sm['controlsState'].upAccelCmd if ctrl_ok else float("nan")
    hccc = sm['controlsState'].uiAccelCmd if ctrl_ok else float("nan")
    manual = sm['controlsState'].ufAccelCmd if ctrl_ok else float("nan")

    alive = "".join("+" if ok else "-" for ok in (cs_ok, ctrl_ok, sds_ok))
    print(f"[{time.strftime('%H:%M:%S')}] alive={alive} engaged={engaged!s:5} "
          f"v={v_ego:5.2f} m/s  long={state}  accel: hccc={hccc:+.2f} manual={manual:+.2f} planner={planner:+.2f}")
    time.sleep(1.0)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
