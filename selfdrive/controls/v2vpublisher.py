#!/usr/bin/env python3
from __future__ import annotations

import cereal.messaging as messaging

from openpilot.common.realtime import Priority, Ratekeeper, config_realtime_process
from openpilot.selfdrive.controls.lib.hcc_v2v import ROLE_LEAD, V2VPublisher, load_v2v_config


def main() -> None:
  config_realtime_process(2, Priority.CTRL_LOW)

  config = load_v2v_config(ROLE_LEAD, default_enabled=True, default_device_id="hcc-lead")
  publisher = V2VPublisher(config)
  publisher.start()

  sm = messaging.SubMaster(['carState'], poll='carState')
  rk = Ratekeeper(config.send_hz, print_delay_threshold=None)

  while True:
    sm.update(0)
    if sm.updated['carState'] and sm.valid['carState']:
      car_state = sm['carState']
      publisher.publish(car_state.aEgo, car_state.vEgo)
    rk.keep_time()


if __name__ == "__main__":
  main()
