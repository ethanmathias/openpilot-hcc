#!/usr/bin/env python3
from __future__ import annotations

import cereal.messaging as messaging

from openpilot.common.realtime import Priority, Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.hcc_v2v import ROLE_LEAD, V2VPublisher, load_v2v_config


def publish_car_state(publisher: V2VPublisher, car_state):
  return publisher.publish(car_state.aEgo, car_state.vEgo)


def main() -> None:
  config_realtime_process(2, Priority.CTRL_LOW)

  config = load_v2v_config(ROLE_LEAD, default_enabled=True, default_device_id="hcc-lead")
  publisher = V2VPublisher(config)
  publisher.start()
  cloudlog.info("v2vpublisher started device_id=%s relay=%s:%s", config.device_id, config.relay_host, config.relay_port)

  sm = messaging.SubMaster(['carState'], poll='carState')
  rk = Ratekeeper(config.send_hz, print_delay_threshold=None)

  while True:
    sm.update(0)
    if sm.updated['carState'] and sm.valid['carState']:
      car_state = sm['carState']
      packet = publish_car_state(publisher, car_state)
      if packet is None:
        cloudlog.warning("v2vpublisher publish failed device_id=%s", config.device_id)
      elif packet.seq % 50 == 0:
        cloudlog.debug("v2vpublisher seq=%d a_lead=%.3f v_lead=%.3f", packet.seq, packet.a_lead, packet.v_lead)
    rk.keep_time()


if __name__ == "__main__":
  main()
