from types import SimpleNamespace

from openpilot.selfdrive.controls.v2vpublisher import publish_car_state


def test_publish_car_state_emits_accel_and_speed():
  captured = {}

  class FakePublisher:
    def publish(self, a_lead, v_lead):
      captured["a_lead"] = a_lead
      captured["v_lead"] = v_lead
      return SimpleNamespace(seq=5, a_lead=a_lead, v_lead=v_lead)

  packet = publish_car_state(FakePublisher(), SimpleNamespace(aEgo=1.25, vEgo=9.5))

  assert captured == {"a_lead": 1.25, "v_lead": 9.5}
  assert packet.seq == 5
