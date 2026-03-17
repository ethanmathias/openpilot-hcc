from types import SimpleNamespace

from openpilot.selfdrive.car import car_specific
from openpilot.selfdrive.car.car_specific import CarSpecificEvents, GearShifter
from openpilot.selfdrive.selfdrived.events import ET


def _cp(enable_hccc):
  return SimpleNamespace(
    brand='honda',
    carFingerprint='test_hccc_platform',
    pcmCruise=False,
    openpilotLongitudinalControl=True,
    minEnableSpeed=0.0,
    enableHCCC=enable_hccc,
  )


def _cs(gas_pressed):
  return SimpleNamespace(
    doorOpen=False,
    seatbeltUnlatched=False,
    gearShifter=GearShifter.drive,
    cruiseState=SimpleNamespace(available=True, nonAdaptive=False, enabled=False, standstill=False),
    espDisabled=False,
    espActive=False,
    stockFcw=False,
    stockAeb=False,
    stockLkas=False,
    vEgo=10.0,
    brakeHoldActive=False,
    parkingBrake=False,
    accFaulted=False,
    steeringPressed=False,
    steeringDisengage=False,
    brakePressed=False,
    standstill=False,
    gasPressed=gas_pressed,
    vehicleSensorsInvalid=False,
    invalidLkasSetting=False,
    lowSpeedAlert=False,
    buttonEnable=False,
    buttonEvents=[],
    steerFaultTemporary=False,
    steerFaultPermanent=False,
    blockPcmEnable=False,
  )


def test_gas_pressed_override_disabled_when_cooperative_longitudinal_enabled(monkeypatch):
  monkeypatch.setitem(car_specific.interfaces, 'test_hccc_platform', SimpleNamespace(DRIVABLE_GEARS={GearShifter.drive}))

  events_disabled = CarSpecificEvents(_cp(enable_hccc=False)).create_common_events(_cs(True), _cs(False))
  assert events_disabled.contains(ET.OVERRIDE_LONGITUDINAL)

  events_enabled = CarSpecificEvents(_cp(enable_hccc=True)).create_common_events(_cs(True), _cs(False))
  assert not events_enabled.contains(ET.OVERRIDE_LONGITUDINAL)
