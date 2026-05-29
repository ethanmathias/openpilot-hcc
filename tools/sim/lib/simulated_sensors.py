import time

from cereal import log
import cereal.messaging as messaging

from openpilot.common.realtime import DT_DMON
from openpilot.tools.sim.lib.camerad import Camerad

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from openpilot.tools.sim.lib.common import World, SimulatorState

# Number of IMU and GPS messages published per tick (matches hardware cadence).
_IMU_MSGS_PER_TICK = 5
_GPS_MSGS_PER_TICK = 10

# Sensor IDs and type code matching the C3 hardware (see locationd).
_ACCEL_SENSOR_ID = 4
_GYRO_SENSOR_ID = 5
_IMU_SENSOR_TYPE = 0x10

# Peripheral state publish interval (seconds).
_PERIPHERAL_INTERVAL_SECS = 0.25

# Simulated peripheral electrical values.
_PERIPHERAL_VOLTAGE_MV = 12000
_PERIPHERAL_CURRENT_MA = 5678
_PERIPHERAL_FAN_RPM = 1000


class SimulatedSensors:
  """Simulates the C3 sensors (acc, gyro, gps, peripherals, dm state, cameras) to OpenPilot"""

  def __init__(self, dual_camera=False):
    self.pm = messaging.PubMaster(['accelerometer', 'gyroscope', 'gpsLocationExternal', 'driverStateV2', 'driverMonitoringState', 'peripheralState'])
    self.camerad = Camerad(dual_camera=dual_camera)
    self.last_perp_update = 0
    self.last_dmon_update = 0

  def _send_imu_message(self, service: str, sensor_id: int, field: str, values: list[float]):
    """Build and publish a single IMU message (shared by accel and gyro paths)."""
    dat = messaging.new_message(service, valid=True)
    entry = getattr(dat, service)
    entry.sensor = sensor_id
    entry.type = _IMU_SENSOR_TYPE
    entry.timestamp = dat.logMonoTime
    entry.init(field)
    getattr(entry, field).v = values
    self.pm.send(service, dat)

  def send_imu_message(self, simulator_state: 'SimulatorState'):
    imu = simulator_state.imu
    for _ in range(_IMU_MSGS_PER_TICK):
      self._send_imu_message('accelerometer', _ACCEL_SENSOR_ID, 'acceleration',
                             [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z])
      self._send_imu_message('gyroscope', _GYRO_SENSOR_ID, 'gyroUncalibrated',
                             [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z])

  def send_gps_message(self, simulator_state: 'SimulatorState'):
    if not simulator_state.valid:
      return

    # transform from vel to NED
    velNED = [
      -simulator_state.velocity.y,
      simulator_state.velocity.x,
      simulator_state.velocity.z,
    ]

    for _ in range(_GPS_MSGS_PER_TICK):
      dat = messaging.new_message('gpsLocationExternal', valid=True)
      dat.gpsLocationExternal = {
        "unixTimestampMillis": int(time.time() * 1000),  # noqa: TID251
        "flags": 1,  # valid fix
        "horizontalAccuracy": 1.0,
        "verticalAccuracy": 1.0,
        "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1,
        "vNED": velNED,
        "bearingDeg": simulator_state.imu.bearing,
        "latitude": simulator_state.gps.latitude,
        "longitude": simulator_state.gps.longitude,
        "altitude": simulator_state.gps.altitude,
        "speed": simulator_state.speed,
        "source": log.GpsLocationData.SensorSource.ublox,
      }

      self.pm.send('gpsLocationExternal', dat)

  def send_peripheral_state(self):
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': _PERIPHERAL_VOLTAGE_MV,
      'current': _PERIPHERAL_CURRENT_MA,
      'fanSpeedRpm': _PERIPHERAL_FAN_RPM
    }
    self.pm.send('peripheralState', dat)

  def send_fake_driver_monitoring(self):
    # dmonitoringmodeld output
    dat = messaging.new_message('driverStateV2')
    dat.driverStateV2.leftDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.leftDriverData.faceProb = 1.0
    dat.driverStateV2.rightDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.rightDriverData.faceProb = 1.0
    self.pm.send('driverStateV2', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState', valid=True)
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    self.pm.send('driverMonitoringState', dat)

  def send_camera_images(self, world: 'World'):
    world.image_lock.acquire()
    yuv = self.camerad.rgb_to_yuv(world.road_image)
    self.camerad.cam_send_yuv_road(yuv)

    if world.dual_camera:
      yuv = self.camerad.rgb_to_yuv(world.wide_road_image)
      self.camerad.cam_send_yuv_wide_road(yuv)

  def update(self, simulator_state: 'SimulatorState', world: 'World'):
    now = time.monotonic()
    self.send_imu_message(simulator_state)
    self.send_gps_message(simulator_state)

    if (now - self.last_dmon_update) > DT_DMON/2:
      self.send_fake_driver_monitoring()
      self.last_dmon_update = now

    if (now - self.last_perp_update) > _PERIPHERAL_INTERVAL_SECS:
      self.send_peripheral_state()
      self.last_perp_update = now
