"""IMU/GPS/peripheral/dmon publisher + camera encoder shipping for one device.

The cereal publishes here (accelerometer, gyroscope, gpsLocationExternal,
peripheralState, driverStateV2, driverMonitoringState) go to the prefix's
local msgq; tools/sim/hil/cereal_bridges.py's msgq→zmq subprocess relays them
out over RNDIS to the device, where the device-side bridge republishes them
locally for the on-device consumers.

Cameras don't go through cereal — they're encoded as H.264 (or NV12 if
raw_yuv) by RemoteCameraEncoder and pushed over a dedicated ZMQ socket
straight to the device's remote_sensor_bridge.

The cereal-publishing methods mirror tools/sim/lib/simulated_sensors.py so
behavior matches the existing local-sim path; we don't subclass to avoid
constructing the local Camerad (which would bind a wasteful VisionIpcServer
on the PC and force a pyopencl context).
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from cereal import log
import cereal.messaging as messaging

from openpilot.common.realtime import DT_DMON
from openpilot.tools.sim.hil import proto
from openpilot.tools.sim.hil.camera_encoder import RemoteCameraEncoder

if TYPE_CHECKING:
  from openpilot.tools.sim.lib.common import SimulatorState, World


class RemoteSensors:
  def __init__(self, device_ip: str, dual_camera: bool = True, raw_yuv: bool = False):
    self.dual_camera = dual_camera
    self.pm = messaging.PubMaster(['accelerometer', 'gyroscope', 'gpsLocationExternal', 'driverStateV2', 'driverMonitoringState', 'peripheralState'])
    self.road_encoder = RemoteCameraEncoder(device_ip, proto.StreamId.ROAD, raw_yuv=raw_yuv)
    self.wide_encoder = RemoteCameraEncoder(device_ip, proto.StreamId.WIDE_ROAD, raw_yuv=raw_yuv) if dual_camera else None
    self._last_perp = 0.0
    self._last_dmon = 0.0

  def send_imu(self, s: SimulatorState) -> None:
    for _ in range(5):
      dat = messaging.new_message('accelerometer', valid=True)
      dat.accelerometer.sensor = 4
      dat.accelerometer.type = 0x10
      dat.accelerometer.timestamp = dat.logMonoTime
      dat.accelerometer.init('acceleration')
      dat.accelerometer.acceleration.v = [s.imu.accelerometer.x, s.imu.accelerometer.y, s.imu.accelerometer.z]
      self.pm.send('accelerometer', dat)

      dat = messaging.new_message('gyroscope', valid=True)
      dat.gyroscope.sensor = 5
      dat.gyroscope.type = 0x10
      dat.gyroscope.timestamp = dat.logMonoTime
      dat.gyroscope.init('gyroUncalibrated')
      dat.gyroscope.gyroUncalibrated.v = [s.imu.gyroscope.x, s.imu.gyroscope.y, s.imu.gyroscope.z]
      self.pm.send('gyroscope', dat)

  def send_gps(self, s: SimulatorState) -> None:
    if not s.valid:
      return
    velNED = [-s.velocity.y, s.velocity.x, s.velocity.z]
    for _ in range(10):
      dat = messaging.new_message('gpsLocationExternal', valid=True)
      dat.gpsLocationExternal = {
        "unixTimestampMillis": int(time.monotonic() * 1000),
        "flags": 1,
        "horizontalAccuracy": 1.0,
        "verticalAccuracy": 1.0,
        "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1,
        "vNED": velNED,
        "bearingDeg": s.imu.bearing,
        "latitude": s.gps.latitude,
        "longitude": s.gps.longitude,
        "altitude": s.gps.altitude,
        "speed": s.speed,
        "source": log.GpsLocationData.SensorSource.ublox,
      }
      self.pm.send('gpsLocationExternal', dat)

  def send_peripheral(self) -> None:
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': 12000,
      'current': 5678,
      'fanSpeedRpm': 1000,
    }
    self.pm.send('peripheralState', dat)

  def send_dmon(self) -> None:
    dat = messaging.new_message('driverStateV2')
    dat.driverStateV2.leftDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.leftDriverData.faceProb = 1.0
    dat.driverStateV2.rightDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.rightDriverData.faceProb = 1.0
    self.pm.send('driverStateV2', dat)

    dat = messaging.new_message('driverMonitoringState', valid=True)
    dat.driverMonitoringState = {"faceDetected": True, "isDistracted": False, "awarenessStatus": 1.}
    self.pm.send('driverMonitoringState', dat)

  def send_camera_images(self, world: World) -> None:
    world.image_lock.acquire()
    self.road_encoder.send_rgb(world.road_image)
    if self.wide_encoder is not None and world.dual_camera:
      self.wide_encoder.send_rgb(world.wide_road_image)

  def update(self, s: SimulatorState, world: World) -> None:
    now = time.monotonic()
    self.send_imu(s)
    self.send_gps(s)
    if (now - self._last_dmon) > DT_DMON / 2:
      self.send_dmon()
      self._last_dmon = now
    if (now - self._last_perp) > 0.25:
      self.send_peripheral()
      self._last_perp = now

  def close(self) -> None:
    self.road_encoder.close()
    if self.wide_encoder is not None:
      self.wide_encoder.close()
