#!/usr/bin/env python3
"""Device-side: subscribe to PC-pushed camera frames over ZMQ, decode, and
republish locally via VisionIPC for modeld to consume.

Runs only when HIL_MODE=1; gated in system/manager/process_config.py. Camerad,
pandad, sensord are blocked via the BLOCK env var in launch_device.sh.

Decoder loop is a near-copy of tools/camerastream/compressed_vipc.py; the
wire format is defined in tools/sim/hil/proto.py.
"""
from __future__ import annotations

import multiprocessing
import os
import signal
import sys
import time

import numpy as np
import zmq

from msgq.visionipc import VisionIpcServer, VisionStreamType

from openpilot.tools.sim.hil import proto
from openpilot.tools.sim.lib.common import H, W


_VST_BY_STREAM = {
  proto.StreamId.ROAD: VisionStreamType.VISION_STREAM_ROAD,
  proto.StreamId.WIDE_ROAD: VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def _stream_worker(stream_id: proto.StreamId, vipc_server: VisionIpcServer, width: int, height: int) -> None:
  """One worker per stream. Bind PULL, decode, republish via VisionIPC."""
  import av

  port = proto.PORT_BY_STREAM[stream_id]
  ctx = zmq.Context.instance()
  sock = ctx.socket(zmq.PULL)
  sock.setsockopt(zmq.RCVHWM, 4)
  sock.setsockopt(zmq.LINGER, 0)
  sock.bind(f"tcp://0.0.0.0:{port}")

  vst = _VST_BY_STREAM[stream_id]
  codec = av.CodecContext.create("h264", "r")
  seen_iframe = False
  cnt = 0

  while True:
    msg = sock.recv()
    header, payload = proto.decode_message(msg)

    # Raw NV12 fast-path: bring-up only.
    if header.flags & proto.FrameFlags.RAW_YUV:
      ts = int(time.monotonic() * 1e9)
      vipc_server.send(vst, np.frombuffer(payload, dtype=np.uint8).data, cnt, ts, ts)
      cnt += 1
      continue

    # Codec extradata: feed once as the codec header, then continue.
    if header.flags & proto.FrameFlags.EXTRADATA:
      try:
        codec.decode(av.packet.Packet(payload))
      except av.AVError:
        pass
      seen_iframe = True
      continue

    # Wait for the first IDR before forwarding anything.
    if not seen_iframe:
      if not (header.flags & proto.FrameFlags.KEYFRAME):
        continue
      seen_iframe = True

    try:
      frames = codec.decode(av.packet.Packet(payload))
    except av.AVError:
      continue
    if not frames:
      continue
    frame = frames[-1]
    img_yuv = frame.to_ndarray(format="yuv420p").flatten()
    # YUV420 planar → NV12 interleaved UV (VisionIPC consumers expect NV12)
    uv_offset = height * width
    y = img_yuv[:uv_offset]
    uv = img_yuv[uv_offset:].reshape(2, -1).ravel("F")
    nv12 = np.concatenate([y, uv])

    ts = int(time.monotonic() * 1e9)
    vipc_server.send(vst, nv12.data, cnt, header.ts_ns or ts, ts)
    cnt += 1


def main() -> int:
  if os.getenv("HIL_MODE") != "1":
    print("[remote_sensor_bridge] HIL_MODE not set; exiting", file=sys.stderr)
    return 0

  dual_camera = os.getenv("HIL_DUAL_CAMERA", "1") == "1"

  vipc_server = VisionIpcServer("camerad")
  vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, W, H)
  if dual_camera:
    vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H)
  vipc_server.start_listener()

  procs = []
  procs.append(multiprocessing.Process(target=_stream_worker, args=(proto.StreamId.ROAD, vipc_server, W, H), daemon=True))
  if dual_camera:
    procs.append(multiprocessing.Process(target=_stream_worker, args=(proto.StreamId.WIDE_ROAD, vipc_server, W, H), daemon=True))

  for p in procs:
    p.start()

  def _shutdown(*_):
    for p in procs:
      p.terminate()
    sys.exit(0)
  signal.signal(signal.SIGTERM, _shutdown)
  signal.signal(signal.SIGINT, _shutdown)

  for p in procs:
    p.join()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
