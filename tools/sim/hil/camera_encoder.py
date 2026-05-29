"""PC-side encoder that pushes synthesized RGB frames over ZMQ to a device.

One encoder per (device, stream). Mirrors the codec setup used in
tools/camerastream/compressed_vipc.py so the decoder side can be a near-copy.
The first message after each (re)connect carries SPS/PPS extradata with
FLAG_KEYFRAME — the decoder feeds that as the codec header before any frame
packets. The --raw_yuv fallback ships NV12 directly for bring-up.
"""
from __future__ import annotations

import argparse
import signal
import time

import numpy as np
import zmq

from openpilot.tools.sim.hil import proto
from openpilot.tools.sim.lib.common import H, W


# H.264 baseline at 20fps, gop=20. Combined road+wide is ~16-32 Mbit/s — well
# under USB-RNDIS budget. zerolatency + baseline avoids B-frames so the
# decoder never reorders and frame_id stays monotonic on the device.
_CODEC_NAME = "h264"
_CODEC_OPTIONS = {
  "tune": "zerolatency",
  "preset": "ultrafast",
  "profile": "baseline",
}
_GOP = 20
_BITRATE = 8_000_000  # ~8 Mbit/s per stream; combined road+wide stays under USB-RNDIS headroom

# ZMQ socket high-water mark: caps the send queue to avoid unbounded memory
# growth when the device decoder can't keep up. 4 frames ≈ 200 ms at 20 fps.
_ZMQ_HWM = 4


class RemoteCameraEncoder:
  def __init__(self, device_ip: str, stream_id: proto.StreamId, raw_yuv: bool = False, width: int = W, height: int = H, fps: int = 20):
    self.stream_id = stream_id
    self.raw_yuv = raw_yuv
    self.width = width
    self.height = height
    self.frame_id = 0
    self._ctx = zmq.Context.instance()
    self._sock = self._ctx.socket(zmq.PUSH)
    self._sock.setsockopt(zmq.SNDHWM, _ZMQ_HWM)
    self._sock.setsockopt(zmq.LINGER, 0)
    self._sock.connect(f"tcp://{device_ip}:{proto.PORT_BY_STREAM[stream_id]}")

    self._codec = None
    self._extradata_sent = False
    if not raw_yuv:
      import av
      self._codec = av.CodecContext.create(_CODEC_NAME, "w")
      self._codec.width = width
      self._codec.height = height
      self._codec.pix_fmt = "yuv420p"
      self._codec.framerate = fps
      self._codec.time_base = f"1/{fps}"
      self._codec.gop_size = _GOP
      self._codec.bit_rate = _BITRATE
      self._codec.options = _CODEC_OPTIONS

  def send_rgb(self, rgb: np.ndarray) -> None:
    """Encode and ship one frame. rgb is (H, W, 3) uint8."""
    assert rgb.shape == (self.height, self.width, 3), f"unexpected shape {rgb.shape}"
    assert rgb.dtype == np.uint8

    ts_ns = time.monotonic_ns()
    if self.raw_yuv:
      self._send_raw_nv12(rgb, ts_ns)
    else:
      self._send_encoded(rgb, ts_ns)
    self.frame_id += 1

  def _send_packets(self, packets, ts_ns: int) -> None:
    """Ship a list of av.Packet objects over ZMQ with proto framing."""
    for pkt in packets:
      payload = bytes(pkt)
      flags = proto.FrameFlags.KEYFRAME if pkt.is_keyframe else proto.FrameFlags.NONE
      header = proto.FrameHeader(self.stream_id, self.frame_id, ts_ns, flags, len(payload))
      self._sock.send(proto.encode_message(header, payload))

  def _send_encoded(self, rgb: np.ndarray, ts_ns: int) -> None:
    import av
    frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
    frame.pts = self.frame_id
    packets = list(self._codec.encode(frame))

    # extradata is populated by the encoder lazily, after the first encode().
    # Decoder must see it before any frame packet; emit it as a special
    # keyframe-flagged message the very first time we have it.
    if not self._extradata_sent and self._codec.extradata:
      ed = bytes(self._codec.extradata)
      header = proto.FrameHeader(self.stream_id, self.frame_id, ts_ns, proto.FrameFlags.EXTRADATA | proto.FrameFlags.KEYFRAME, len(ed))
      self._sock.send(proto.encode_message(header, ed))
      self._extradata_sent = True

    self._send_packets(packets, ts_ns)

  def _send_raw_nv12(self, rgb: np.ndarray, ts_ns: int) -> None:
    nv12 = _rgb_to_nv12(rgb)
    header = proto.FrameHeader(self.stream_id, self.frame_id, ts_ns, proto.FrameFlags.RAW_YUV | proto.FrameFlags.KEYFRAME, nv12.nbytes)
    self._sock.send(proto.encode_message(header, nv12.tobytes()))

  def close(self) -> None:
    if self._codec is not None:
      try:
        self._send_packets(self._codec.encode(None), time.monotonic_ns())
      except Exception:
        pass
    self._sock.close(linger=0)


def _rgb_to_nv12(rgb: np.ndarray) -> np.ndarray:
  """Pure-numpy RGB → NV12 fallback. Slow; only used by --raw_yuv bring-up."""
  h, w, _ = rgb.shape
  r = rgb[..., 0].astype(np.int32)
  g = rgb[..., 1].astype(np.int32)
  b = rgb[..., 2].astype(np.int32)
  # BT.601 fixed-point integer coefficients (SDTV full-range)
  y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16
  u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128
  v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128
  y = np.clip(y, 0, 255).astype(np.uint8)
  uv = np.empty((h // 2, w), dtype=np.uint8)
  uv[:, 0::2] = np.clip(u[::2, ::2], 0, 255).astype(np.uint8)
  uv[:, 1::2] = np.clip(v[::2, ::2], 0, 255).astype(np.uint8)
  return np.concatenate([y.ravel(), uv.ravel()])


def _testbars(frame_id: int, w: int, h: int) -> np.ndarray:
  """Cheap moving-color test pattern for bring-up."""
  rgb = np.zeros((h, w, 3), dtype=np.uint8)
  bar_w = w // 8
  shift = (frame_id * 4) % w
  colors = [(255, 255, 255), (255, 255, 0), (0, 255, 255), (0, 255, 0), (255, 0, 255), (255, 0, 0), (0, 0, 255), (0, 0, 0)]
  for i, c in enumerate(colors):
    x0 = (i * bar_w + shift) % w
    x1 = min(x0 + bar_w, w)
    rgb[:, x0:x1] = c
  return rgb


def main() -> int:
  parser = argparse.ArgumentParser(description="Push test camera frames to a device's remote_sensor_bridge.")
  parser.add_argument("--device", required=True, help="Device IP")
  parser.add_argument("--stream", default="road", choices=["road", "wide_road"])
  parser.add_argument("--raw_yuv", action="store_true")
  parser.add_argument("--fps", type=int, default=20)
  parser.add_argument("--seconds", type=int, default=30)
  parser.add_argument("--pattern", default="testbars", choices=["testbars"])
  args = parser.parse_args()

  stream_id = proto.StreamId.ROAD if args.stream == "road" else proto.StreamId.WIDE_ROAD
  enc = RemoteCameraEncoder(args.device, stream_id, raw_yuv=args.raw_yuv)
  signal.signal(signal.SIGINT, lambda *_: enc.close() or exit(0))

  period = 1.0 / args.fps
  total = args.seconds * args.fps
  next_t = time.monotonic()
  for i in range(total):
    rgb = _testbars(i, W, H)
    enc.send_rgb(rgb)
    next_t += period
    sleep = next_t - time.monotonic()
    if sleep > 0:
      time.sleep(sleep)
  enc.close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
