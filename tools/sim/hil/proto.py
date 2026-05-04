"""Wire format for the PC-to-device camera stream.

One ZMQ message per frame: fixed-size header followed by raw payload bytes
(H.264 NAL units in Annex-B form, or raw NV12 when FLAG_RAW_YUV is set).

The first message after each (re)connect carries SPS/PPS extradata with
FLAG_KEYFRAME set; the device-side decoder feeds it as the codec header
before any frame packets, mirroring tools/camerastream/compressed_vipc.py.
"""
from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct


class StreamId(IntEnum):
  ROAD = 0
  WIDE_ROAD = 1


class FrameFlags(IntFlag):
  NONE = 0
  KEYFRAME = 1
  RAW_YUV = 2     # payload is NV12, no codec; bring-up fallback only
  EXTRADATA = 4   # payload is the codec extradata (SPS/PPS), to be fed as the codec header


# Per-stream ZMQ ports. PC binds PUSH, device connects PULL — same direction
# as tools/camerastream/compressed_vipc.py so device routing only needs its
# own RNDIS peer.
PORT_BY_STREAM = {
  StreamId.ROAD: 19100,
  StreamId.WIDE_ROAD: 19101,
}

# stream_id (u8), frame_id (u32), ts_ns (u64), flags (u8), payload_len (u32)
_HEADER_FMT = "<BIQBI"
HEADER_SIZE = struct.calcsize(_HEADER_FMT)


@dataclass(frozen=True)
class FrameHeader:
  stream_id: StreamId
  frame_id: int
  ts_ns: int
  flags: FrameFlags
  payload_len: int

  def pack(self) -> bytes:
    return struct.pack(_HEADER_FMT, int(self.stream_id), self.frame_id, self.ts_ns, int(self.flags), self.payload_len)

  @classmethod
  def unpack(cls, buf: bytes) -> "FrameHeader":
    stream_id, frame_id, ts_ns, flags, payload_len = struct.unpack(_HEADER_FMT, buf[:HEADER_SIZE])
    return cls(StreamId(stream_id), frame_id, ts_ns, FrameFlags(flags), payload_len)


def encode_message(header: FrameHeader, payload: bytes) -> bytes:
  return header.pack() + payload


def decode_message(msg: bytes) -> tuple[FrameHeader, bytes]:
  header = FrameHeader.unpack(msg)
  payload = msg[HEADER_SIZE:HEADER_SIZE + header.payload_len]
  return header, payload
