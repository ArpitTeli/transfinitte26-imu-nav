"""UDP wire format shared with the Android app (android/.../StreamService.kt).

Packet (little-endian):
  header  : magic b"IMU1" (4 bytes), uint32 sequence number, uint16 record count
  records : count x [uint8 type, int64 t_ns, float32 x, float32 y, float32 z]
Types: 1 = accelerometer (m/s^2, gravity included), 2 = gyroscope (rad/s, uncalibrated).
t_ns is the Android sensor timestamp (nanoseconds since boot).
"""
import struct

MAGIC = b"IMU1"
HEADER = struct.Struct("<4sIH")
RECORD = struct.Struct("<Bqfff")
ACC, GYRO = 1, 2
MAX_RECORDS = (1400 - HEADER.size) // RECORD.size


def encode(seq, records):
    """records: iterable of (type, t_ns, x, y, z)."""
    records = list(records)
    return HEADER.pack(MAGIC, seq & 0xFFFFFFFF, len(records)) + b"".join(RECORD.pack(*r) for r in records)


def decode(data):
    """Return (seq, [(type, t_ns, x, y, z), ...]); raise ValueError on a malformed packet."""
    if len(data) < HEADER.size:
        raise ValueError("short packet")
    magic, seq, n = HEADER.unpack_from(data, 0)
    if magic != MAGIC or len(data) != HEADER.size + n * RECORD.size:
        raise ValueError("bad packet")
    return seq, [RECORD.unpack_from(data, HEADER.size + i * RECORD.size) for i in range(n)]
