"""UDP wire formats shared with the Android app (android/.../StreamService.kt and LivePath.kt). Little-endian.

Phone -> laptop, IMU samples:
  header  : magic b"IMU1" (4 bytes), uint32 sequence number, uint16 record count
  records : count x [uint8 type, int64 t_ns, float32 x, float32 y, float32 z]
Types: 1 = accelerometer (m/s^2, gravity included), 2 = gyroscope (rad/s, uncalibrated).
t_ns is the Android sensor timestamp (nanoseconds since boot).

Laptop -> phone, the live path (sent back to the address the IMU1 packets come from, so the phone can draw what
the laptop draws):
  header  : magic b"PTH1", uint16 generation, uint8 state, uint8 flags, uint32 steps, float32 distance (m),
            float32 walking speed (m/s), float32 shoe motion while calibrating (rad/s; -1 = not measured yet),
            float32 current x, float32 current y, uint32 points in the path, uint32 first point in this packet,
            uint16 point count
  points  : count x [float32 x, float32 y]
The path is in the laptop's view frame (start at the origin, first 3 m pointing up), one point per 0.1 m walked.
A packet carries any run of points, so a lost packet is repaired by a later one. A new generation means the path
was replaced (recalibrated, origin moved, or turned to point up): the phone drops the points it has.
State: 0 = calibrating, 1 = tracking. Flags: bit 0 = foot flat on the ground.
"""
import struct

MAGIC = b"IMU1"
HEADER = struct.Struct("<4sIH")
RECORD = struct.Struct("<Bqfff")
ACC, GYRO = 1, 2
MAX_RECORDS = (1400 - HEADER.size) // RECORD.size

PATH_MAGIC = b"PTH1"
PATH_HEADER = struct.Struct("<4sHBBIfffffIIH")
POINT = struct.Struct("<ff")
MAX_POINTS = (1400 - PATH_HEADER.size) // POINT.size
CALIBRATING_STATE, TRACKING_STATE = 0, 1
FLAG_STILL = 1


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


def encode_path(gen, state, flags, steps, distance, speed, motion, cur, n_total, start, points):
    """points: the path's points start, start + 1, ... as (x, y); at most MAX_POINTS."""
    points = list(points)
    assert len(points) <= MAX_POINTS
    head = PATH_HEADER.pack(PATH_MAGIC, gen & 0xFFFF, state, flags, steps, distance, speed, motion,
                            cur[0], cur[1], n_total, start, len(points))
    return head + b"".join(POINT.pack(x, y) for x, y in points)


def decode_path(data):
    """Return a dict of the header fields plus "points" [(x, y), ...]; raise ValueError on a malformed packet."""
    if len(data) < PATH_HEADER.size:
        raise ValueError("short packet")
    f = PATH_HEADER.unpack_from(data, 0)
    if f[0] != PATH_MAGIC or len(data) != PATH_HEADER.size + f[-1] * POINT.size or f[11] + f[12] > f[10]:
        raise ValueError("bad packet")
    keys = ("gen", "state", "flags", "steps", "distance", "speed", "motion", "x", "y", "n_total", "start", "count")
    out = dict(zip(keys, f[1:]))
    out["points"] = [POINT.unpack_from(data, PATH_HEADER.size + i * POINT.size) for i in range(out["count"])]
    return out
