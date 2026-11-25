"""Decoder for the WitMotion (WIT) standard serial protocol used by the WTGAHRS3.

Each packet is 11 bytes: 0x55, type, 8 data bytes (little-endian), checksum = sum of the first
10 bytes & 0xFF. Values are converted to SI units here (m/s^2, deg/s, deg, m, m/s).
"""
import struct

GRAVITY = 9.80665
PACKET_SIZE = 11


def _to_degrees(raw):
    # NMEA-style dddmm.mmmmm packed as an integer: degrees = raw // 1e7, minutes = (raw % 1e7) / 1e5
    sign = -1.0 if raw < 0 else 1.0
    raw = abs(raw)
    return sign * (raw // 10_000_000 + (raw % 10_000_000) / 100_000 / 60.0)


def decode(ptype, data):
    """Returns (kind, fields) for a known packet type, else None."""
    if ptype == 0x51:
        ax, ay, az, temp = struct.unpack("<4h", data)
        return "acc", {
            "acc": tuple(v / 32768.0 * 16.0 * GRAVITY for v in (ax, ay, az)),
            "temp": temp / 100.0,
        }
    if ptype == 0x52:
        wx, wy, wz, _ = struct.unpack("<4h", data)
        return "gyro", {"gyro": tuple(v / 32768.0 * 2000.0 for v in (wx, wy, wz))}
    if ptype == 0x53:
        roll, pitch, yaw, _ = struct.unpack("<4h", data)
        return "angle", {"euler": tuple(v / 32768.0 * 180.0 for v in (roll, pitch, yaw))}
    if ptype == 0x57:
        lon, lat = struct.unpack("<2i", data)
        return "position", {"lon": _to_degrees(lon), "lat": _to_degrees(lat)}
    if ptype == 0x58:
        height, heading, speed = struct.unpack("<hhi", data)
        # Ground speed is documented as value / 1000 in km/h
        return "gps", {"alt": height / 10.0, "heading": heading / 100.0, "speed": speed / 1000.0 / 3.6}
    if ptype == 0x59:
        q = struct.unpack("<4h", data)
        return "quat", {"quat": tuple(v / 32768.0 for v in q)}
    if ptype == 0x5A:
        sats, pdop, hdop, vdop = struct.unpack("<4h", data)
        return "dop", {"sats": sats, "pdop": pdop / 100.0, "hdop": hdop / 100.0, "vdop": vdop / 100.0}
    return None


class WitMotionParser:
    def __init__(self):
        self.buf = bytearray()
        self.bad_packets = 0
        self.good_packets = 0

    def feed(self, data):
        """Adds raw bytes and returns the list of decoded (kind, fields) packets."""
        self.buf += data
        out = []
        while len(self.buf) >= PACKET_SIZE:
            if self.buf[0] != 0x55:
                del self.buf[0]
                continue
            pkt = bytes(self.buf[:PACKET_SIZE])
            if sum(pkt[:10]) & 0xFF != pkt[10]:
                self.bad_packets += 1
                del self.buf[0]
                continue
            del self.buf[:PACKET_SIZE]
            self.good_packets += 1
            decoded = decode(pkt[1], pkt[2:10])
            if decoded is not None:
                out.append(decoded)
        return out
