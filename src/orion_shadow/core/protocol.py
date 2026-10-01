import struct
from dataclasses import dataclass

# --- Constants & Enumerations ---

# Protocol & SDK Target Versions
TARGET_SDK_VERSION = "3.1.9"
PROTOCOL_VERSION = "1.4.0"

# Default ports (from OrionComm.h)
UDP_OUT_PORT = 8745   # For sending discovery broadcasts
UDP_IN_PORT = 8746    # For receiving discovery responses
TCP_PORT = 8747       # For persistent TCP communication

# Legacy aliases
DEFAULT_UDP_PORT = UDP_OUT_PORT
DEFAULT_TCP_PORT = TCP_PORT
DISCOVERY_PORT = UDP_OUT_PORT

ORION_SYNC0 = 0xD0
ORION_SYNC1 = 0x0D

class OrionPktType:
    INITIALIZE = 0x00
    CMD = 0x01
    UART_CONFIG = 0x02
    LASER_CMD = 0x03
    RESET = 0x04
    LASER_STATES = 0x06
    STARTUP_CMD = 0x07
    POSITIONS = 0x0A
    LIMITS = 0x22
    DIAGNOSTICS = 0x41
    FAULTS = 0x42
    BOARD_HEARTBEAT = 0x4A
    ADVANCED_ENCODING = 0x5F
    CAMERA_SWITCH = 0x60
    CAMERA_STATE = 0x61
    NETWORK_VIDEO = 0x62
    CAMERA_CMD = 0x61  # Backwards compatibility alias for CAMERA_STATE
    CAMERAS = 0x63
    KTNC_SETTINGS = 0x6D
    VIDEO_OPTIONS = 0x70
    TRACK_OPTIONS = 0x71
    SIONYX_SETTINGS = 0x7A
    LYNRED_SETTINGS = 0x7B
    SENSOR_DATA = 0xD0
    GPS_DATA = 0xD1
    EXT_HEADING_DATA = 0xD2
    GEOLOCATE_TELEMETRY_CORE = 0xD4
    GEOPOINT_CMD = 0xD5
    UNIFIED_CAM_ERR_SETTINGS = 0xFD

class OrionMode:
    DISABLED = 0x00
    FAULT = 0x01
    RATE = 0x10
    GEO_RATE = 0x11
    SCENE = 0x30
    TRACK = 0x31
    CALIBRATION = 0x40
    NULL_GYROS = 0x41
    POSITION = 0x50
    POSITION_NO_LIMITS = 0x51
    GEOPOINT = 0x60
    PATH = 0x70
    DOWN = 0x71

def compute_checksum(data: bytes) -> tuple:
    """Compute Fletcher's checksum (mod 251) matching Orion SDK TrilliumPacket.c."""
    if len(data) == 0:
        return (1, 1)

    a = (data[0] + 1) % 251
    b = a
    for byte in data[1:]:
        a = (a + byte) % 251
        b = (b + a) % 251

    return (a, b)


def verify_checksum(packet: bytes) -> bool:
    """Verify packet checksum."""
    if len(packet) < 6:
        return False
    a, b = compute_checksum(packet[:-2])
    return packet[-2] == a and packet[-1] == b


@dataclass
class OrionPacket:
    packet_id: int
    data: bytes

    def encode(self) -> bytes:
        length = len(self.data)
        payload = struct.pack(">BBBB", ORION_SYNC0, ORION_SYNC1, self.packet_id, length) + self.data
        a, b = compute_checksum(payload)
        return payload + bytes([a, b])
