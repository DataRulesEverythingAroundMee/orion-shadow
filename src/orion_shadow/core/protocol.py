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
    CLEVIS_VERSION = 0x25
    RESET_SOURCE = 0x26
    BOARD = 0x27
    CROWN_VERSION = 0x28
    PAYLOAD_VERSION = 0x29
    TRACKER_VERSION = 0x2C
    DIAGNOSTICS = 0x41
    FAULTS = 0x42
    TLE_STATUS = 0x47
    TLE_COMMAND = 0x48
    BOARD_HEARTBEAT = 0x4A
    ADVANCED_ENCODING = 0x5F
    CAMERA_SWITCH = 0x60
    CAMERA_STATE = 0x61
    NETWORK_VIDEO = 0x62
    CAMERA_CMD = 0x61  # Backwards compatibility alias for CAMERA_STATE
    CAMERAS = 0x63
    FLIR_SETTINGS = 0x67
    KTNC_SETTINGS = 0x6D
    VIDEO_OPTIONS = 0x70
    TRACK_OPTIONS = 0x71
    GEO_TRACK_STATUS = 0x72
    GEO_TRACK_COMMAND = 0x73
    TRACK_CMD = 0x74
    VIDEORECORD_STATUS = 0x75
    VIDEORECORD_CMD = 0x76
    VIDEORECORD_CLOCK = 0x77
    SIONYX_SETTINGS = 0x7A
    LYNRED_SETTINGS = 0x7B
    AUTOPILOT_DATA = 0x80
    RETRACT_CMD = 0xA0
    RETRACT_STATUS = 0xA1
    PRODUCT = 0xA7
    CROWN_MODE = 0xB0
    DEBUG_STRING = 0xB1
    USER_DATA = 0xB2
    KLV_USER_DATA = 0xB3
    SENSOR_DATA = 0xD0
    GPS_DATA = 0xD1
    EXT_HEADING_DATA = 0xD2
    INS_QUALITY = 0xD3
    GEOLOCATE_TELEMETRY_CORE = 0xD4
    GEOPOINT_CMD = 0xD5
    RANGE_DATA = 0xD6
    PATH = 0xD7
    INS_OPTIONS = 0xD8
    STARE_START = 0xD9
    STARE_ACK = 0xDA
    GEOID_UNDULATION = 0xDC
    NETWORK_SETTINGS = 0xE4
    RETRACT_VERSION = 0xF8
    LENSCTL_VERSION = 0xF9
    SCAN_PLAN = 0xFC
    UNIFIED_CAM_ERR_SETTINGS = 0xFD

class OrionBoard:
    BOARD_NONE = 0
    BOARD_CLEVIS = 1
    BOARD_CROWN = 2
    BOARD_PAYLOAD = 3
    BOARD_LENSCTRL = 4
    BOARD_TRACKER = 5

class OrionTrackCmd:
    TRACK_START_PRIMARY = 0
    TRACK_START_SECONDARY = 1
    TRACK_STOP_ALL = 2
    TRACK_STOP_ALL_BUT_PRIMARY = 3
    TRACK_NUDGE_PRIMARY = 4
    TRACK_RESIZE_PRIMARY = 5
    TRACK_NUDGE_BY_INDEX = 6
    TRACK_RESIZE_BY_INDEX = 7
    TRACK_REMOVE_BY_INDEX = 8
    TRACK_REMOVE_BY_PIXEL = 9

class OrionMode:
    DISABLED = 0x00
    FAULT = 0x01
    RATE = 0x10
    GEO_RATE = 0x11
    FFC_AUTO = 0x20
    FFC = 0x20
    FFC_MANUAL = 0x21
    SCENE = 0x30
    TRACK = 0x31
    NUDGE_TRACK = 0x32
    SECONDARY_TRACK = 0x33
    CALIBRATION = 0x40
    NULL_GYROS = 0x41
    POSITION = 0x50
    POSITION_NO_LIMITS = 0x51
    GEOPOINT = 0x60
    PATH = 0x70
    DOWN = 0x71
    UNKNOWN = 0xFF

import logging

logger = logging.getLogger(__name__)

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
        logger.warning("Packet buffer too short for checksum verification: %d bytes (min 6)", len(packet))
        return False
    a, b = compute_checksum(packet[:-2])
    match = (packet[-2] == a and packet[-1] == b)
    if not match:
        logger.warning(
            "Packet checksum mismatch: expected (0x%02X, 0x%02X), got (0x%02X, 0x%02X) [len=%d]",
            a, b, packet[-2], packet[-1], len(packet)
        )
    elif logger.isEnabledFor(logging.DEBUG):
        logger.debug("Packet checksum verified OK: (0x%02X, 0x%02X)", a, b)
    return match


@dataclass
class OrionPacket:
    packet_id: int
    data: bytes

    def encode(self) -> bytes:
        length = len(self.data)
        payload = struct.pack(">BBBB", ORION_SYNC0, ORION_SYNC1, self.packet_id, length) + self.data
        a, b = compute_checksum(payload)
        encoded = payload + bytes([a, b])
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "Encoded packet 0x%02X (data_len=%d, frame_len=%d)",
                self.packet_id, length, len(encoded)
            )
        return encoded
