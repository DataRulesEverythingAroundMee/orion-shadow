import struct
from dataclasses import dataclass

# --- Constants & Enumerations ---

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
    CAMERA_SWITCH = 0x60
    CAMERA_STATE = 0x61
    CAMERAS = 0x63
    VIDEO_OPTIONS = 0x70
    TRACK_OPTIONS = 0x71
    SENSOR_DATA = 0xD0
    GPS_DATA = 0xD1
    EXT_HEADING_DATA = 0xD2

@dataclass
class OrionPacket:
    packet_id: int
    data: bytes

    def encode(self) -> bytes:
        length = len(self.data)
        payload = struct.pack(">BBBB", ORION_SYNC0, ORION_SYNC1, self.packet_id, length) + self.data
        
        # Fletcher-16 Checksum (modified mod 251)
        a, b = 1, 1
        for byte in payload:
            a = (a + byte) % 251
            b = (b + a) % 251
            
        checksum = (b << 8) | a
        return payload + struct.pack(">H", checksum)
