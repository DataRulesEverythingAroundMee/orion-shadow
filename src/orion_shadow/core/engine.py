import struct
from orion_shadow.core.protocol import OrionPacket, ORION_SYNC0, ORION_SYNC1

class ProtocolEngine:
    def __init__(self):
        self.packet_id_map = {
            0x00: "ORION_PKT_INITIALIZE",
            0x01: "ORION_PKT_CMD",
            0x0A: "ORION_PKT_POSITIONS",
            0x4A: "ORION_PKT_BOARD_HEARTBEAT",
            0x5F: "ORION_PKT_ADVANCED_ENCODING",
            0x63: "ORION_PKT_CAMERAS",
            0x7A: "ORION_PKT_SIONYX_SETTINGS",
            0x7B: "ORION_PKT_LYNRED_SETTINGS",
            0xD0: "ORION_PKT_SENSOR_DATA",
            0xFD: "ORION_PKT_UNIFIED_CAM_ERR_SETTINGS",
        }

    def parse(self, raw_data: bytes) -> OrionPacket:
        if len(raw_data) < 6:
            raise ValueError("Packet too short")
            
        sync0, sync1, p_id, length = struct.unpack(">BBBB", raw_data[:4])
        if sync0 != ORION_SYNC0 or sync1 != ORION_SYNC1:
            raise ValueError("Invalid Sync bytes")
            
        data = raw_data[4:4+length]
        return OrionPacket(p_id, data)
