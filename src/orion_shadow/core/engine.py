import struct
import logging
from orion_shadow.core.protocol import OrionPacket, ORION_SYNC0, ORION_SYNC1

logger = logging.getLogger(__name__)

class ProtocolEngine:
    def __init__(self):
        self.packet_id_map = {
            0x00: "ORION_PKT_INITIALIZE",
            0x01: "ORION_PKT_CMD",
            0x02: "ORION_PKT_UART_CONFIG",
            0x03: "ORION_PKT_LASER_CMD",
            0x04: "ORION_PKT_RESET",
            0x06: "ORION_PKT_LASER_STATES",
            0x07: "ORION_PKT_STARTUP_CMD",
            0x0A: "ORION_PKT_POSITIONS",
            0x22: "ORION_PKT_LIMITS",
            0x25: "ORION_PKT_CLEVIS_VERSION",
            0x26: "ORION_PKT_RESET_SOURCE",
            0x27: "ORION_PKT_BOARD",
            0x28: "ORION_PKT_CROWN_VERSION",
            0x29: "ORION_PKT_PAYLOAD_VERSION",
            0x2C: "ORION_PKT_TRACKER_VERSION",
            0x41: "ORION_PKT_DIAGNOSTICS",
            0x42: "ORION_PKT_FAULTS",
            0x47: "ORION_PKT_TLE_STATUS",
            0x48: "ORION_PKT_TLE_COMMAND",
            0x4A: "ORION_PKT_BOARD_HEARTBEAT",
            0x5F: "ORION_PKT_ADVANCED_ENCODING",
            0x60: "ORION_PKT_CAMERA_SWITCH",
            0x61: "ORION_PKT_CAMERA_STATE",
            0x62: "ORION_PKT_NETWORK_VIDEO",
            0x63: "ORION_PKT_CAMERAS",
            0x67: "ORION_PKT_FLIR_SETTINGS",
            0x6D: "ORION_PKT_KTNC_SETTINGS",
            0x70: "ORION_PKT_VIDEO_OPTIONS",
            0x71: "ORION_PKT_TRACK_OPTIONS",
            0x72: "ORION_PKT_GEO_TRACK_STATUS",
            0x73: "ORION_PKT_GEO_TRACK_COMMAND",
            0x74: "ORION_PKT_TRACK_CMD",
            0x75: "ORION_PKT_VIDEORECORD_STATUS",
            0x76: "ORION_PKT_VIDEORECORD_CMD",
            0x77: "ORION_PKT_VIDEORECORD_CLOCK",
            0x7A: "ORION_PKT_SIONYX_SETTINGS",
            0x7B: "ORION_PKT_LYNRED_SETTINGS",
            0x80: "ORION_PKT_AUTOPILOT_DATA",
            0xA0: "ORION_PKT_RETRACT_CMD",
            0xA1: "ORION_PKT_RETRACT_STATUS",
            0xA7: "ORION_PKT_PRODUCT",
            0xB0: "ORION_PKT_CROWN_MODE",
            0xB1: "ORION_PKT_DEBUG_STRING",
            0xB2: "ORION_PKT_USER_DATA",
            0xB3: "ORION_PKT_KLV_USER_DATA",
            0xD0: "ORION_PKT_SENSOR_DATA",
            0xD1: "ORION_PKT_GPS_DATA",
            0xD2: "ORION_PKT_EXT_HEADING_DATA",
            0xD3: "ORION_PKT_INS_QUALITY",
            0xD4: "ORION_PKT_GEOLOCATE_TELEMETRY_CORE",
            0xD5: "ORION_PKT_GEOPOINT_CMD",
            0xD6: "ORION_PKT_RANGE_DATA",
            0xD7: "ORION_PKT_PATH",
            0xD8: "ORION_PKT_INS_OPTIONS",
            0xD9: "ORION_PKT_STARE_START",
            0xDA: "ORION_PKT_STARE_ACK",
            0xDC: "ORION_PKT_GEOID_UNDULATION",
            0xE4: "ORION_PKT_NETWORK_SETTINGS",
            0xF8: "ORION_PKT_RETRACT_VERSION",
            0xF9: "ORION_PKT_LENSCTL_VERSION",
            0xFC: "ORION_PKT_SCAN_PLAN",
            0xFD: "ORION_PKT_UNIFIED_CAM_ERR_SETTINGS",
        }

    def parse(self, raw_data: bytes) -> OrionPacket:
        if len(raw_data) < 6:
            logger.error("Cannot parse packet: buffer too short (%d bytes, min 6)", len(raw_data))
            raise ValueError("Packet too short")
            
        sync0, sync1, p_id, length = struct.unpack(">BBBB", raw_data[:4])
        if sync0 != ORION_SYNC0 or sync1 != ORION_SYNC1:
            logger.error(
                "Invalid sync bytes: 0x%02X 0x%02X (expected 0x%02X 0x%02X)",
                sync0, sync1, ORION_SYNC0, ORION_SYNC1
            )
            raise ValueError("Invalid Sync bytes")
            
        if len(raw_data) < 4 + length:
            logger.warning(
                "Packet buffer shorter than declared payload: need %d bytes, got %d",
                4 + length, len(raw_data)
            )

        data = raw_data[4:4+length]
        pkt_name = self.packet_id_map.get(p_id)
        if pkt_name is None:
            logger.warning("Parsed packet with unknown packet ID: 0x%02X (len=%d)", p_id, length)
        elif logger.isEnabledFor(logging.DEBUG):
            logger.debug("Parsed packet %s (0x%02X, len=%d)", pkt_name, p_id, length)

        return OrionPacket(p_id, data)
