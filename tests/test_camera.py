import unittest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState


class TestCamera(unittest.TestCase):
    def test_camera_switch_and_cmd(self):
        state = GimbalState()

        # 1. Test Switch
        switch_packet = OrionPacket(OrionPktType.CAMERA_SWITCH, b'\x02')
        state.update_from_command(switch_packet)
        self.assertEqual(state.camera_id, 2)
        self.assertFalse(state.camera_ready)

        # 2. Test Command (Zoom/Focus)
        # zoom=2.5, focus=0.5
        cmd_data = struct.pack(">ff", 2.5, 0.5)
        cmd_packet = OrionPacket(OrionPktType.CAMERA_CMD, cmd_data)
        state.update_from_command(cmd_packet)

        self.assertAlmostEqual(state.camera_zoom, 2.5)
        self.assertAlmostEqual(state.camera_focus, 0.5)
        self.assertTrue(state.camera_ready)

    def test_camera_state_variable_lengths(self):
        state = GimbalState()

        # Test 2-byte CAMERA_STATE (Zoom only, big-endian int16 scaled by 100.0)
        # e.g., 5.0x zoom -> raw value 500
        raw_zoom = int(round(5.0 * 100.0))
        pkt_2b = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">h", raw_zoom))
        state.update_from_command(pkt_2b)
        self.assertAlmostEqual(state.camera_zoom, 5.0)

        # Zoom out to 1.0x (wide angle) -> raw value 100
        raw_zoom_wide = int(round(1.0 * 100.0))
        pkt_wide = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">h", raw_zoom_wide))
        state.update_from_command(pkt_wide)
        self.assertAlmostEqual(state.camera_zoom, 1.0)

        # Values < 1.0 should not change zoom according to Orion SDK spec
        pkt_zero = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">h", 0))
        state.update_from_command(pkt_zero)
        self.assertAlmostEqual(state.camera_zoom, 1.0)

        # Test 4-byte CAMERA_STATE (Zoom + Focus, both int16)
        # Zoom 3.5x (350), Focus 0.5 (scaled by 10000.0 = 5000)
        pkt_4b = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">hh", 350, 5000))
        state.update_from_command(pkt_4b)
        self.assertAlmostEqual(state.camera_zoom, 3.5)
        self.assertAlmostEqual(state.camera_focus, 0.5)

        # Test 5-byte CAMERA_STATE (Zoom + Focus + Flags/Index)
        # Zoom 2.0x (200), Focus 0.8 (8000), KeepActiveCamera=1, Index=0 -> flags=0x80
        pkt_5b = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">hhB", 200, 8000, 0x80))
        state.update_from_command(pkt_5b)
        self.assertAlmostEqual(state.camera_zoom, 2.0)
        self.assertAlmostEqual(state.camera_focus, 0.8)

        # Test 8-byte legacy (>ff format)
        pkt_8b = OrionPacket(OrionPktType.CAMERA_STATE, struct.pack(">ff", 4.2, 0.8))
        state.update_from_command(pkt_8b)
        self.assertAlmostEqual(state.camera_zoom, 4.2, places=5)
        self.assertAlmostEqual(state.camera_focus, 0.8, places=5)

        # Test 0-byte CAMERA_STATE (Query) returns telemetry response
        pkt_query = OrionPacket(OrionPktType.CAMERA_STATE, b'')
        resp = state.update_from_command(pkt_query)
        self.assertIsNotNone(resp)

    def test_camera_telemetry(self):
        state = GimbalState()
        state.camera_zoom = 3.0
        state.camera_focus = 0.7
        state.camera_ready = True
        state.camera_id = 0

        telemetry_bytes = state.get_camera_state_packet()

        # OrionPublicProtocol CAMERA_STATE payload is 5 bytes (>hhB).
        # Total encoded length = 2 sync + 1 id + 1 len + 5 payload + 2 checksum = 11 bytes.
        self.assertEqual(len(telemetry_bytes), 11)

        # Unpack payload part: zoom (int16/100), focus (int16/10000), cam_idx (uint8)
        # Offset 4 is the start of payload
        zoom_raw, focus_raw, cam_idx = struct.unpack(">hhB", telemetry_bytes[4:9])

        self.assertAlmostEqual(zoom_raw / 100.0, 3.0)
        self.assertAlmostEqual(focus_raw / 10000.0, 0.7, places=4)
        self.assertEqual(cam_idx, 0)


if __name__ == "__main__":
    unittest.main()
