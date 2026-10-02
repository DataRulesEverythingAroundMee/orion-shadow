import math
import struct
import unittest
from unittest.mock import MagicMock

from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.video_server import VideoServer
from orion_shadow.server import OrionServer

try:
    import numpy as np
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class TestHudSpeed(unittest.TestCase):
    def setUp(self):
        self.terrain = TerrainEngine()

    def test_gimbal_state_initial_speed(self):
        """Verify speed parameter sets initial and current aircraft speed, and reset restores it."""
        state = GimbalState(0.1, self.terrain, speed=120.0)
        self.assertEqual(state.aircraft_speed, 120.0)
        self.assertEqual(state.initial_speed, 120.0)

        # Alter speed
        state.aircraft_speed = 250.0
        self.assertEqual(state.aircraft_speed, 250.0)

        # Reset packet restores initial speed
        reset_pkt = OrionPacket(OrionPktType.RESET, b"")
        state.update_from_command(reset_pkt)
        self.assertEqual(state.aircraft_speed, 120.0)

    def test_simulated_motion_with_speed(self):
        """Verify state.step() moves aircraft along heading when speed > 0 and no external GPS."""
        state = GimbalState(0.1, self.terrain, lat=35.0, lon=-117.0, heading=0.0, speed=194.384)
        # 194.384 knots = 100 m/s. In 0.1s dt, aircraft moves 10 meters North.
        initial_lat = state.gps_lat
        initial_lon = state.gps_lon

        state.step()

        self.assertGreater(state.gps_lat, initial_lat)
        self.assertAlmostEqual(state.gps_lon, initial_lon, places=5)

        # Heading East (90 deg)
        state.aircraft_heading = 90.0
        cur_lat = state.gps_lat
        cur_lon = state.gps_lon
        state.step()
        self.assertAlmostEqual(state.gps_lat, cur_lat, places=5)
        self.assertGreater(state.gps_lon, cur_lon)

    def test_gps_data_explicit_4float_speed(self):
        """Verify GPS_DATA with 4 floats parses lat, lon, alt, and speed."""
        state = GimbalState(0.1, self.terrain)
        payload = struct.pack(">ffff", 41.8925, -87.6242, 2500.0, 165.0)
        pkt = OrionPacket(OrionPktType.GPS_DATA, payload)
        state.update_from_command(pkt)

        self.assertTrue(state.gps_received)
        self.assertAlmostEqual(state.gps_lat, 41.8925, places=4)
        self.assertAlmostEqual(state.gps_lon, -87.6242, places=4)
        self.assertAlmostEqual(state.gps_alt, 2500.0, places=1)
        self.assertAlmostEqual(state.aircraft_speed, 165.0, places=1)

    def test_gps_data_velned_speed(self):
        """Verify GPS_DATA with 28+ byte Orion SDK VelNED parses velocity into knots."""
        state = GimbalState(0.1, self.terrain)
        # Orion standard GPS packet:
        # 4 bytes header / flags
        # lat (int32 * 1e7), lon (int32 * 1e7), alt (int32 * 10000)
        # VelNED: vn (int32 mm/s), ve (int32 mm/s), vd (int32 mm/s)
        # vn = 30000 mm/s (30 m/s), ve = 40000 mm/s (40 m/s) -> 50 m/s = 50 * 1.94384 = 97.192 kts
        data = (
            struct.pack(">I", 0) +
            struct.pack(">iii", int(34.0 * 1e7), int(-118.0 * 1e7), int(1000.0 * 10000)) +
            struct.pack(">iii", 30000, 40000, 0)
        )
        pkt = OrionPacket(OrionPktType.GPS_DATA, data)
        state.update_from_command(pkt)

        self.assertAlmostEqual(state.gps_lat, 34.0, places=5)
        self.assertAlmostEqual(state.gps_lon, -118.0, places=5)
        self.assertAlmostEqual(state.gps_alt, 1000.0, places=1)
        expected_kts = 50.0 * 1.94384
        self.assertAlmostEqual(state.aircraft_speed, expected_kts, places=2)

    def test_video_server_telemetry_speed(self):
        """Verify VideoServer._get_telemetry() extracts aircraft_speed."""
        state = GimbalState(0.1, self.terrain, speed=135.5)
        server = VideoServer(state)
        telem = server._get_telemetry()
        self.assertIn('speed', telem)
        self.assertEqual(telem['speed'], 135.5)

    def test_video_server_draw_hud(self):
        """Verify _draw_hud draws HUD overlay including airspeed box and footer speed."""
        import orion_shadow.engine.video_server as vs_mod
        orig_cv2 = vs_mod.cv2
        orig_np = vs_mod.np
        try:
            mock_cv2 = MagicMock()
            mock_np = MagicMock()
            class DummyArray:
                shape = (720, 1280, 3)
            mock_np.ndarray = DummyArray
            vs_mod.cv2 = mock_cv2
            vs_mod.np = mock_np

            state = GimbalState(0.1, self.terrain, speed=142.0)
            state.initialized = True
            server = VideoServer(state, width=1280, height=720)
            dummy_frame = DummyArray()
            ret = server._draw_hud(dummy_frame)
            self.assertIs(ret, dummy_frame)

            # Check that putText was called with speed indicator and footer
            putText_calls = [c[0][1] for c in mock_cv2.putText.call_args_list if len(c[0]) > 1]
            self.assertTrue(any("142 KTS" in str(txt) for txt in putText_calls), f"Expected 142 KTS in putText calls: {putText_calls}")
            self.assertTrue(any("SPD: 142 kts" in str(txt) for txt in putText_calls), f"Expected SPD: 142 kts in footer: {putText_calls}")
        finally:
            vs_mod.cv2 = orig_cv2
            vs_mod.np = orig_np

    def test_server_speed_argument(self):
        """Verify OrionServer passes speed to state."""
        server = OrionServer(video_enabled=False, speed=185.0)
        self.assertEqual(server.initial_speed, 185.0)
        self.assertEqual(server.state.aircraft_speed, 185.0)

    def test_adsb_attach_bridge_send_speed(self):
        """Verify OrionBridge in adsb_attach sends 4-float GPS_DATA with speed."""
        import sys
        import os
        scripts_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        import adsb_attach

        bridge = adsb_attach.OrionBridge(host="127.0.0.1", port=8745)
        bridge.is_connected = True
        bridge.sock = MagicMock()

        success = bridge.send_aircraft_position(41.8, -87.6, 1200.0, 180.0, 210.0)
        self.assertTrue(success)
        self.assertEqual(bridge.sock.sendto.call_count, 1)

        sent_bytes, addr = bridge.sock.sendto.call_args[0]
        # First packet is GPS_DATA (packet_id 0xD1)
        # Header: D0 0D D1 len(16)
        self.assertEqual(sent_bytes[0], 0xD0)
        self.assertEqual(sent_bytes[1], 0x0D)
        self.assertEqual(sent_bytes[2], 0xD1)
        payload_len = sent_bytes[3]
        self.assertEqual(payload_len, 16)
        payload = sent_bytes[4:20]
        lat, lon, alt, spd = struct.unpack(">ffff", payload)
        self.assertAlmostEqual(lat, 41.8, places=3)
        self.assertAlmostEqual(lon, -87.6, places=3)
        self.assertAlmostEqual(alt, 1200.0, places=1)
        self.assertAlmostEqual(spd, 210.0, places=1)


if __name__ == "__main__":
    unittest.main()
