import unittest
import math
import struct
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from orion_shadow.core.state import GimbalState
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.server import OrionServer
from orion_shadow.engine.video_server import VideoServer


class TestInitialPosition(unittest.TestCase):
    def test_default_initial_position(self):
        state = GimbalState()
        self.assertEqual(state.gps_lat, 0.0)
        self.assertEqual(state.gps_lon, 0.0)
        self.assertEqual(state.gps_alt, 0.0)
        self.assertEqual(state.target_pan, 0.0)
        self.assertEqual(state.target_tilt, 0.0)
        self.assertEqual(state.current_pan, 0.0)
        self.assertEqual(state.current_tilt, 0.0)
        self.assertEqual(state.aircraft_heading, 0.0)
        self.assertFalse(state.gps_received)

    def test_custom_starting_coordinates(self):
        state = GimbalState(lat=41.8925, lon=-87.6242, alt=1000.0)
        self.assertAlmostEqual(state.gps_lat, 41.8925, places=4)
        self.assertAlmostEqual(state.gps_lon, -87.6242, places=4)
        self.assertAlmostEqual(state.gps_alt, 1000.0, places=1)
        # Automatic default tilt for airborne camera
        self.assertAlmostEqual(state.target_tilt, -20.0, places=1)
        self.assertAlmostEqual(state.current_tilt, -20.0, places=1)
        self.assertFalse(state.gps_received)

    def test_custom_tilt_and_pan_override(self):
        state = GimbalState(lat=41.8925, lon=-87.6242, alt=1000.0,
                            pan=15.0, tilt=-25.0, heading=180.0)
        self.assertAlmostEqual(state.gps_lat, 41.8925, places=4)
        self.assertAlmostEqual(state.gps_lon, -87.6242, places=4)
        self.assertAlmostEqual(state.gps_alt, 1000.0, places=1)
        self.assertAlmostEqual(state.target_pan, 15.0, places=1)
        self.assertAlmostEqual(state.current_pan, 15.0, places=1)
        self.assertAlmostEqual(state.target_tilt, -25.0, places=1)
        self.assertAlmostEqual(state.current_tilt, -25.0, places=1)
        self.assertAlmostEqual(state.aircraft_heading, 180.0, places=1)

    def test_geolocate_telemetry_reflects_starting_position(self):
        state = GimbalState(lat=41.8925, lon=-87.6242, alt=1000.0, tilt=-45.0)
        pkt_bytes = state.get_geolocate_telemetry_core_packet()
        # Decode payload: 4-byte header (D0 0D 0B len), data, 2-byte checksum
        self.assertEqual(pkt_bytes[0], 0xD0)
        self.assertEqual(pkt_bytes[1], 0x0D)
        self.assertEqual(pkt_bytes[2], OrionPktType.GEOLOCATE_TELEMETRY_CORE)
        data = pkt_bytes[4:-2]
        # Data structure: uptime(I), pad(I), pad(H), pad(h), lat_raw(i), lon_raw(i), alt_raw(i)
        lat_raw, lon_raw, alt_raw = struct.unpack_from(">iii", data, 12)
        lat_deg = lat_raw / 1e7
        lon_deg = lon_raw / 1e7
        alt_m = alt_raw / 10000.0
        self.assertAlmostEqual(lat_deg, 41.8925, places=4)
        self.assertAlmostEqual(lon_deg, -87.6242, places=4)
        self.assertAlmostEqual(alt_m, 1000.0, places=1)

    def test_reset_restores_starting_position(self):
        state = GimbalState(lat=41.8925, lon=-87.6242, alt=1000.0, tilt=-30.0)
        # Update with new GPS position from incoming stream
        gps_pkt = OrionPacket(OrionPktType.GPS_DATA, struct.pack(">fff", 34.05, -118.25, 5000.0))
        state.update_from_command(gps_pkt)
        self.assertAlmostEqual(state.gps_lat, 34.05, places=2)
        self.assertTrue(state.gps_received)

        # Send RESET
        reset_pkt = OrionPacket(OrionPktType.RESET, b"")
        state.update_from_command(reset_pkt)
        self.assertAlmostEqual(state.gps_lat, 41.8925, places=4)
        self.assertAlmostEqual(state.gps_lon, -87.6242, places=4)
        self.assertAlmostEqual(state.gps_alt, 1000.0, places=1)
        self.assertAlmostEqual(state.target_tilt, -30.0, places=1)
        self.assertFalse(state.gps_received)

    def test_orion_server_initialization_with_starting_position(self):
        server = OrionServer(
            lat=41.8925, lon=-87.6242, alt=1000.0,
            pan=10.0, tilt=-45.0, heading=90.0,
            video_enabled=False
        )
        self.assertAlmostEqual(server.state.gps_lat, 41.8925, places=4)
        self.assertAlmostEqual(server.state.gps_lon, -87.6242, places=4)
        self.assertAlmostEqual(server.state.gps_alt, 1000.0, places=1)
        self.assertAlmostEqual(server.state.current_pan, 10.0, places=1)
        self.assertAlmostEqual(server.state.current_tilt, -45.0, places=1)
        self.assertAlmostEqual(server.state.aircraft_heading, 90.0, places=1)

    def test_video_server_telemetry_and_mode_transitions(self):
        state = GimbalState(lat=41.8925, lon=-87.6242, alt=1000.0)
        vs = VideoServer(state)
        telem = vs._get_telemetry()
        self.assertAlmostEqual(telem['lat'], 41.8925, places=4)
        self.assertAlmostEqual(telem['lon'], -87.6242, places=4)
        self.assertAlmostEqual(telem['alt'], 1000.0, places=1)
        self.assertAlmostEqual(telem['tilt'], -20.0, places=1)
        self.assertAlmostEqual(telem['cam_pitch'], -20.0, places=1)

        # Before any GPS packets arrive, mode is SIMULATOR
        self.assertFalse(state.gps_received)

        # Incoming GPS stream arrives
        gps_pkt = OrionPacket(OrionPktType.GPS_DATA, struct.pack(">fff", 41.89, -87.62, 1020.0))
        state.update_from_command(gps_pkt)
        self.assertTrue(state.gps_received)


if __name__ == "__main__":
    unittest.main()
