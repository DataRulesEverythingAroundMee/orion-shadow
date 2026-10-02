import math
import struct
import unittest
import asyncio
import threading
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "orion-sdk", "Communications", "python")))

from orion_shadow.core.protocol import OrionPacket, OrionPktType, OrionMode
from orion_shadow.core.state import GimbalState
from orion_shadow.server import OrionServer
from orion_shadow.engine.video_server import VideoServer

try:
    from orion_sdk.packets import GeopointCmd, GeolocateTelemetryCore
    from orion_sdk.enums import geopointOptions
    from orion_sdk.connection import OrionConnection
    HAS_ORION_SDK = True
except ImportError:
    HAS_ORION_SDK = False


class TestGeopointMode(unittest.TestCase):
    def setUp(self):
        # Aircraft starts at (41.8925, -87.6242), 1000m MSL, heading North (0 deg), pitch 0 deg
        self.state = GimbalState(
            lat=41.8925, lon=-87.6242, alt=1000.0,
            pan=0.0, tilt=-20.0, heading=0.0, speed=0.0
        )

    def test_geopoint_constants(self):
        self.assertEqual(OrionPktType.GEOPOINT_CMD, 0xD5)
        self.assertEqual(OrionMode.GEOPOINT, 0x60)
        self.assertEqual(OrionMode.GEOPOINT, 96)

    def test_geopoint_pan_tilt_due_north(self):
        """Target 1000m due North, 0m MSL: pan=0 deg, tilt=-45 deg."""
        target_lat = 41.8925 + 1000.0 / 111320.0
        target_lon = -87.6242
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        resp = self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        self.assertIsNotNone(resp)
        self.assertEqual(self.state.mode, OrionMode.GEOPOINT)
        self.assertAlmostEqual(self.state.target_pan, 0.0, places=1)
        self.assertAlmostEqual(self.state.target_tilt, -45.0, places=1)

    def test_geopoint_pan_tilt_due_east(self):
        """Target ~1000m due East, 0m MSL: pan=+90 deg, tilt=-45 deg."""
        lat_rad = math.radians(41.8925)
        d_lon = 1000.0 / (111320.0 * math.cos(lat_rad))
        target_lat = 41.8925
        target_lon = -87.6242 + d_lon
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        self.assertEqual(self.state.mode, OrionMode.GEOPOINT)
        self.assertAlmostEqual(self.state.target_pan, 90.0, delta=0.5)
        self.assertAlmostEqual(self.state.target_tilt, -45.0, places=1)

    def test_geopoint_with_aircraft_heading_and_pitch(self):
        """With aircraft heading=45 deg, pitch=5 deg, pan and tilt compensate exactly."""
        self.state.aircraft_heading = 45.0
        self.state.aircraft_pitch = 5.0

        # Target due North (azimuth 0 deg, elevation -45 deg)
        target_lat = 41.8925 + 1000.0 / 111320.0
        target_lon = -87.6242
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        # Expected pan: 0 - 45 = -45 deg
        # Expected tilt: -45 - 5 = -50 deg
        self.assertAlmostEqual(self.state.target_pan, -45.0, places=1)
        self.assertAlmostEqual(self.state.target_tilt, -50.0, places=1)

    def test_geopoint_closure_mode_snaps_instantly(self):
        """Option 0x02 (geopointClosure) drives gimbal immediately to target."""
        target_lat = 41.8925 + 1000.0 / 111320.0
        target_lon = -87.6242
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        # 21-byte packet with options=0x02 (closure)
        pkt_data = struct.pack(">iiihhhHB", raw_lat, raw_lon, raw_alt, 0, 0, 0, 0, 0x02)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        self.assertEqual(self.state.mode, OrionMode.GEOPOINT)
        self.assertAlmostEqual(self.state.current_pan, 0.0, places=1)
        self.assertAlmostEqual(self.state.current_tilt, -45.0, places=1)

    def test_geopoint_continuous_tracking_with_aircraft_movement(self):
        """As the aircraft flies forward, gimbal continuously adjusts tilt to keep center locked."""
        target_lat = 41.8925 + 1000.0 / 111320.0
        target_lon = -87.6242
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhhHB", raw_lat, raw_lon, raw_alt, 0, 0, 0, 0, 0x02)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))
        self.assertAlmostEqual(self.state.target_tilt, -45.0, places=1)

        # Move aircraft closer to target: 500m North
        self.state.gps_lat += 500.0 / 111320.0
        self.state.step()

        # Distance now ~500m, alt=1000m -> elevation = atan2(-1000, 500) ~ -63.4 deg
        self.assertAlmostEqual(self.state.target_tilt, math.degrees(math.atan2(-1000.0, 500.0)), delta=0.5)

    def test_geopoint_target_velocity_propagation(self):
        """If target has velocity (e.g. driving North at 20 m/s), target propagates each step."""
        target_lat = 41.8925 + 0.009
        target_lon = -87.6242
        target_alt = 0.0

        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        # 20 m/s North -> raw_vn = 2000
        pkt_data = struct.pack(">iiihhhHB", raw_lat, raw_lon, raw_alt, 2000, 0, 0, 0, 0x00)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        initial_tgt_lat = self.state.geopoint_lat
        # Step forward 10 timesteps (dt=0.1s -> 1.0 second total = 20 meters North)
        for _ in range(10):
            self.state.step()

        expected_dlat = 20.0 / 111320.0
        self.assertAlmostEqual(self.state.geopoint_lat - initial_tgt_lat, expected_dlat, places=6)

    def test_orion_cmd_cancels_geopoint_mode(self):
        """Sending OrionCmd packet cancels ORION_MODE_GEOPOINT."""
        target_lat = 41.8925 + 0.009
        target_lon = -87.6242
        target_alt = 0.0
        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))
        self.assertEqual(self.state.mode, OrionMode.GEOPOINT)

        # Now send standard OrionCmd in position mode (0x50)
        cmd_packet = OrionPacket(OrionPktType.CMD, struct.pack(">hhBBB", 0, 0, 0x50, 0, 0))
        self.state.update_from_command(cmd_packet)
        self.assertEqual(self.state.mode, 0x50)
        self.assertNotEqual(self.state.mode, OrionMode.GEOPOINT)

    def test_reset_cancels_geopoint_mode(self):
        """Sending RESET cancels ORION_MODE_GEOPOINT and resets state."""
        raw_lat = int(round(41.89 * 1e7))
        raw_lon = int(round(-87.62 * 1e7))
        raw_alt = int(round(100.0 * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))
        self.assertEqual(self.state.mode, OrionMode.GEOPOINT)

        self.state.update_from_command(OrionPacket(OrionPktType.RESET, b""))
        self.assertEqual(self.state.mode, OrionMode.RATE)

    def test_geolocate_telemetry_core_reports_geopoint_mode(self):
        """GeolocateTelemetryCore packet reports mode=96 (0x60) when in geopoint mode."""
        target_lat = 41.8925 + 0.009
        target_lon = -87.6242
        target_alt = 0.0
        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        telemetry_bytes = self.state.get_geolocate_telemetry_core_packet()
        self.assertEqual(telemetry_bytes[0], 0xD0)
        self.assertEqual(telemetry_bytes[1], 0x0D)
        self.assertEqual(telemetry_bytes[2], OrionPktType.GEOLOCATE_TELEMETRY_CORE)

        data = telemetry_bytes[4:-2]
        # mode byte is at offset 38: uptime(4)+pad(4)+pad(2)+pad(2)+lat(4)+lon(4)+alt(4)+vel(6)+quat(8)+pan(2)+tilt(2)+hfov(2)+vfov(2)+los(6)+dim(4) = 46
        # In state.py: 4+4+2+2=12 (time,pad) + 12 (lat,lon,alt) + 6 (velNED) + 8 (quat) + 4 (pan,tilt) + 4 (fov) + 6 (los) + 4 (wh) = 56
        mode_val = data[56]
        self.assertEqual(mode_val, 96)  # 0x60 ORION_MODE_GEOPOINT

    def test_video_server_hud_geopoint_mode(self):
        """VideoServer._draw_hud displays GEOPOINT status and GEO LOCK reticle."""
        target_lat = 41.8925 + 0.009
        target_lon = -87.6242
        target_alt = 0.0
        raw_lat = int(round(target_lat * 1e7))
        raw_lon = int(round(target_lon * 1e7))
        raw_alt = int(round(target_alt * 10000.0))
        pkt_data = struct.pack(">iiihhh", raw_lat, raw_lon, raw_alt, 0, 0, 0)
        self.state.update_from_command(OrionPacket(OrionPktType.GEOPOINT_CMD, pkt_data))

        vs = VideoServer(self.state)
        try:
            import numpy as np
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            hud_frame = vs._draw_hud(frame)
            self.assertIsNotNone(hud_frame)
            self.assertEqual(hud_frame.shape, (720, 1280, 3))
        except ImportError:
            pass

    def test_server_tcp_geopoint_cmd_roundtrip(self):
        """End-to-end: client connects via TCP, sends GeopointCmd, receives GeopointCmd acknowledgment."""
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('127.0.0.1', 0))
        tcp_port = s.getsockname()[1]
        s.close()

        server = OrionServer(
            host="127.0.0.1", port=0, tcp_port=tcp_port, dt=0.05, video_enabled=False,
            lat=41.8925, lon=-87.6242, alt=1000.0, heading=0.0
        )
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.15)

        try:
            client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_sock.connect(("127.0.0.1", tcp_port))

            # Send GeopointCmd packet: target 1000m North, alt 0m
            target_lat = 41.8925 + 1000.0 / 111320.0
            target_lon = -87.6242
            target_alt = 0.0
            raw_lat = int(round(target_lat * 1e7))
            raw_lon = int(round(target_lon * 1e7))
            raw_alt = int(round(target_alt * 10000.0))
            payload = struct.pack(">iiihhhHB", raw_lat, raw_lon, raw_alt, 0, 0, 0, 0, 0x02)
            pkt = OrionPacket(OrionPktType.GEOPOINT_CMD, payload).encode()
            client_sock.sendall(pkt)

            # Read response
            client_sock.settimeout(2.0)
            resp_buf = client_sock.recv(1024)
            client_sock.close()

            self.assertGreaterEqual(len(resp_buf), 6)
            self.assertEqual(resp_buf[0], 0xD0)
            self.assertEqual(resp_buf[1], 0x0D)
            self.assertEqual(resp_buf[2], OrionPktType.GEOPOINT_CMD)

            # Server state should be in GEOPOINT mode with center locked
            self.assertEqual(server.state.mode, OrionMode.GEOPOINT)
            self.assertAlmostEqual(server.state.target_pan, 0.0, places=1)
            self.assertAlmostEqual(server.state.target_tilt, -45.0, places=1)
        finally:
            server.close()
            thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
