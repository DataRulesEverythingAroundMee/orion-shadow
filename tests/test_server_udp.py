import unittest
import socket
import struct
import time
import asyncio
import threading
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts")))

from orion_shadow.server import OrionServer
from orion_shadow.core.protocol import OrionPacket, OrionPktType, UDP_OUT_PORT, UDP_IN_PORT, TCP_PORT
from orion_shadow.core.engine import ProtocolEngine


def get_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class TestOrionServerUDP(unittest.TestCase):
    def setUp(self):
        self.port = get_free_port()
        self.server = OrionServer(host="127.0.0.1", port=self.port, tcp_port=None, dt=0.05, video_enabled=False)
        self.server_thread = threading.Thread(target=lambda: asyncio.run(self.server.run()), daemon=True)
        self.server_thread.start()
        for _ in range(50):
            if self.server.transport is not None:
                break
            time.sleep(0.01)

    def tearDown(self):
        self.server.close()
        self.server_thread.join(timeout=1.0)

    def test_udp_initialize_and_echo(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2.0)
            init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
            sock.sendto(init_pkt, ("127.0.0.1", self.port))

            resp, addr = sock.recvfrom(1024)
            self.assertEqual(addr[0], "127.0.0.1")
            self.assertEqual(resp, init_pkt)

            time.sleep(0.1)
            self.assertTrue(self.server.state.initialized)

    def test_udp_command_packet(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2.0)
            cmd_payload = struct.pack(">ff", 30.0, -15.0)
            cmd_pkt = OrionPacket(OrionPktType.CMD, cmd_payload).encode()
            sock.sendto(cmd_pkt, ("127.0.0.1", self.port))

            resp, _ = sock.recvfrom(1024)
            self.assertEqual(resp, cmd_pkt)

            time.sleep(0.1)
            self.assertAlmostEqual(self.server.state.target_pan, 30.0, places=2)
            self.assertAlmostEqual(self.server.state.target_tilt, -15.0, places=2)

    def test_udp_multi_packet_datagram(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2.0)
            gps_payload = struct.pack(">fff", 37.7749, -122.4194, 2500.0)
            gps_pkt = OrionPacket(OrionPktType.GPS_DATA, gps_payload).encode()

            heading_payload = struct.pack(">fff", 270.0, 0.0, 0.0)
            heading_pkt = OrionPacket(OrionPktType.EXT_HEADING_DATA, heading_payload).encode()

            sock.sendto(gps_pkt + heading_pkt, ("127.0.0.1", self.port))

            echo1, _ = sock.recvfrom(1024)
            echo2, _ = sock.recvfrom(1024)
            self.assertEqual(echo1, gps_pkt)
            self.assertEqual(echo2, heading_pkt)

            time.sleep(0.1)
            self.assertAlmostEqual(self.server.state.gps_lat, 37.7749, places=4)
            self.assertAlmostEqual(self.server.state.gps_lon, -122.4194, places=4)
            self.assertAlmostEqual(self.server.state.gps_alt, 2500.0, places=1)
            self.assertAlmostEqual(self.server.state.aircraft_heading, 270.0, places=1)

    def test_udp_telemetry_broadcast(self):
        engine = ProtocolEngine()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2.0)
            init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
            sock.sendto(init_pkt, ("127.0.0.1", self.port))
            _ = sock.recvfrom(1024)  # discard echo

            received_packet_ids = set()
            for _ in range(15):
                try:
                    data, _ = sock.recvfrom(1024)
                    parsed = engine.parse(data)
                    received_packet_ids.add(parsed.packet_id)
                except socket.timeout:
                    break

            self.assertIn(OrionPktType.POSITIONS, received_packet_ids)

    def test_send_init_script_udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2.0)
            init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
            sock.sendto(init_pkt, ("127.0.0.1", self.port))
            resp, _ = sock.recvfrom(1024)
            self.assertEqual(resp, init_pkt)
            self.assertTrue(self.server.state.initialized)


class TestOrionServerSdkPorts(unittest.TestCase):
    def test_default_port_constants(self):
        self.assertEqual(UDP_OUT_PORT, 8745)
        self.assertEqual(UDP_IN_PORT, 8746)
        self.assertEqual(TCP_PORT, 8747)

    def test_tcp_server_communication(self):
        udp_port = get_free_port()
        tcp_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_port, tcp_port=tcp_port, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(2.0)
                sock.connect(("127.0.0.1", tcp_port))

                # Send INITIALIZE command over TCP
                init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
                sock.sendall(init_pkt)
                echo = sock.recv(len(init_pkt))
                self.assertEqual(echo, init_pkt)
                self.assertTrue(server.state.initialized)

                # Send CMD packet over TCP
                cmd_pkt = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 22.5, -15.0)).encode()
                sock.sendall(cmd_pkt)
                echo_cmd = sock.recv(len(cmd_pkt))
                self.assertEqual(echo_cmd, cmd_pkt)
                self.assertAlmostEqual(server.state.target_pan, 22.5, places=2)
                self.assertAlmostEqual(server.state.target_tilt, -15.0, places=2)

                # Receive telemetry over TCP
                telem = sock.recv(1024)
                self.assertGreater(len(telem), 0)
                self.assertEqual(telem[0], 0xD0)
                self.assertEqual(telem[1], 0x0D)
        finally:
            server.close()
            thread.join(timeout=1.0)

    def test_discovery_response_to_udp_in_port(self):
        udp_out_port = get_free_port()
        udp_in_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_out_port, udp_in_port=udp_in_port, tcp_port=None, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            # Client socket listening on udp_in_port (like OrionCommLinux recvfrom)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as in_sock:
                in_sock.bind(("127.0.0.1", udp_in_port))
                in_sock.settimeout(2.0)

                # Client sender socket sending to udp_out_port
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as send_sock:
                    init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
                    send_sock.sendto(init_pkt, ("127.0.0.1", udp_out_port))

                # Verify discovery response arrives on in_sock (udp_in_port)
                resp, _ = in_sock.recvfrom(1024)
                self.assertEqual(resp, init_pkt)
        finally:
            server.close()
            thread.join(timeout=1.0)

    def test_camera_info_request_response(self):
        udp_port = get_free_port()
        tcp_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_port, tcp_port=tcp_port, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            sys.path.insert(0, "/home/user/dev/orion-sdk/Communications/python")
            from orion_sdk.connection import OrionConnection
            from orion_sdk.packets import OrionCameras

            conn = OrionConnection.open_tcp("127.0.0.1", tcp_port)
            conn.send(OrionCameras())

            received_cam_pkt = None
            for _ in range(20):
                pkt = conn.receive(timeout=0.1)
                if isinstance(pkt, OrionCameras) and pkt.NumCameras > 0:
                    received_cam_pkt = pkt
                    break

            conn.close()
            self.assertIsNotNone(received_cam_pkt)
            self.assertEqual(received_cam_pkt.NumCameras, 1)
            self.assertEqual(received_cam_pkt.OrionCamSettings[0].Type, 1)  # Visible
            self.assertEqual(received_cam_pkt.OrionCamSettings[0].Proto, 7)  # KTnC

            # Test OrionKtncSettings packet
            from orion_sdk.packets import OrionKtncSettings
            conn2 = OrionConnection.open_tcp("127.0.0.1", tcp_port)
            conn2.send(OrionKtncSettings(Index=0, Sharpness=10))
            ktnc_resp = conn2.receive(timeout=0.5)
            conn2.close()
            self.assertIsNotNone(ktnc_resp)
            self.assertIsInstance(ktnc_resp, OrionKtncSettings)
            self.assertEqual(server.state.ktnc_sharpness, 10)
        finally:
            server.close()
            thread.join(timeout=1.0)

    def test_geolocate_telemetry_core_broadcast(self):
        udp_port = get_free_port()
        tcp_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_port, tcp_port=tcp_port, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            sys.path.insert(0, "/home/user/dev/orion-sdk/Communications/python")
            from orion_sdk.connection import OrionConnection
            from orion_sdk.packets import GeolocateTelemetryCore

            conn = OrionConnection.open_tcp("127.0.0.1", tcp_port)
            received_geo_pkt = None
            for _ in range(20):
                pkt = conn.receive(timeout=0.2)
                if isinstance(pkt, GeolocateTelemetryCore):
                    received_geo_pkt = pkt
                    break

            conn.close()
            self.assertIsNotNone(received_geo_pkt)
            self.assertAlmostEqual(received_geo_pkt.pan, 0.0, places=2)
            self.assertAlmostEqual(received_geo_pkt.tilt, 0.0, places=2)
            import math
            self.assertAlmostEqual(math.degrees(received_geo_pkt.hfov), 47.7, places=1)
            self.assertAlmostEqual(math.degrees(received_geo_pkt.vfov), 28.0, places=0)
            self.assertEqual(received_geo_pkt.mode, 16)
        finally:
            server.close()
            thread.join(timeout=1.0)

    def test_pan_continuous_and_tilt_range_limits(self):
        udp_port = get_free_port()
        tcp_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_port, tcp_port=tcp_port, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            sys.path.insert(0, "/home/user/dev/orion-sdk/Communications/python")
            from orion_sdk.connection import OrionConnection
            from orion_sdk.packets import OrionLimitsData
            import math

            conn = OrionConnection.open_tcp("127.0.0.1", tcp_port)

            # Query LIMITS packet
            conn.send(OrionLimitsData())
            limits_pkt = None
            for _ in range(20):
                pkt = conn.receive(timeout=0.1)
                if isinstance(pkt, OrionLimitsData):
                    limits_pkt = pkt
                    break

            self.assertIsNotNone(limits_pkt)
            self.assertAlmostEqual(math.degrees(limits_pkt.MinPos[1]), -80.0, places=1)
            self.assertAlmostEqual(math.degrees(limits_pkt.MaxPos[1]), 28.0, places=1)
            self.assertAlmostEqual(math.degrees(limits_pkt.MinPos[0]), -180.0, places=1)
            self.assertAlmostEqual(math.degrees(limits_pkt.MaxPos[0]), 180.0, places=1)
            self.assertAlmostEqual(limits_pkt.MaxPower[0], 75.0, places=0)
            self.assertAlmostEqual(limits_pkt.ContCur[0], 0.625, places=2)
            self.assertAlmostEqual(limits_pkt.PeakCur[0], 3.125, places=2)

            # Test command outside tilt range (exceeds +28°)
            cmd_high_tilt = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 45.0, 50.0)).encode()
            conn.send_raw(cmd_high_tilt)
            time.sleep(0.1)
            self.assertAlmostEqual(server.state.target_tilt, 28.0, places=2)

            # Test command outside tilt range (below -80°)
            cmd_low_tilt = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 45.0, -95.0)).encode()
            conn.send_raw(cmd_low_tilt)
            time.sleep(0.1)
            self.assertAlmostEqual(server.state.target_tilt, -80.0, places=2)

            # Test continuous pan wrapping across 360° (190° -> -170°)
            cmd_wrap_pan = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 190.0, 0.0)).encode()
            conn.send_raw(cmd_wrap_pan)
            time.sleep(0.1)
            self.assertAlmostEqual(server.state.target_pan, -170.0, places=2)

            conn.close()
        finally:
            server.close()
            thread.join(timeout=1.0)

    def test_camera_zoom_optical_and_digital_limits(self):
        udp_port = get_free_port()
        tcp_port = get_free_port()
        server = OrionServer(host="127.0.0.1", port=udp_port, tcp_port=tcp_port, dt=0.05, video_enabled=False)
        thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        thread.start()
        time.sleep(0.1)

        try:
            sys.path.insert(0, "/home/user/dev/orion-sdk/Communications/python")
            from orion_sdk.connection import OrionConnection
            from orion_sdk.packets import OrionCameraState, GeolocateTelemetryCore
            import math

            conn = OrionConnection.open_tcp("127.0.0.1", tcp_port)

            # 1. Command 30x optical zoom
            conn.send(OrionCameraState(Zoom=30.0))
            time.sleep(0.15)
            self.assertAlmostEqual(server.state.camera_zoom, 30.0, places=1)

            # Receive telemetry and verify HFOV at 30x optical zoom (~1.7°–1.8°)
            found_30 = False
            for _ in range(30):
                pkt = conn.receive(timeout=0.1)
                if isinstance(pkt, GeolocateTelemetryCore) and math.degrees(pkt.hfov) < 2.0:
                    self.assertAlmostEqual(math.degrees(pkt.hfov), 1.7, places=1)
                    found_30 = True
                    break
            self.assertTrue(found_30)

            # 2. Command 112x total zoom (digital zoom)
            conn.send(OrionCameraState(Zoom=112.0))
            time.sleep(0.15)
            self.assertAlmostEqual(server.state.camera_zoom, 112.0, places=1)

            found_112 = False
            for _ in range(30):
                pkt = conn.receive(timeout=0.1)
                if isinstance(pkt, GeolocateTelemetryCore) and math.degrees(pkt.hfov) < 1.0:
                    self.assertAlmostEqual(math.degrees(pkt.hfov), 0.45, places=1)
                    found_112 = True
                    break
            self.assertTrue(found_112)

            # 3. Command beyond 112x (should clamp to max_total_zoom = 112.0)
            conn.send(OrionCameraState(Zoom=150.0))
            time.sleep(0.1)
            self.assertEqual(server.state.camera_zoom, 112.0)

            conn.close()
        finally:
            server.close()
            thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
