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


if __name__ == "__main__":
    unittest.main()
