import asyncio
import argparse
import struct
from typing import List, Optional, Set, Tuple
from orion_shadow.core.engine import ProtocolEngine
from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.video_server import VideoServer

class OrionUDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, server: 'OrionServer'):
        self.server = server
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport
        self.server.transport = transport

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        self.server.handle_datagram(data, addr)

    def error_received(self, exc: Exception):
        print(f"[!] UDP error received: {exc}")

    def connection_lost(self, exc: Optional[Exception]):
        if exc:
            print(f"[!] UDP connection lost: {exc}")


class OrionServer:
    def __init__(self, host='0.0.0.0', port=8745, dt=0.1, dted_path: Optional[str] = None, 
                 tile_url: Optional[str] = None, video_port: int = 5004,
                 multicast_group: str = '239.255.0.1', video_enabled: bool = True):
        self.host = host
        self.port = port
        self.dt = dt
        self.terrain = TerrainEngine(dted_path)
        self.state = GimbalState(dt, self.terrain)
        self.engine = ProtocolEngine()
        self.clients: Set[Tuple[str, int]] = set()
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._tasks: List[asyncio.Task] = []
        self._running: bool = False
        
        # Multicast Video Server
        self.video_server: Optional[VideoServer] = None
        if video_enabled:
            self.video_server = VideoServer(
                self.state, 
                tile_url_template=tile_url, 
                multicast_group=multicast_group,
                port=video_port,
                host=host
            )

    def handle_datagram(self, data: bytes, addr: Tuple[str, int]):
        if addr not in self.clients:
            print("Got connection")
            print(f"[*] New connection from {addr}")
            self.clients.add(addr)
        
        offset = 0
        while offset + 6 <= len(data):
            if data[offset] == 0xD0 and data[offset + 1] == 0x0D:
                p_id = data[offset + 2]
                length = data[offset + 3]
                pkt_len = 4 + length + 2
                if offset + pkt_len <= len(data):
                    full_packet = data[offset:offset + pkt_len]
                    try:
                        packet = self.engine.parse(full_packet)
                        if packet:
                            self.state.update_from_command(packet)
                        # Echo behavior
                        if self.transport and not self.transport.is_closing():
                            self.transport.sendto(full_packet, addr)
                    except Exception as e:
                        print(f"[!] Error with {addr}: {e}")
                    offset += pkt_len
                else:
                    break
            else:
                offset += 1

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Deprecated TCP handler kept for backwards compatibility."""
        pass

    async def simulation_loop(self):
        """The 'Heartbeat' of the simulator: physics and telemetry."""
        while self._running:
            await asyncio.sleep(self.dt)
            if not self._running:
                break
            
            # 1. Advance Physics
            self.state.step()
            
            # 2. Broadcast Telemetry to all clients
            if self.clients and self.transport and not self.transport.is_closing():
                packets = [
                    self.state.get_telemetry_packet(),
                    self.state.get_laser_state_packet(),
                    self.state.get_camera_state_packet(),
                    self.state.get_sensor_data_packet(),
                    self.state.get_diagnostics_packet(),
                    self.state.get_video_options_packet(),
                    self.state.get_tracking_options_packet(),
                    self.state.get_cameras_packet(),
                    self.state.get_faults_packet(),
                ]
                        
                for addr in list(self.clients):
                    if not self._running or not self.transport or self.transport.is_closing():
                        break
                    try:
                        for p in packets:
                            self.transport.sendto(p, addr)
                    except Exception as e:
                        print(f"[!] Error sending telemetry to {addr}: {e}")
                        if addr in self.clients:
                            self.clients.remove(addr)

    def close(self):
        self._running = False
        if self.transport:
            try:
                self.transport.close()
            except Exception:
                pass
            self.transport = None
        if self.loop and self.loop.is_running():
            for t in list(self._tasks):
                self.loop.call_soon_threadsafe(t.cancel)

    async def run(self):
        self._running = True
        self.loop = asyncio.get_running_loop()
        try:
            transport, protocol = await self.loop.create_datagram_endpoint(
                lambda: OrionUDPProtocol(self),
                local_addr=(self.host, self.port),
                reuse_port=True
            )
        except (TypeError, OSError):
            transport, protocol = await self.loop.create_datagram_endpoint(
                lambda: OrionUDPProtocol(self),
                local_addr=(self.host, self.port)
            )
        self.transport = transport
        print(f"[+] Orion Simulator (Enhanced) active on {self.host}:{self.port}")
        print(f"[+] Physics Rate: {1/self.dt:.1f}Hz | Telemetry Rate: {1/self.dt:.1f}Hz")
        
        self._tasks = [asyncio.create_task(self.simulation_loop())]
        
        if self.video_server:
            print(f"[+] Video Stream (Multicast) active on udp://@{self.video_server.multicast_group}:{self.video_server.port}")
            self._tasks.append(asyncio.create_task(self.video_server.start()))
        
        try:
            await asyncio.gather(*self._tasks)
        except asyncio.CancelledError:
            pass
        finally:
            self._running = False
            for t in self._tasks:
                if not t.done():
                    t.cancel()
            if self.transport:
                self.transport.close()
                self.transport = None

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8745)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--dted-path", type=str, default=None, help="Path to DTED folder or file for terrain simulation")
    parser.add_argument("--tile-url", type=str, default=None, help="XYZ tile URL template (e.g. 'https://{z}/{x}/{y}.png')")
    parser.add_argument("--multicast-group", type=str, default="239.255.0.1", help="Multicast IP address for video stream (default: 239.255.0.1)")
    parser.add_argument("--video-port", type=int, default=5004, help="Multicast UDP port for video stream (default: 5004)")
    parser.add_argument("--no-video", action="store_true", help="Disable multicast video streaming")
    args = parser.parse_args()

    server = OrionServer(
        host=args.host, 
        port=args.port, 
        dt=args.dt, 
        dted_path=args.dted_path,
        tile_url=args.tile_url,
        video_port=args.video_port,
        multicast_group=args.multicast_group,
        video_enabled=not args.no_video
    )
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\n[!] Simulator shut down.")
