import asyncio
import argparse
import struct
from typing import List, Optional
from orion_shadow.core.engine import ProtocolEngine
from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.video_server import VideoServer

class OrionServer:
    def __init__(self, host='0.0.0.0', port=5000, dt=0.1, dted_path: Optional[str] = None, 
                 tile_url: Optional[str] = None, video_port: int = 5001):
        self.host = host
        self.port = port
        self.dt = dt
        self.terrain = TerrainEngine(dted_path)
        self.state = GimbalState(dt, self.terrain)
        self.engine = ProtocolEngine()
        self.clients: List[asyncio.StreamWriter] = []
        
        # Video Side-Channel
        self.video_server: Optional[VideoServer] = None
        if tile_url:
            self.video_server = VideoServer(self.state, tile_url, host=host, port=video_port)

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        addr = writer.get_extra_info('peername')
        print(f"[*] New connection from {addr}")
        self.clients.append(writer)
        
        try:
            while True:
                header = await reader.read(4)
                if not header:
                    break
                
                sync0, sync1, p_id, length = struct.unpack(">BBBB", header)
                remaining = await reader.read(length + 2)
                full_packet = header + remaining
                
                packet = self.engine.parse(full_packet)
                if packet:
                    self.state.update_from_command(packet)
                
                # Echo behavior
                writer.write(full_packet)
                await writer.drain()
                
        except Exception as e:
            print(f"[!] Error with {addr}: {e}")
        finally:
            print(f"[-] Closing connection {addr}")
            if writer in self.clients:
                self.clients.remove(writer)
            writer.close()
            await writer.wait_closed()

    async def simulation_loop(self):
        """The 'Heartbeat' of the simulator: physics and telemetry."""
        while True:
            await asyncio.sleep(self.dt)
            
            # 1. Advance Physics
            self.state.step()
            
            # 2. Broadcast Telemetry to all clients
            if self.clients:
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
                        
                for writer in list(self.clients):
                    try:
                        for p in packets:
                            writer.write(p)
                        await writer.drain()
                    except Exception:
                        if writer in self.clients:
                            self.clients.remove(writer)

    async def run(self):
        server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"[+] Orion Simulator (Enhanced) active on {self.host}:{self.port}")
        print(f"[+] Physics Rate: {1/self.dt:.1f}Hz | Telemetry Rate: {1/self.dt:.1f}Hz")
        
        tasks = [server.serve_forever(), self.simulation_loop()]
        
        if self.video_server:
            print(f"[+] Video Stream (MJPEG) active on {self.host}:{self.video_server.port}")
            tasks.append(self.video_server.start())
        
        async with server:
            await asyncio.gather(*tasks)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--dted-path", type=str, default=None, help="Path to DTED folder for terrain simulation")
    parser.add_argument("--tile-url", type=str, default=None, help="XYZ tile URL template (e.g. 'https://{z}/{x}/{y}.png')")
    parser.add_argument("--video-port", type=int, default=5001, help="Port for the MJPEG video stream")
    args = parser.parse_args()

    server = OrionServer(
        host=args.host, 
        port=args.port, 
        dt=args.dt, 
        dted_path=args.dted_path,
        tile_url=args.tile_url,
        video_port=args.video_port
    )
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\n[!] Simulator shut down.")
