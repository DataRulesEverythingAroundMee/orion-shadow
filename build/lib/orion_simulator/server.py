
import asyncio
import struct
from dataclasses import dataclass
from typing import Dict, Any, List

@dataclass
class OrionPacket:
    packet_id: int
    data: bytes

    def encode(self) -> bytes:
        sync0 = 0xD0
        sync1 = 0x0D
        length = len(self.data)
        
        # Build payload: Sync0, Sync1, ID, Length, Data
        payload = struct.pack(">BBBB", sync0, sync1, self.packet_id, length) + self.data
        
        # Fletcher-16 Checksum (modified mod 251)
        # Initial value 1
        a, b = 1, 1
        for byte in payload:
            a = (a + byte) % 251
            b = (b + a) % 251
            
        # Packet: Payload + Checksum (MSB, LSB)
        # The checksum is 16-bit.
        checksum = (b << 8) | a
        return payload + struct.pack(">H", checksum)

class ProtocolEngine:
    def __init__(self):
        self.packet_id_map = {
            0x00: "ORION_PKT_INITIALIZE",
            0x01: "ORION_PKT_CMD",
            0x0A: "ORION_PKT_POSITIONS",
            0x63: "ORION_PKT_CAMERAS",
            0xD0: "ORION_PKT_SENSOR_DATA"
        }

    def parse(self, raw_data: bytes) -> OrionPacket:
        # Simplified parser for simulation purposes
        if len(raw_data) < 6:
            raise ValueError("Packet too short")
            
        sync0, sync1, p_id, length = struct.unpack(">BBBB", raw_data[:4])
        if sync0 != 0xD0 or sync1 != 0x0D:
            raise ValueError("Invalid Sync bytes")
            
        data = raw_data[4:4+length]
        # Checksum verification would go here in a real implementation
        return OrionPacket(p_id, data)

class GimbalState:
    def __init__(self):
        self.pan = 0.0
        self.tilt = 0.0
        self.initialized = False
        self.camera_state = 0x00 # 0 for idle/off
        
    def update_from_command(self, packet: OrionPacket):
        if packet.packet_id == 0x01: # CMD
            # Expecting pan/tilt in some format (float32 x2)
            if len(packet.data) >= 8:
                self.pan, self.tilt = struct.unpack(">ff", packet.data[:8])
        elif packet.packet_id == 0x00: # INITIALIZE
            self.initialized = True

    def get_telemetry_packet(self) -> bytes:
        # Generate ORION_PKT_POSITIONS (0x0A)
        # Data: pan (f32), tilt (f32)
        data = struct.pack(">ff", self.pan, self.tilt)
        return OrionPacket(0x0A, data).encode()

class OrionServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.state = GimbalState()
        self.engine = ProtocolEngine()
        self.clients = set()

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        addr = writer.get_extra_info('peername')
        print(f"New connection from {addr}")
        self.clients.add(writer)
        
        try:
            while True:
                # Reading the packet header first to know the length
                header = await reader.read(4)
                if not header:
                    break
                
                sync0, sync1, p_id, length = struct.unpack(">BBBB", header)
                # Read the rest: Data + 2 bytes checksum
                remaining = await reader.read(length + 2)
                full_packet = header + remaining
                
                # Parse and process
                packet = self.engine.parse(full_packet)
                print(f"Received Packet ID: 0x{packet.packet_id:02X}")
                self.state.update_from_command(packet)
                
                # Echo behavior (mimicking hardware)
                writer.write(full_packet)
                await writer.drain()
                
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
        finally:
            print(f"Closing connection {addr}")
            self.clients.remove(writer)
            writer.close()
            await writer.wait_closed()

    async def telemetry_loop(self):
        """Periodically broadcast telemetry to all clients."""
        while True:
            await asyncio.sleep(0.1) # 10Hz
            if self.clients:
                packet = self.state.get_telemetry_packet()
                for writer in list(self.clients):
                    try:
                        writer.write(packet)
                        await writer.drain()
                    except Exception:
                        self.clients.remove(writer)

    async def run(self):
        server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"Orion Simulator running on {self.host}:{self.port}")
        
        async with server:
            await asyncio.gather(server.serve_forever(), self.telemetry_loop())

if __name__ == "__main__":
    server = OrionServer()
    asyncio.run(server.run())
