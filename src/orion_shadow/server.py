import asyncio
import struct
import math
import csv
import os
import numpy as np
from dataclasses import dataclass
from typing import Dict, Any, List, Optional

# --- Constants & Enumerations ---

ORION_SYNC0 = 0xD0
ORION_SYNC1 = 0x0D

class OrionPktType:
    INITIALIZE = 0x00
    CMD = 0x01
    UART_CONFIG = 0x02
    LASER_CMD = 0x03
    RESET = 0x04
    LASER_STATES = 0x06
    STARTUP_CMD = 0x07
    POSITIONS = 0x0A
    LIMITS = 0x22
    DIAGNOSTICS = 0x41
    FAULTS = 0x42
    CAMERA_SWITCH = 0x60
    CAMERA_STATE = 0x61
    CAMERAS = 0x63
    VIDEO_OPTIONS = 0x70
    TRACK_OPTIONS = 0x71
    SENSOR_DATA = 0xD0
    GPS_DATA = 0xD1
    EXT_HEADING_DATA = 0xD2

# --- Core Protocol ---

@dataclass
class OrionPacket:
    packet_id: int
    data: bytes

    def encode(self) -> bytes:
        length = len(self.data)
        payload = struct.pack(">BBBB", ORION_SYNC0, ORION_SYNC1, self.packet_id, length) + self.data
        
        # Fletcher-16 Checksum (modified mod 251)
        a, b = 1, 1
        for byte in payload:
            a = (a + byte) % 251
            b = (b + a) % 251
            
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
        if len(raw_data) < 6:
            raise ValueError("Packet too short")
            
        sync0, sync1, p_id, length = struct.unpack(">BBBB", raw_data[:4])
        if sync0 != ORION_SYNC0 or sync1 != ORION_SYNC1:
            raise ValueError("Invalid Sync bytes")
            
        data = raw_data[4:4+length]
        return OrionPacket(p_id, data)

# --- Physics Engine ---

class PhysicsEngine:
    """Simulates gimbal dynamics: inertia, velocity, and acceleration."""
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        # Current state: [pos, vel, acc]
        self.pan = {"pos": 0.0, "vel": 0.0, "acc": 0.0}
        self.tilt = {"pos": 0.0, "vel": 0.0, "acc": 0.0}
        
        # Physical constants (approximating a heavy gimbal)
        self.max_vel = 60.0  # deg/s
        self.max_acc = 100.0 # deg/s^2
        self.damping = 0.95  # simplistic friction/damping

    def step(self, target_pan: float, target_tilt: float):
        """Integrates physics one timestep forward."""
        self._update_axis(self.pan, target_pan)
        self._update_axis(self.tilt, target_tilt)

    def _update_axis(self, axis: Dict[str, float], target: float):
        # Error to target
        error = target - axis["pos"]
        
        # Simple Proportional control for acceleration
        desired_acc = error * 10.0 
        
        # Clamp acceleration
        desired_acc = max(min(desired_acc, self.max_acc), -self.max_acc)
        
        # Update acceleration
        axis["acc"] = desired_acc
        
        # Update velocity: v = v + a*dt
        axis["vel"] += axis["acc"] * self.dt
        
        # Clamp velocity
        axis["vel"] = max(min(axis["vel"], self.max_vel), -self.max_vel)
        
        # Update position: p = p + v*dt
        axis["pos"] += axis["vel"] * self.dt
        
        # Apply damping
        axis["vel"] *= self.damping

# --- Terrain Engine ---

class TerrainEngine:
    """Handles elevation queries from binary DTED data (.dt0, .dt1, .dt2)."""
    # DTED Header Format (Simplified for Simulation)
    # 0-3: Magic 'DTED'
    # 4-7: Rows (uint32)
    # 8-11: Cols (uint32)
    # 12-19: Lat_min (float64)
    # 20-27: Lat_max (float64)
    # 28-35: Lon_min (float64)
    # 36-43: Lon_max (float64)
    # 44-127: Padding
    HEADER_SIZE = 128
    MAGIC = b'DTED'

    def __init__(self, dted_path: Optional[str] = None):
        self.enabled = dted_path is not None
        self.dted_path = dted_path
        self.rows = 0
        self.cols = 0
        self.lat_min = 0.0
        self.lat_max = 0.0
        self.lon_min = 0.0
        self.lon_max = 0.0
        self.grid = None
        
        if self.enabled:
            print(f"[*] TerrainEngine enabled with path: {self.dted_path}")
            self._load_dted()
        else:
            print("[*] TerrainEngine disabled (no DTED path provided)")

    def _load_dted(self):
        if not os.path.exists(self.dted_path):
            print(f"[!] DTED file not found: {self.dted_path}")
            return

        try:
            with open(self.dted_path, 'rb') as f:
                header = f.read(self.HEADER_SIZE)
                if len(header) < self.HEADER_SIZE or header[:4] != self.MAGIC:
                    raise ValueError("Invalid DTED magic header")
                
                self.rows, self.cols = struct.unpack(">II", header[4:12])
                self.lat_min, self.lat_max = struct.unpack(">dd", header[12:28])
                self.lon_min, self.lon_max = struct.unpack(">dd", header[28:44])
                
                # Read elevation data (uint16)
                data_bytes = f.read()
                self.grid = np.frombuffer(data_bytes, dtype='>u2').reshape((self.rows, self.cols))
                print(f"[*] Loaded DTED grid: {self.rows}x{self.cols} ({len(data_bytes)} bytes)")
        except Exception as e:
            print(f"[!] Failed to parse DTED file: {e}")

    def get_elevation(self, lat: float, lon: float) -> float:
        if not self.enabled or self.grid is None:
            return 0.0
        
        # Clamp input to bounds
        lat = max(self.lat_min, min(self.lat_max, lat))
        lon = max(self.lon_min, min(self.lon_max, lon))

        # Map Lat/Lon to Grid index
        # Row index (Lat)
        row_frac = (lat - self.lat_min) / (self.lat_max - self.lat_min)
        # Col index (Lon)
        col_frac = (lon - self.lon_min) / (self.lon_max - self.lon_min)
        
        row = row_frac * (self.rows - 1)
        col = col_frac * (self.cols - 1)

        # Bilinear Interpolation
        r0, c0 = int(math.floor(row)), int(math.floor(col))
        r1, c1 = min(r0 + 1, self.rows - 1), min(c0 + 1, self.cols - 1)
        
        dr = row - r0
        dc = col - c0
        
        v00 = self.grid[r0, c0]
        v01 = self.grid[r0, c1]
        v10 = self.grid[r1, c0]
        v11 = self.grid[r1, c1]
        
        # Interpolate
        res = (1-dr)*(1-dc)*v00 + (1-dr)*dc*v01 + dr*(1-dc)*v10 + dr*dc*v11
        return float(res)

# --- Gimbal State & Logic ---

class GimbalState:
    def __init__(self, dt: float = 0.1, terrain_engine: Optional[TerrainEngine] = None):
        self.physics = PhysicsEngine(dt)
        self.terrain = terrain_engine
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.initialized = False
        self.camera_id = 0
        self.is_faulty = False
        # Navigation State
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.aircraft_heading = 0.0
        self.aircraft_roll = 0.0
        self.aircraft_pitch = 0.0

    def update_from_command(self, packet: OrionPacket):
        if packet.packet_id == OrionPktType.INITIALIZE:
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.CMD:
            # Assuming CMD data is pan (f32), tilt (f32)
            if len(packet.data) >= 8:
                self.target_pan, self.target_tilt = struct.unpack(">ff", packet.data[:8])
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]

        elif packet.packet_id == OrionPktType.GPS_DATA:
            # Expecting Lat, Lon, Alt (3 x float32)
            if len(packet.data) >= 12:
                self.gps_lat, self.gps_lon, self.gps_alt = struct.unpack(">fff", packet.data[:12])

        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            # Expecting Heading, Roll, Pitch (3 x float32)
            if len(packet.data) >= 12:
                self.aircraft_heading, self.aircraft_roll, self.aircraft_pitch = struct.unpack(">fff", packet.data[:12])

    def step(self):
        """Advance the simulation."""
        self.physics.step(self.target_pan, self.target_tilt)
        
        # If terrain engine is enabled, update altitude from DTED
        if self.terrain and self.terrain.enabled:
            terrain_alt = self.terrain.get_elevation(self.gps_lat, self.gps_lon)
            self.gps_alt = terrain_alt

    def get_telemetry_packet(self) -> bytes:
        # ORION_PKT_POSITIONS (0x0A): pan (f32), tilt (f32)
        pos_data = struct.pack(">ff", self.physics.pan["pos"], self.physics.tilt["pos"])
        return OrionPacket(OrionPktType.POSITIONS, pos_data).encode()

    def get_sensor_data_packet(self) -> bytes:
        # ORION_PKT_SENSOR_DATA (0xD0): simplified dummy data
        # In real hardware this would be IMU/Gyro
        sensor_data = struct.pack(">fff", 0.0, 0.0, 1.0) # Placeholder
        return OrionPacket(OrionPktType.SENSOR_DATA, sensor_data).encode()

# --- Server Implementation ---

class OrionServer:
    def __init__(self, host='0.0.0.0', port=5000, dt=0.1, dted_path: Optional[str] = None):
        self.host = host
        self.port = port
        self.dt = dt
        self.terrain = TerrainEngine(dted_path)
        self.state = GimbalState(dt, self.terrain)
        self.engine = ProtocolEngine()
        self.clients: List[asyncio.StreamWriter] = []

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
                # Combined burst of telemetry packets
                packets = [
                    self.state.get_telemetry_packet(),
                    self.state.get_sensor_data_packet()
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
        
        async with server:
            await asyncio.gather(server.serve_forever(), self.simulation_loop())

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--dted-path", type=str, default=None, help="Path to DTED folder for terrain simulation")
    args = parser.parse_args()

    server = OrionServer(host=args.host, port=args.port, dt=args.dt, dted_path=args.dted_path)
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\n[!] Simulator shut down.")
