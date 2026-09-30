import struct
from typing import Dict, Optional
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.engine.physics import PhysicsEngine
from orion_shadow.engine.terrain import TerrainEngine

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
