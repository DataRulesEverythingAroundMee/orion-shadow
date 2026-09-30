import struct
from typing import Dict, Optional
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.engine.physics import PhysicsEngine
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.faults import FaultEngine

class GimbalState:
    def __init__(self, dt: float = 0.1, terrain_engine: Optional[TerrainEngine] = None):
        self.physics = PhysicsEngine(dt)
        self.terrain = terrain_engine
        self.faults = FaultEngine()
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.initialized = False
        self.camera_id = 0
        self.laser_power = 0.0  # 0.0 to 1.0
        self.is_faulty = False
        
        # Camera State
        self.camera_zoom = 1.0
        self.camera_focus = 0.0
        self.camera_ready = True

        # Configuration & Limits
        self.baud_rate = 115200
        self.pan_limit = 45.0
        self.tilt_limit = 45.0

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
                pan, tilt = struct.unpack(">ff", packet.data[:8])
                # Apply limits
                self.target_pan = max(min(pan, self.pan_limit), -self.pan_limit)
                self.target_tilt = max(min(tilt, self.tilt_limit), -self.tilt_limit)
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]

        elif packet.packet_id == OrionPktType.LASER_CMD:
            # Expecting Laser Power (f32)
            if len(packet.data) >= 4:
                self.laser_power = max(0.0, min(1.0, struct.unpack(">f", packet.data[:4])[0]))

        elif packet.packet_id == OrionPktType.UART_CONFIG:
            # Expecting Baud Rate (u32)
            if len(packet.data) >= 4:
                self.baud_rate = struct.unpack(">I", packet.data[:4])[0]

        elif packet.packet_id == OrionPktType.LIMITS:
            # Expecting pan_limit (f32), tilt_limit (f32)
            if len(packet.data) >= 8:
                self.pan_limit, self.tilt_limit = struct.unpack(">ff", packet.data[:8])

        elif packet.packet_id == OrionPktType.GPS_DATA:
            # Expecting Lat, Lon, Alt (3 x float32)
            # If a sensor timeout fault is active, ignore incoming GPS data
            if not self.is_faulty and len(packet.data) >= 12:
                self.gps_lat, self.gps_lon, self.gps_alt = struct.unpack(">fff", packet.data[:12])

        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            # Expecting Heading, Roll, Pitch (3 x float32)
            if len(packet.data) >= 12:
                self.aircraft_heading, self.aircraft_roll, self.aircraft_pitch = struct.unpack(">fff", packet.data[:12])

    def step(self):
        """Advance the simulation."""
        self.physics.step(self.target_pan, self.target_tilt)
        
        # Apply injected faults
        self.faults.apply_faults(self)
        
        # If terrain engine is enabled, update altitude from DTED
        if self.terrain and self.terrain.enabled:
            terrain_alt = self.terrain.get_elevation(self.gps_lat, self.gps_lon)
            self.gps_alt = terrain_alt

    def get_telemetry_packet(self) -> bytes:
        # ORION_PKT_POSITIONS (0x0A): pan (f32), tilt (f32)
        pos_data = struct.pack(">ff", self.physics.pan["pos"], self.physics.tilt["pos"])
        return OrionPacket(OrionPktType.POSITIONS, pos_data).encode()

    def get_laser_state_packet(self) -> bytes:
        # ORION_PKT_LASER_STATES (0x06): laser_power (f32)
        data = struct.pack(">f", self.laser_power)
        return OrionPacket(OrionPktType.LASER_STATES, data).encode()

    def get_camera_state_packet(self) -> bytes:
        # ORION_PKT_CAMERA_STATE (0x61): zoom (f32), focus (f32), ready (u8)
        data = struct.pack(">ffB", self.camera_zoom, self.camera_focus, 1 if self.camera_ready else 0)
        return OrionPacket(OrionPktType.CAMERA_STATE, data).encode()

    def get_sensor_data_packet(self) -> bytes:
        # ORION_PKT_SENSOR_DATA (0xD0): simplified dummy data
        # In real hardware this would be IMU/Gyro
        sensor_data = struct.pack(">fff", 0.0, 0.0, 1.0) # Placeholder
        return OrionPacket(OrionPktType.SENSOR_DATA, sensor_data).encode()
