import struct
from typing import Dict, Optional
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.engine.physics import PhysicsEngine
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.faults import FaultEngine

class GimbalState:
    def __init__(self, dt: float = 0.1, terrain_engine: Optional[TerrainEngine] = None, initialized: bool = True):
        self.dt = dt
        self.physics = PhysicsEngine(dt)
        self.terrain = terrain_engine
        self.faults = FaultEngine()
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.initialized = initialized
        self.camera_id = 0
        self.laser_power = 0.0  # 0.0 to 1.0
        self.is_faulty = False
        
        # Camera State (Single KTnC Camera Setup)
        self.camera_zoom = 1.0
        self.camera_focus = 0.0
        self.camera_ready = True
        self.cameras = [
            {
                "type": 1,  # OrionCameraType_t.CAMERA_TYPE_VISIBLE
                "proto": 7,  # OrionCameraProtocol_t.CAMERA_PROTO_KTNC
                "min_focal": 4.3,
                "max_focal": 129.0,
                "pixel_pitch": 0.00225,
                "width": 1920,
                "height": 1080,
                "align_min": (0, 0),
                "align_max": (0, 0),
            }
        ]

        # OrionKtnc Specific Settings
        self.ktnc_index = 0
        self.ktnc_integration_time = -1
        self.ktnc_aperture = -1.0
        self.ktnc_sharpness = 8
        self.ktnc_vertical_flip = 0
        self.ktnc_exposure_comp = 7
        self.ktnc_contrast = 8
        self.ktnc_saturation = 14
        self.ktnc_night_mode = 0
        self.ktnc_has_max_exposure = 1
        self.ktnc_max_exposure = 0.008
        self.ktnc_version_major = 1
        self.ktnc_version_minor = 0
        self.ktnc_version_patch = 0

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

        # Video & Tracking State
        self.video_resolution_width = 1920
        self.video_resolution_height = 1080
        self.video_fps = 30
        self.tracking_target_id = 0
        self.tracking_mode = 0  # 0: None, 1: Object, 2: Point

        # Lifecycle & Diagnostics
        self.uptime = 0.0
        self.error_count = 0
        self.fault_count = 0

    def update_from_command(self, packet: OrionPacket) -> Optional[OrionPacket]:
        if packet.packet_id == OrionPktType.INITIALIZE:
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.RESET:
            self.__init__(dt=self.dt, terrain_engine=self.terrain)
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.STARTUP_CMD:
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.CMD:
            if len(packet.data) >= 8:
                pan, tilt = struct.unpack(">ff", packet.data[:8])
                self.target_pan = max(min(pan, self.pan_limit), -self.pan_limit)
                self.target_tilt = max(min(tilt, self.tilt_limit), -self.tilt_limit)
            elif len(packet.data) >= 4:
                pan_raw, tilt_raw = struct.unpack(">hh", packet.data[:4])
                pan, tilt = pan_raw / 1000.0, tilt_raw / 1000.0
                self.target_pan = max(min(pan, self.pan_limit), -self.pan_limit)
                self.target_tilt = max(min(tilt, self.tilt_limit), -self.tilt_limit)
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]
                self.camera_ready = False
            
        elif packet.packet_id == OrionPktType.CAMERA_CMD:
            if len(packet.data) >= 8:
                self.camera_zoom, self.camera_focus = struct.unpack(">ff", packet.data[:8])
                self.camera_ready = True
        
        elif packet.packet_id == OrionPktType.LASER_CMD:
            if len(packet.data) >= 4:
                self.laser_power = max(0.0, min(1.0, struct.unpack(">f", packet.data[:4])[0]))

        elif packet.packet_id == OrionPktType.UART_CONFIG:
            if len(packet.data) >= 4:
                self.baud_rate = struct.unpack(">I", packet.data[:4])[0]

        elif packet.packet_id == OrionPktType.LIMITS:
            if len(packet.data) >= 8:
                self.pan_limit, self.tilt_limit = struct.unpack(">ff", packet.data[:8])

        elif packet.packet_id == OrionPktType.GPS_DATA:
            if not self.is_faulty and len(packet.data) >= 12:
                self.gps_lat, self.gps_lon, self.gps_alt = struct.unpack(">fff", packet.data[:12])

        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            if len(packet.data) >= 12:
                self.aircraft_heading, self.aircraft_roll, self.aircraft_pitch = struct.unpack(">fff", packet.data[:12])

        elif packet.packet_id == OrionPktType.VIDEO_OPTIONS:
            if len(packet.data) >= 5:
                self.video_resolution_width, self.video_resolution_height, self.video_fps = struct.unpack(">HHB", packet.data[:5])

        elif packet.packet_id == OrionPktType.TRACK_OPTIONS:
            if len(packet.data) >= 5:
                self.tracking_target_id, self.tracking_mode = struct.unpack(">IB", packet.data[:5])

        elif packet.packet_id == OrionPktType.CAMERAS:
            return self.get_cameras_packet()

        elif packet.packet_id == OrionPktType.KTNC_SETTINGS:
            if len(packet.data) >= 16:
                idx, it, ap, sh, vf, ec, ct, st = struct.unpack_from(">BhhBBBBB", packet.data, 0)
                bf = packet.data[10]
                me = struct.unpack_from(">H", packet.data, 11)[0]
                self.ktnc_index = idx
                self.ktnc_integration_time = it
                self.ktnc_aperture = ap / 10.0
                self.ktnc_sharpness = int(round(sh / 15.0))
                self.ktnc_vertical_flip = vf
                self.ktnc_exposure_comp = int(round(ec / 17.0))
                self.ktnc_contrast = int(round(ct / 7.0))
                self.ktnc_saturation = int(round(st / 7.0))
                self.ktnc_night_mode = (bf >> 7) & 1
                self.ktnc_has_max_exposure = (bf >> 5) & 1
                self.ktnc_max_exposure = me / 1000000.0
            return self.get_ktnc_settings_packet()
            
        return None

    def step(self):
        self.physics.step(self.target_pan, self.target_tilt)
        self.faults.apply_faults(self)
        if self.terrain and self.terrain.enabled:
            terrain_alt = self.terrain.get_elevation(self.gps_lat, self.gps_lon)
            self.gps_alt = terrain_alt
        self.uptime += self.dt

    def get_telemetry_packet(self) -> bytes:
        pos_data = struct.pack(">ff", self.physics.pan["pos"], self.physics.tilt["pos"])
        return OrionPacket(OrionPktType.POSITIONS, pos_data).encode()

    def get_laser_state_packet(self) -> bytes:
        data = struct.pack(">f", self.laser_power)
        return OrionPacket(OrionPktType.LASER_STATES, data).encode()

    def get_camera_state_packet(self) -> bytes:
        data = struct.pack(">ffB", self.camera_zoom, self.camera_focus, 1 if self.camera_ready else 0)
        return OrionPacket(OrionPktType.CAMERA_STATE, data).encode()

    def get_sensor_data_packet(self) -> bytes:
        sensor_data = struct.pack(">fff", 0.0, 0.0, 1.0)
        return OrionPacket(OrionPktType.SENSOR_DATA, sensor_data).encode()

    def get_diagnostics_packet(self) -> bytes:
        data = struct.pack(">fII", self.uptime, self.error_count, self.fault_count)
        return OrionPacket(OrionPktType.DIAGNOSTICS, data).encode()

    def get_video_options_packet(self) -> bytes:
        data = struct.pack(">HHB", self.video_resolution_width, self.video_resolution_height, self.video_fps)
        return OrionPacket(OrionPktType.VIDEO_OPTIONS, data).encode()

    def get_tracking_options_packet(self) -> bytes:
        data = struct.pack(">IB", self.tracking_target_id, self.tracking_mode)
        return OrionPacket(OrionPktType.TRACK_OPTIONS, data).encode()

    def get_cameras_packet(self) -> bytes:
        num_cams = len(self.cameras)
        data = bytearray(struct.pack(">BBBB", num_cams, 0, 0, 0))
        for cam in self.cameras:
            data.extend(struct.pack(
                ">BBIIHHHhhhh",
                cam["type"],
                cam["proto"],
                int(round(cam["min_focal"] * 1000)),
                int(round(cam["max_focal"] * 1000)),
                int(round(cam["pixel_pitch"] * 1000000)),
                cam["width"],
                cam["height"],
                cam.get("align_min", (0, 0))[0],
                cam.get("align_min", (0, 0))[1],
                cam.get("align_max", (0, 0))[0],
                cam.get("align_max", (0, 0))[1],
            ))
        return OrionPacket(OrionPktType.CAMERAS, bytes(data)).encode()

    def get_ktnc_settings_packet(self) -> bytes:
        bitfield = ((self.ktnc_night_mode & 1) << 7) | ((self.ktnc_has_max_exposure & 1) << 5)
        aperture_encoded = int(round(self.ktnc_aperture * 10.0))
        data = struct.pack(
            ">BhhBBBBBBHBBB",
            self.ktnc_index,
            self.ktnc_integration_time,
            aperture_encoded,
            int(round(self.ktnc_sharpness * 15.0)),
            self.ktnc_vertical_flip,
            int(round(self.ktnc_exposure_comp * 17.0)),
            int(round(self.ktnc_contrast * 7.0)),
            int(round(self.ktnc_saturation * 7.0)),
            bitfield,
            int(round(self.ktnc_max_exposure * 1000000.0)),
            self.ktnc_version_major,
            self.ktnc_version_minor,
            self.ktnc_version_patch,
        )
        return OrionPacket(OrionPktType.KTNC_SETTINGS, data).encode()

    def get_faults_packet(self) -> bytes:
        fault_ids = self.faults.get_active_fault_ids()
        if not fault_ids:
            return OrionPacket(OrionPktType.FAULTS, struct.pack(">B", 0)).encode()
        data = struct.pack(f">B{len(fault_ids)}B", len(fault_ids), *fault_ids)
        return OrionPacket(OrionPktType.FAULTS, data).encode()
