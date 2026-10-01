import struct
import math
from typing import Dict, Optional
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.engine.physics import PhysicsEngine
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.faults import FaultEngine

class GimbalState:
    def __init__(self, dt: float = 0.1, terrain_engine: Optional[TerrainEngine] = None, initialized: bool = True,
                 lat: float = 0.0, lon: float = 0.0, alt: float = 0.0,
                 pan: float = 0.0, tilt: Optional[float] = None, heading: float = 0.0,
                 speed: float = 0.0):
        self.dt = dt
        self.initial_lat = float(lat)
        self.initial_lon = float(lon)
        self.initial_alt = float(alt)
        self.initial_pan = float(pan)
        self.initial_heading = float(heading)
        self.initial_speed = float(speed)

        # Default tilt to -20.0 if alt > 0 or non-zero lat/lon and tilt not specified, else 0.0
        if tilt is None:
            effective_tilt = -20.0 if (self.initial_alt > 0.0 or self.initial_lat != 0.0 or self.initial_lon != 0.0) else 0.0
        else:
            effective_tilt = float(tilt)
        self.initial_tilt = effective_tilt

        self.physics = PhysicsEngine(dt, initial_pan=self.initial_pan, initial_tilt=self.initial_tilt)
        self.terrain = terrain_engine
        self.faults = FaultEngine()
        self.target_pan = self.initial_pan
        self.target_tilt = self.initial_tilt
        self.initialized = initialized
        self.camera_id = 0
        self.laser_power = 0.0  # 0.0 to 1.0
        self.is_faulty = False
        
        # Camera State (Trillium HD40-XV Single Visible Camera Setup)
        self.camera_zoom = 1.0
        self.camera_focus = 0.0
        self.camera_ready = True
        self.min_zoom = 1.0
        self.max_optical_zoom = 30.0         # 30x optical zoom
        self.max_total_zoom = 112.0          # 112x total zoom (30x optical + 3.73x digital)
        self.min_hfov_deg = 0.4              # 0.4° at max total zoom (112x)
        self.max_hfov_deg = 47.7             # 47.7° at wide (1.0x)
        self.optical_tele_hfov_deg = 1.8     # 1.8° at optical telephoto (30x)
        self.cameras = [
            {
                "name": "EO Visible",
                "type": 1,  # OrionCameraType_t.CAMERA_TYPE_VISIBLE
                "proto": 7,  # OrionCameraProtocol_t.CAMERA_PROTO_KTNC
                "min_focal": 4.3,
                "max_focal": 129.0,
                "max_optical_zoom": 30.0,
                "max_total_zoom": 112.0,
                "pixel_pitch": 0.00297,
                "width": 1280,
                "height": 720,
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

        # Trillium HD40-XV Gimbal Limits & Physical Specs
        self.model_name = "Trillium HD40-XV"
        self.gimbal_weight_g = 840.0         # 840g weight
        self.dimensions_mm = (98.0, 151.0)   # 98mm diameter x 151mm height
        self.input_voltage_v = 24.0          # 24VDC regulated input
        self.power_avg_w = 15.0              # 15W average consumption
        self.power_peak_w = 75.0             # 75W peak consumption
        self.encoder_resolution_deg = 0.02   # 0.02° encoder resolution
        self.stabilization_bandwidth_hz = 60.0

        # Motion & Range Limits
        self.baud_rate = 115200
        self.pan_continuous = True
        self.pan_min = -180.0                # 360° continuous pan (-180° to +180°)
        self.pan_max = 180.0
        self.tilt_min = -80.0                # -80° to +28° tilt range
        self.tilt_max = 28.0
        self.pan_limit = 360.0
        self.tilt_limit = 28.0
        self.max_slew_rate = 60.0            # 60.0 deg/s max slew rate
        self.max_acceleration = 100.0        # 100.0 deg/s^2 max acceleration

        # Navigation State
        self.gps_lat = self.initial_lat
        self.gps_lon = self.initial_lon
        self.gps_alt = self.initial_alt
        self.terrain_alt = 0.0
        self.aircraft_heading = self.initial_heading
        self.aircraft_roll = 0.0
        self.aircraft_pitch = 0.0
        self.aircraft_speed = self.initial_speed  # Ground speed in knots
        self.gps_received = False
        self._last_gps_time: Optional[float] = None
        self._last_gps_pos: Optional[float, float] = None

        # Video & Tracking State
        self.video_resolution_width = 1280
        self.video_resolution_height = 720
        self.video_fps = 30
        self.tracking_target_id = 0
        self.tracking_mode = 0  # 0: None, 1: Object, 2: Point

        # Lifecycle & Diagnostics
        self.uptime = 0.0
        self.error_count = 0
        self.fault_count = 0

    @property
    def current_pan(self) -> float:
        if hasattr(self, 'physics') and isinstance(self.physics.pan, dict) and 'pos' in self.physics.pan:
            return float(self.physics.pan['pos'])
        return float(getattr(self, 'target_pan', 0.0))

    @property
    def current_tilt(self) -> float:
        if hasattr(self, 'physics') and isinstance(self.physics.tilt, dict) and 'pos' in self.physics.tilt:
            return float(self.physics.tilt['pos'])
        return float(getattr(self, 'target_tilt', 0.0))

    def update_from_command(self, packet: OrionPacket) -> Optional[OrionPacket]:
        if packet.packet_id == OrionPktType.INITIALIZE:
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.RESET:
            self.__init__(dt=self.dt, terrain_engine=self.terrain,
                          lat=self.initial_lat, lon=self.initial_lon, alt=self.initial_alt,
                          pan=self.initial_pan, tilt=self.initial_tilt, heading=self.initial_heading,
                          speed=self.initial_speed)
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.STARTUP_CMD:
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.CMD:
            if len(packet.data) == 8:
                pan, tilt = struct.unpack(">ff", packet.data[:8])
                self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
            elif len(packet.data) >= 4:
                pan_raw, tilt_raw = struct.unpack_from(">hh", packet.data, 0)
                # Mode byte is at offset 4 for standard OrionCmd_t and OrionCmdExtended_t
                if len(packet.data) >= 5:
                    mode = packet.data[4]
                else:
                    # Legacy 4-byte payload without mode byte (treat as position mode)
                    mode = 0x50

                # Rate modes: ORION_MODE_RATE (0x10), ORION_MODE_GEO_RATE (0x11), ORION_MODE_SCENE (0x30)
                if mode in (0x10, 0x11, 0x30):
                    pan_rate = math.degrees(pan_raw / 1000.0)   # deg/s
                    tilt_rate = math.degrees(tilt_raw / 1000.0) # deg/s

                    impulse_time = 0.0
                    if len(packet.data) >= 7:
                        impulse_time = packet.data[6] / 10.0

                    dt = impulse_time if impulse_time > 0.0 else self.dt

                    if abs(pan_rate) > 1e-4:
                        new_pan = self.target_pan + pan_rate * dt
                        self.target_pan = (new_pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(new_pan, self.pan_max), self.pan_min)

                    if abs(tilt_rate) > 1e-4:
                        new_tilt = self.target_tilt + tilt_rate * dt
                        self.target_tilt = max(min(new_tilt, self.tilt_max), self.tilt_min)

                # Position modes: ORION_MODE_POSITION (0x50), ORION_MODE_POSITION_NO_LIMITS (0x51)
                elif mode in (0x50, 0x51):
                    pan, tilt = math.degrees(pan_raw / 1000.0), math.degrees(tilt_raw / 1000.0)
                    self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                    self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)

                elif mode == 0x71:  # ORION_MODE_DOWN
                    self.target_tilt = self.tilt_min

                # If extended command (len >= 10), return OrionCmdExtendedResponse
                if len(packet.data) >= 10:
                    return self.get_cmd_extended_response_packet(packet.data)
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]
                self.camera_ready = False
            
        elif packet.packet_id in (OrionPktType.CAMERA_CMD, OrionPktType.CAMERA_STATE):
            if len(packet.data) >= 2 and len(packet.data) < 8:
                zoom_raw = struct.unpack_from(">h", packet.data, 0)[0]
                zoom = zoom_raw / 100.0
                if zoom >= 1.0:
                    self.camera_zoom = min(zoom, self.max_total_zoom)
                if len(packet.data) >= 4:
                    focus_raw = struct.unpack_from(">h", packet.data, 2)[0]
                    if focus_raw != -1:
                        self.camera_focus = focus_raw / 10000.0
                if len(packet.data) >= 5:
                    b = packet.data[4]
                    keep_active = (b >> 7) & 1
                    cam_idx = b & 0x7F
                    if not keep_active:
                        self.camera_id = min(cam_idx, len(self.cameras) - 1) if self.cameras else cam_idx
                self.camera_ready = True
            elif len(packet.data) >= 8:
                zoom, focus = struct.unpack(">ff", packet.data[:8])
                if zoom >= 1.0:
                    self.camera_zoom = min(zoom, self.max_total_zoom)
                if focus >= 0.0:
                    self.camera_focus = focus
                self.camera_ready = True
            return self.get_camera_state_packet()
        
        elif packet.packet_id == OrionPktType.LASER_CMD:
            if len(packet.data) >= 4:
                self.laser_power = max(0.0, min(1.0, struct.unpack(">f", packet.data[:4])[0]))

        elif packet.packet_id == OrionPktType.LASER_STATES:
            return self.get_laser_state_packet()

        elif packet.packet_id == OrionPktType.UART_CONFIG:
            if len(packet.data) >= 4:
                self.baud_rate = struct.unpack(">I", packet.data[:4])[0]

        elif packet.packet_id == OrionPktType.LIMITS:
            if len(packet.data) == 8:
                self.pan_limit, self.tilt_limit = struct.unpack(">ff", packet.data[:8])
                self.pan_max = self.pan_limit
                self.pan_min = -self.pan_limit
                self.tilt_max = self.tilt_limit
                self.tilt_min = -self.tilt_limit
                self.pan_continuous = False
            return self.get_limits_packet()

        elif packet.packet_id == OrionPktType.GPS_DATA:
            if not self.is_faulty:
                self.gps_received = True
                new_lat, new_lon, new_alt = None, None, None
                explicit_speed = False
                if len(packet.data) >= 28:
                    raw_lat, raw_lon, raw_alt = struct.unpack_from(">iii", packet.data, 4)
                    vn, ve, vd = struct.unpack_from(">iii", packet.data, 16)
                    new_lat = raw_lat * 1e-7
                    new_lon = raw_lon * 1e-7
                    new_alt = raw_alt / 10000.0
                    spd_mps = math.hypot(vn / 1000.0, ve / 1000.0)
                    self.aircraft_speed = spd_mps * 1.94384
                    explicit_speed = True
                elif len(packet.data) >= 16:
                    f_lat, f_lon, f_alt, f_spd = struct.unpack_from(">ffff", packet.data, 0)
                    if -90.0 <= f_lat <= 90.0 and -180.0 <= f_lon <= 180.0 and (abs(f_lat) > 0.001 or abs(f_lon) > 0.001):
                        new_lat, new_lon, new_alt = f_lat, f_lon, f_alt
                        self.aircraft_speed = max(0.0, f_spd)
                        explicit_speed = True
                    else:
                        raw_lat, raw_lon, raw_alt = struct.unpack_from(">iii", packet.data, 4)
                        new_lat = raw_lat * 1e-7
                        new_lon = raw_lon * 1e-7
                        new_alt = raw_alt / 10000.0
                elif len(packet.data) >= 12:
                    new_lat, new_lon, new_alt = struct.unpack(">fff", packet.data[:12])

                if new_lat is not None:
                    # Estimate ground speed from GPS delta if speed wasn't explicitly provided
                    import time
                    now = time.time()
                    if not explicit_speed and self._last_gps_pos is not None and self._last_gps_time is not None:
                        dt = now - self._last_gps_time
                        if dt > 0.05:
                            dlat = (new_lat - self._last_gps_pos[0]) * 111320.0
                            dlon = (new_lon - self._last_gps_pos[1]) * 111320.0 * math.cos(math.radians(new_lat))
                            dist_m = math.hypot(dlat, dlon)
                            if dist_m > 0.1:
                                self.aircraft_speed = (dist_m / dt) * 1.94384
                    self._last_gps_pos = (new_lat, new_lon)
                    self._last_gps_time = now
                    self.gps_lat = new_lat
                    self.gps_lon = new_lon
                    self.gps_alt = new_alt

        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            if len(packet.data) >= 8 and len(packet.data) < 12:
                # Official Orion SDK OrionExtHeadingData packet (8 bytes: extHeading >h, noise >H, flags >H, pitch >h)
                raw_hdg = struct.unpack_from(">h", packet.data, 0)[0]
                raw_pitch = struct.unpack_from(">h", packet.data, 6)[0]
                self.aircraft_heading = math.degrees(raw_hdg / 10430.06004058) % 360.0
                self.aircraft_pitch = math.degrees(raw_pitch / 10430.06004058)
            elif len(packet.data) >= 12:
                self.aircraft_heading, self.aircraft_roll, self.aircraft_pitch = struct.unpack(">fff", packet.data[:12])

        elif packet.packet_id == OrionPktType.NETWORK_VIDEO:
            if len(packet.data) >= 6:
                self.video_dest_ip, self.video_dest_port = struct.unpack_from(">IH", packet.data, 0)
            return self.get_network_video_packet()

        elif packet.packet_id == OrionPktType.VIDEO_OPTIONS:
            if len(packet.data) >= 5 and len(packet.data) < 10:
                try:
                    self.video_resolution_width, self.video_resolution_height, self.video_fps = struct.unpack(">HHB", packet.data[:5])
                except Exception:
                    pass
            return self.get_video_options_packet()

        elif packet.packet_id == OrionPktType.TRACK_OPTIONS:
            if len(packet.data) >= 9:
                self.tracking_mode = packet.data[8]
            elif len(packet.data) >= 5:
                self.tracking_target_id, self.tracking_mode = struct.unpack(">IB", packet.data[:5])
            return self.get_tracking_options_packet()

        elif packet.packet_id == OrionPktType.DIAGNOSTICS:
            return self.get_diagnostics_packet()

        elif packet.packet_id == OrionPktType.FAULTS:
            return self.get_faults_packet()

        elif packet.packet_id == OrionPktType.SENSOR_DATA:
            return self.get_sensor_data_packet()

        elif packet.packet_id == OrionPktType.POSITIONS:
            return self.get_telemetry_packet()

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

        elif packet.packet_id == OrionPktType.GEOLOCATE_TELEMETRY_CORE:
            return self.get_geolocate_telemetry_core_packet()
            
        return None

    def step(self):
        self.physics.step(
            self.target_pan, 
            self.target_tilt, 
            continuous_pan=self.pan_continuous,
            tilt_min=self.tilt_min, 
            tilt_max=self.tilt_max
        )
        self.faults.apply_faults(self)
        if self.terrain and self.terrain.enabled:
            terrain_alt = self.terrain.get_elevation(self.gps_lat, self.gps_lon)
            self.terrain_alt = terrain_alt
            if self.gps_alt == 0.0:
                self.gps_alt = terrain_alt
        # Advance simulated aircraft position along heading if speed > 0 and no external GPS active
        if not self.gps_received and self.aircraft_speed > 0.0:
            speed_mps = self.aircraft_speed / 1.94384
            dist_m = speed_mps * self.dt
            hdg_rad = math.radians(self.aircraft_heading)
            self.gps_lat += (dist_m * math.cos(hdg_rad)) / 111320.0
            cos_lat = math.cos(math.radians(self.gps_lat))
            self.gps_lon += (dist_m * math.sin(hdg_rad)) / (111320.0 * (cos_lat if abs(cos_lat) > 1e-6 else 1.0))
        self.uptime += self.dt

    def get_telemetry_packet(self) -> bytes:
        # OrionPositions packet (Orion SDK 3.1.9, Packet ID: 10, min len 1, max len 49)
        # Stored presets: NumPositions=0 (1 byte payload)
        return OrionPacket(OrionPktType.POSITIONS, struct.pack(">B", 0)).encode()

    def get_laser_state_packet(self) -> bytes:
        # OrionLaserStates packet (Orion SDK 3.1.9, Packet ID: 6, min len 1, max len 19)
        # 1 installed pointer laser: Type=1, Flags: Enabled(15), Armed(14), Active(13), Temp=25C, WaitTimer=0
        if self.laser_power > 0:
            flags = (1 << 15) | (1 << 14) | (1 << 13)
        else:
            flags = 0
        data = struct.pack(">BBHBH", 1, 1, flags, 25, 0)
        return OrionPacket(OrionPktType.LASER_STATES, data).encode()

    def get_cmd_extended_response_packet(self, cmd_data: bytes) -> bytes:
        """Generates standard 28-byte OrionCmdExtendedResponse payload (Orion Public Protocol 1.4)."""
        pan_raw, tilt_raw = struct.unpack_from(">hh", cmd_data, 0)
        mode = cmd_data[4] if len(cmd_data) >= 5 else 0
        stabilized = cmd_data[5] if len(cmd_data) >= 6 else 0
        impulse_raw = cmd_data[6] if len(cmd_data) >= 7 else 0
        seq_num = struct.unpack_from(">H", cmd_data, 7)[0] if len(cmd_data) >= 9 else 0
        flags = cmd_data[9] if len(cmd_data) >= 10 else 0

        clevis_time = int(self.uptime * 1000) & 0xFFFFFF
        drops = 0
        gyro_rates = (0, 0, 0)

        pan_rad = math.radians(self.current_pan)
        tilt_rad = math.radians(self.current_tilt)
        scale_pos = 10430.06004058427
        enc_pan = max(-32768, min(32767, int(round(pan_rad * scale_pos))))
        enc_tilt = max(-32768, min(32767, int(round(tilt_rad * scale_pos))))

        vel_pan_rad = math.radians(self.physics.pan.get("vel", 0.0) if hasattr(self, 'physics') and isinstance(self.physics.pan, dict) else 0.0)
        vel_tilt_rad = math.radians(self.physics.tilt.get("vel", 0.0) if hasattr(self, 'physics') and isinstance(self.physics.tilt, dict) else 0.0)
        scale_vel = 5215.030020292134
        enc_rate_pan = max(-32768, min(32767, int(round(vel_pan_rad * scale_vel))))
        enc_rate_tilt = max(-32768, min(32767, int(round(vel_tilt_rad * scale_vel))))

        clevis_bytes = clevis_time.to_bytes(3, 'big')

        payload = (
            struct.pack(">hhBBBH", pan_raw, tilt_raw, mode, stabilized, impulse_raw, seq_num)
            + clevis_bytes
            + struct.pack(">BhhhhhhhB",
                          drops,
                          gyro_rates[0], gyro_rates[1], gyro_rates[2],
                          enc_pan, enc_tilt,
                          enc_rate_pan, enc_rate_tilt,
                          flags)
        )
        return OrionPacket(OrionPktType.CMD, payload).encode()

    def get_camera_state_packet(self) -> bytes:
        data = struct.pack(
            ">hhB",
            int(round(self.camera_zoom * 100.0)),
            int(round(self.camera_focus * 10000.0)),
            self.camera_id & 0x7F
        )
        return OrionPacket(OrionPktType.CAMERA_STATE, data).encode()

    def get_sensor_data_packet(self) -> bytes:
        # OrionSensorData packet (Orion SDK 3.1.9, Packet ID: 208, min len 22, max len 29)
        uptime_ms = int(self.uptime * 1000) & 0xFFFFFFFF
        dt_us = int(self.dt * 1000000) & 0xFFFF
        counter = (getattr(self, "_sensor_counter", 0) + 1) & 0xFF
        self._sensor_counter = counter
        baro_raw = int(round(101325.0 * 0.02))  # 1013.25 hPa
        oat_raw = int(round((15.0 + 273.15) * 100.0))  # 15°C
        gyro_temp_raw = int(round((25.0 + 273.15) * 100.0))  # 25°C
        accel_z = int(round(9.80665 * 1000.0))  # 1G in mg
        data = struct.pack(
            ">hhhhhhHHIHHHhB",
            0, 0, 0,
            0, 0, accel_z,
            baro_raw,
            oat_raw,
            uptime_ms,
            gyro_temp_raw,
            dt_us,
            dt_us,
            -1,
            counter
        )
        return OrionPacket(OrionPktType.SENSOR_DATA, data).encode()

    def get_diagnostics_packet(self) -> bytes:
        # OrionDiagnostics packet (Orion SDK 3.1.9, Packet ID: 65, min len 28, max len 33)
        v24 = int(round(self.input_voltage_v * 1000.0))
        v12 = int(round(12.0 * 1000.0))
        v3v3 = int(round(3.3 * 1000.0))
        i24 = int(round((self.power_avg_w / self.input_voltage_v) * 1000.0))
        data = struct.pack(
            ">HHHHHHbbbBHHHHHHbBHb",
            v24, v12, v3v3, i24, 0, 0,
            35, 35, 35, 0,
            0, 0, 0, 0, 0, 0,
            35, 0, 0, 35
        )
        return OrionPacket(OrionPktType.DIAGNOSTICS, data).encode()

    def get_video_options_packet(self) -> bytes:
        # VideoOptions packet (Orion SDK 3.1.9, Packet ID: 112, min len 4, max len 49)
        # Stabilized(7), ShowReticle(5) -> 0xA0
        data = struct.pack(">BBBBBIB", 0xA0, 0, 0, 0, 25, 0, 0)
        return OrionPacket(OrionPktType.VIDEO_OPTIONS, data).encode()

    def get_network_video_packet(self) -> bytes:
        # OrionNetworkVideo packet (Orion SDK 3.1.9, Packet ID: 98, min len 6, max len 15)
        dest_ip = getattr(self, "video_dest_ip", 0xEFFF0001)  # 239.255.0.1 default
        port = getattr(self, "video_dest_port", 5004)
        bitrate = getattr(self, "video_bitrate", 4000000)
        data = struct.pack(">IHIbBBBB", dest_ip, port, bitrate, 64, 0, 30, 0, 0)
        return OrionPacket(OrionPktType.NETWORK_VIDEO, data).encode()

    def get_tracking_options_packet(self) -> bytes:
        # TrackOptions packet (Orion SDK 3.1.9, Packet ID: 113, min len 9, max len 31)
        data = struct.pack(
            ">16BHBBffBH",
            0, 0, 0, 0, 0, 0, 0, 0, self.tracking_mode & 0xFF,
            0, 0, 0, 0, 100, 0, 0,
            0, 6, 3,
            0.0, 0.0,
            51, 0
        )
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
        # OrionFault packet (Orion SDK 3.1.9, Packet ID: 66, exact len 11)
        fault_ids = self.faults.get_active_fault_ids()
        if not fault_ids:
            data = struct.pack(">BBBII", 0, 0, 0, 0, 0)
        else:
            fault_type = fault_ids[0]
            data = struct.pack(">BBBII", fault_type, 3, 1, fault_type, 0)
        return OrionPacket(OrionPktType.FAULTS, data).encode()

    def get_limits_packet(self) -> bytes:
        min_pan_rad = math.radians(self.pan_min)
        min_tilt_rad = math.radians(self.tilt_min)
        max_pan_rad = math.radians(self.pan_max)
        max_tilt_rad = math.radians(self.tilt_max)
        max_vel_rad = math.radians(self.physics.max_vel)
        max_acc_rad = math.radians(self.physics.max_acc)

        data = bytearray()
        # MinPos [pan, tilt] (scaled 1000.0, >h)
        data.extend(struct.pack(">hh", int(round(min_pan_rad * 1000.0)), int(round(min_tilt_rad * 1000.0))))
        # MaxPos [pan, tilt] (scaled 1000.0, >h)
        data.extend(struct.pack(">hh", int(round(max_pan_rad * 1000.0)), int(round(max_tilt_rad * 1000.0))))
        # MaxVel [pan, tilt] (scaled 1000.0, >H)
        data.extend(struct.pack(">HH", int(round(max_vel_rad * 1000.0)), int(round(max_vel_rad * 1000.0))))
        # MaxAccel [pan, tilt] (scaled 1.0, >H)
        data.extend(struct.pack(">HH", int(round(max_acc_rad)), int(round(max_acc_rad))))
        # ContCur (scaled 1000.0, >H) - 15W / 24V = 0.625 A
        cont_cur = int(round((self.power_avg_w / self.input_voltage_v) * 1000.0))
        data.extend(struct.pack(">HH", cont_cur, cont_cur))
        # PeakCur (scaled 1000.0, >H) - 75W / 24V = 3.125 A
        peak_cur = int(round((self.power_peak_w / self.input_voltage_v) * 1000.0))
        data.extend(struct.pack(">HH", peak_cur, peak_cur))
        # PeakCurTime, InitCur
        data.extend(struct.pack(">HHHH", 0, 0, 0, 0))
        # MaxPower (unscaled Watts, >BB) - 75W peak power
        max_power = int(round(self.power_peak_w))
        data.extend(struct.pack(">BB", max_power, max_power))

        return OrionPacket(OrionPktType.LIMITS, bytes(data)).encode()

    def get_geolocate_telemetry_core_packet(self) -> bytes:
        uptime_ms = int(self.uptime * 1000)
        lat_rad = math.radians(self.gps_lat)
        lon_rad = math.radians(self.gps_lon)
        alt_m = float(self.gps_alt)
        pan_rad = math.radians(self.physics.pan["pos"])
        tilt_rad = math.radians(self.physics.tilt["pos"])

        # Mode: 1 if faulty, 0 if disabled, 16 if rate
        if self.is_faulty or bool(self.faults.active_faults):
            mode = 1
        elif not self.initialized:
            mode = 0
        else:
            mode = 16

        # Camera FOV calculation
        if self.cameras:
            cam = self.cameras[min(self.camera_id, len(self.cameras) - 1)]
            min_focal = cam.get("min_focal", 4.3)
            pixel_pitch = cam.get("pixel_pitch", 0.00297)
            width = cam.get("width", 1280)
            height = cam.get("height", 720)
            max_zoom = cam.get("max_total_zoom", self.max_total_zoom)
            zoom = max(1.0, min(self.camera_zoom, max_zoom))
            focal = min_focal * zoom
            hfov_rad = 2.0 * math.atan2(0.5 * (width * pixel_pitch), focal)
            vfov_rad = 2.0 * math.atan2(0.5 * (height * pixel_pitch), focal)
        else:
            width = 1280
            height = 720
            hfov_rad = math.radians(47.7)
            vfov_rad = math.radians(28.0)

        data = bytearray()
        data.extend(struct.pack(">IIHh", uptime_ms, 0, 0, 0))
        data.extend(struct.pack(">iii", int(round(lat_rad * 572957795.1308)), int(round(lon_rad * 572957795.1308)), int(round(alt_m * 10000.0))))
        data.extend(struct.pack(">hhh", 0, 0, 0))  # velNED
        data.extend(struct.pack(">hhhh", 32767, 0, 0, 0))  # gimbalQuat
        data.extend(struct.pack(">hh", int(round(pan_rad * 10430.06004058)), int(round(tilt_rad * 10430.06004058))))
        data.extend(struct.pack(">HH", int(round(hfov_rad * 10430.21919553)), int(round(vfov_rad * 10430.21919553))))
        data.extend(struct.pack(">hhh", 0, 0, 0))  # losECEF
        data.extend(struct.pack(">HH", width, height))
        data.extend(struct.pack(">BBBBB", mode, 0, 0, 0, 0))  # mode, pathProgress, stareTime, pathFrom, pathTo
        data.extend(struct.pack(">ii", 0, 0))  # imageShifts
        data.extend(struct.pack(">HB", 0, 0))  # imageShiftDeltaTime, imageShiftConfidence
        data.extend(struct.pack(">hh", 0, 0))  # outputShifts
        data.extend(struct.pack(">BBbbB", 0, 18, 0, 0, 0))  # rangeSource, leapSeconds, panAlignment, tiltAlignment, insRotationOption
        data.extend(struct.pack(">h", 0))  # imageRotation
        data.extend(struct.pack(">bB", self.camera_id, 0))  # cameraIndex, hasTrackData

        return OrionPacket(OrionPktType.GEOLOCATE_TELEMETRY_CORE, bytes(data)).encode()
