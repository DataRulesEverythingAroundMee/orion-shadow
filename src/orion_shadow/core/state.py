import struct
import math
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
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.aircraft_heading = 0.0
        self.aircraft_roll = 0.0
        self.aircraft_pitch = 0.0

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
                self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
            elif len(packet.data) >= 4:
                pan_raw, tilt_raw = struct.unpack(">hh", packet.data[:4])
                pan, tilt = pan_raw / 1000.0, tilt_raw / 1000.0
                self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]
                self.camera_ready = False
            
        elif packet.packet_id == OrionPktType.CAMERA_CMD:
            if len(packet.data) >= 8:
                zoom, focus = struct.unpack(">ff", packet.data[:8])
                if zoom >= 1.0:
                    self.camera_zoom = min(zoom, self.max_total_zoom)
                if focus >= 0.0:
                    self.camera_focus = focus
                self.camera_ready = True

        elif packet.packet_id == OrionPktType.CAMERA_STATE:
            if len(packet.data) >= 5:
                zoom_raw, focus_raw = struct.unpack_from(">hh", packet.data, 0)
                zoom = zoom_raw / 100.0
                if zoom >= 1.0:
                    self.camera_zoom = min(zoom, self.max_total_zoom)
                if focus_raw != -1:
                    self.camera_focus = focus_raw / 10000.0
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

        elif packet.packet_id == OrionPktType.UART_CONFIG:
            if len(packet.data) >= 4:
                self.baud_rate = struct.unpack(">I", packet.data[:4])[0]

        elif packet.packet_id == OrionPktType.LIMITS:
            if len(packet.data) == 8:
                self.pan_limit, self.tilt_limit = struct.unpack(">ff", packet.data[:8])
            return self.get_limits_packet()

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
            self.gps_alt = terrain_alt
        self.uptime += self.dt

    def get_telemetry_packet(self) -> bytes:
        pos_data = struct.pack(">ff", self.physics.pan["pos"], self.physics.tilt["pos"])
        return OrionPacket(OrionPktType.POSITIONS, pos_data).encode()

    def get_laser_state_packet(self) -> bytes:
        data = struct.pack(">f", self.laser_power)
        return OrionPacket(OrionPktType.LASER_STATES, data).encode()

    def get_camera_state_packet(self) -> bytes:
        data = struct.pack(
            ">hhB",
            int(round(self.camera_zoom * 100.0)),
            int(round(self.camera_focus * 10000.0)),
            self.camera_id & 0x7F
        )
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
