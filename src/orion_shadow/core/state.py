import struct
import math
import logging
from typing import Dict, Optional, Tuple
from orion_shadow.core.protocol import OrionPacket, OrionPktType, OrionMode
from orion_shadow.engine.physics import PhysicsEngine
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.faults import FaultEngine

logger = logging.getLogger(__name__)

def _encode_fixed_string(s: str, length: int) -> bytes:
    """Helper to encode string to fixed-length null-padded ASCII bytes."""
    b = s.encode("ascii", errors="replace")[:length]
    return b.ljust(length, b"\x00")

class GimbalState:
    def __init__(self, dt: float = 0.1, terrain_engine: Optional[TerrainEngine] = None, initialized: bool = True,
                 lat: float = 0.0, lon: float = 0.0, alt: float = 0.0,
                 pan: float = 0.0, tilt: Optional[float] = None, heading: float = 0.0,
                 speed: float = 0.0, model: str = "HD40-XV"):
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
        self.mode = OrionMode.RATE if initialized else OrionMode.DISABLED
        self.camera_id = 0
        self.laser_power = 0.0  # 0.0 to 1.0
        self.is_faulty = False

        # Model Profile Configuration (HD40-XV Default Single-Sensor vs HD40-LV Dual-Sensor)
        self.model = "HD40-LV" if "LV" in model.upper() else "HD40-XV"
        if self.model == "HD40-LV":
            self.model_name = "Trillium HD40-LV"
            self.part_number = "HD40-LV-001"
            self.serial_number = 4000101
            self.hardware_id = 0x404C
            self.gimbal_weight_g = 920.0
            self.cameras = [
                {
                    "name": "EO Visible",
                    "type": 1,  # CAMERA_TYPE_VISIBLE
                    "proto": 7,  # CAMERA_PROTO_KTNC
                    "min_focal": 4.3,
                    "max_focal": 129.0,
                    "max_optical_zoom": 30.0,
                    "max_total_zoom": 112.0,
                    "pixel_pitch": 0.00297,
                    "width": 1280,
                    "height": 720,
                    "align_min": (0, 0),
                    "align_max": (0, 0),
                },
                {
                    "name": "LWIR Thermal",
                    "type": 2,  # CAMERA_TYPE_LWIR
                    "proto": 1,  # CAMERA_PROTO_FLIR_TAU / BOSON
                    "min_focal": 18.0,
                    "max_focal": 18.0,
                    "max_optical_zoom": 1.0,
                    "max_total_zoom": 4.0,  # 4x digital zoom
                    "pixel_pitch": 0.012,
                    "width": 640,
                    "height": 512,
                    "align_min": (0, 0),
                    "align_max": (0, 0),
                },
            ]
        else:
            self.model_name = "Trillium HD40-XV"
            self.part_number = "HD40-XV-001"
            self.serial_number = 4000100
            self.hardware_id = 0x4040
            self.gimbal_weight_g = 840.0
            self.cameras = [
                {
                    "name": "EO Visible",
                    "type": 1,  # CAMERA_TYPE_VISIBLE
                    "proto": 7,  # CAMERA_PROTO_KTNC
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

        # Geopoint Mode State (ORION_MODE_GEOPOINT: 0x60 / 96)
        self.geopoint_lat = 0.0
        self.geopoint_lon = 0.0
        self.geopoint_alt = 0.0
        self.geopoint_vel_ned = [0.0, 0.0, 0.0]
        self.geopoint_joystick_range = 0.0
        self.geopoint_options = 0
        
        # Additional Operational Mode State
        self.track_box = (0.0, 0.0)
        self.ffc_active = False
        self.gyro_calibrated = False
        self.calibration_active = False
        self.path_points = []
        self.path_progress = 0.0
        self.stare_time = 0.0
        self.path_from = 0
        self.path_to = 0

        # Camera Optical Zoom State
        self.camera_zoom = 1.0
        self.camera_focus = 0.0
        self.camera_ready = True
        self.min_zoom = 1.0
        self.max_optical_zoom = 30.0         # 30x optical zoom
        self.max_total_zoom = 112.0          # 112x total zoom (30x optical + 3.73x digital)
        self.min_hfov_deg = 0.4              # 0.4° at max total zoom (112x)
        self.max_hfov_deg = 47.7             # 47.7° at wide (1.0x)
        self.optical_tele_hfov_deg = 1.8     # 1.8° at optical telephoto (30x)

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

        # Board Firmware Versions & Inventory
        self.crown_version = "3.1.9-rc4"
        self.crown_part_number = "CRWN-40-101"
        self.clevis_version = "2.4.1"
        self.clevis_part_number = "CLVS-40-201"
        self.payload_version = "1.8.0"
        self.payload_part_number = "PAYL-40-301"
        self.payload_hw_type = 1
        self.payload_hw_rev = 2
        self.tracker_version = "2.1.0"
        self.tracker_part_number = "TRKR-40-401"
        self.tracker_app_bits = 0x00000007
        self.lensctl_version = "1.2.0"
        self.retract_version = "1.0.0"

        # Thermal FLIR & Lynred Settings (for HD40-LV)
        self.flir_palette = 0  # 0: WhiteHot, 1: BlackHot, 2: Rainbow, 4: Ironbow
        self.flir_nuc_type = 0
        self.flir_black_hot = 0
        self.flir_disable_sffc = 0
        self.flir_max_agc_gain = 25
        self.flir_ace_level = 1
        self.flir_dde_threshold = 24
        self.flir_agc_midpoint = 128
        self.flir_integration_time = 8.0
        self.flir_agc_type = 0
        self.flir_agc_gamma = 1.0
        self.flir_agc_linear_percent = 50.0
        self.flir_tce_enable = 0
        self.flir_tce_gamma = 90
        self.flir_tce_clip_limit = 15
        self.flir_tce_alpha = 248

        self.lynred_contrast_mode = 0
        self.lynred_ice_moving_avg = 50
        self.lynred_ice_min_thresh = 0
        self.lynred_ice_max_thresh = 100

        # Unified Camera Error / Boresight Alignment Settings
        self.cam_errors = [(0.0, 0.0) for _ in self.cameras]

        # Video Tracking State & Targets
        self.primary_track = {"index": 0, "id": 1, "status": 1, "pan": 0.0, "tilt": 0.0, "size": 0.05, "active": False}
        self.active_tracks: Dict[int, dict] = {}
        self.tle_running = False
        self.tle_filter_type = 0

        # Onboard Video Recording & Stream Options
        self.videorecord_enabled = False
        self.videorecord_state = 0  # 0: Idle, 1: Recording, 2: Rec+Stream, 3: Streaming
        self.videorecord_udp_dest_ip = 0
        self.videorecord_udp_dest_port = 5004
        self.videorecord_bitrate = 4000
        self.videorecord_klv_mode = 1
        self.videorecord_disk_consumption = 0.15

        # Navigation Aiding, Slant Range & INS
        self.slant_range = 0.0
        self.slant_range_max_age_ms = 1000
        self.slant_range_source = 0
        self.slant_range_time = 0.0
        self.ins_platform_rotation = False
        self.ins_euler = [0.0, 0.0, 0.0]
        self.ins_initial_heading = 0.0
        self.ins_gps_lever_arm = [0.0, 0.0, 0.0]
        self.ins_quality_mode = 2
        self.geoid_undulation = 0.0

        # Telemetry, Comms & Retract
        self.crown_mode = 0
        self.retract_cmd = 0
        self.retract_state = 0
        self.retract_pos = 0.0
        self.retract_flags = 0
        self.net_ip = 0xC0A801C8        # 192.168.1.200
        self.net_mask = 0xFFFFFF00      # 255.255.255.0
        self.net_gateway = 0xC0A80101   # 192.168.1.1
        self.net_low_delay = 0
        self.net_mtu = 1500
        self.net_secondary_port = 0
        self.net_low_bandwidth = 0
        self.net_max_clients = 4

        # Gimbal Limits & Physical Specs
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

    @property
    def current_fov_deg(self) -> Tuple[float, float]:
        if self.cameras:
            cam = self.cameras[min(self.camera_id, len(self.cameras) - 1)]
            min_focal = cam.get("min_focal", 4.3)
            pixel_pitch = cam.get("pixel_pitch", 0.00297)
            width = cam.get("width", 1280)
            height = cam.get("height", 720)
            max_zoom = cam.get("max_total_zoom", self.max_total_zoom)
            zoom = max(1.0, min(self.camera_zoom, max_zoom))
            focal = min_focal * zoom
            hfov_deg = math.degrees(2.0 * math.atan2(0.5 * (width * pixel_pitch), focal))
            vfov_deg = math.degrees(2.0 * math.atan2(0.5 * (height * pixel_pitch), focal))
            return hfov_deg, vfov_deg
        return 47.7, 28.0

    def update_from_command(self, packet: OrionPacket) -> Optional[OrionPacket]:
        if packet.packet_id == OrionPktType.INITIALIZE:
            self.initialized = True
            logger.info("Gimbal initialized via INITIALIZE command (0x00)")
            
        elif packet.packet_id == OrionPktType.RESET:
            logger.info("Gimbal reset via RESET command (0x04) to initial state")
            self.__init__(dt=self.dt, terrain_engine=self.terrain,
                          lat=self.initial_lat, lon=self.initial_lon, alt=self.initial_alt,
                          pan=self.initial_pan, tilt=self.initial_tilt, heading=self.initial_heading,
                          speed=self.initial_speed)
            self.initialized = True
            
        elif packet.packet_id == OrionPktType.STARTUP_CMD:
            self.initialized = True
            logger.info("Gimbal initialized via STARTUP_CMD command (0x07)")
            
        elif packet.packet_id == OrionPktType.CMD:
            if len(packet.data) == 8:
                pan, tilt = struct.unpack(">ff", packet.data[:8])
                self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug("CMD (8-byte float) target: pan=%.2f, tilt=%.2f", self.target_pan, self.target_tilt)
            elif len(packet.data) >= 4:
                pan_raw, tilt_raw = struct.unpack_from(">hh", packet.data, 0)
                # Mode byte is at offset 4 for standard OrionCmd_t and OrionCmdExtended_t
                if len(packet.data) >= 5:
                    mode = packet.data[4]
                else:
                    # Legacy 4-byte payload without mode byte (treat as position mode)
                    mode = 0x50

                old_mode = self.mode
                self.mode = mode
                if old_mode != mode:
                    logger.info("Gimbal operational mode changed: 0x%02X -> 0x%02X", old_mode, mode)

                # Disabled mode: ORION_MODE_DISABLED (0x00)
                if mode == 0x00:
                    self.target_pan = self.current_pan
                    self.target_tilt = self.current_tilt
                    if hasattr(self, 'physics'):
                        self.physics.pan["vel"] = 0.0
                        self.physics.pan["acc"] = 0.0
                        self.physics.tilt["vel"] = 0.0
                        self.physics.tilt["acc"] = 0.0
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Disabled mode: motor output disabled")

                # Rate modes: ORION_MODE_RATE (0x10), ORION_MODE_GEO_RATE (0x11), ORION_MODE_SCENE (0x30)
                elif mode in (0x10, 0x11, 0x30):
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

                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug(
                            "CMD Rate mode 0x%02X: rates (pan=%.2f, tilt=%.2f deg/s) -> targets (pan=%.2f, tilt=%.2f)",
                            mode, pan_rate, tilt_rate, self.target_pan, self.target_tilt
                        )

                # Flat field correction modes: ORION_MODE_FFC_AUTO (0x20), ORION_MODE_FFC_MANUAL (0x21)
                elif mode == 0x20:  # ORION_MODE_FFC_AUTO / FFC
                    self.ffc_active = True
                    self.target_pan = 0.0
                    self.target_tilt = min(25.0, self.tilt_max)
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD FFC Auto mode: driving to blackbody position (pan=0, tilt=%.2f)", self.target_tilt)

                elif mode == 0x21:  # ORION_MODE_FFC_MANUAL
                    self.ffc_active = True
                    pan, tilt = math.degrees(pan_raw / 1000.0), math.degrees(tilt_raw / 1000.0)
                    self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                    self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD FFC Manual mode: driving to payload target (pan=%.2f, tilt=%.2f)", self.target_pan, self.target_tilt)

                # Track mode: ORION_MODE_TRACK (0x31)
                elif mode == 0x31:  # ORION_MODE_TRACK
                    norm_x = max(-0.5, min(0.5, pan_raw / 1000.0))
                    norm_y = max(-0.5, min(0.5, tilt_raw / 1000.0))
                    self.track_box = (norm_x, norm_y)
                    if self.tracking_mode == 0:
                        self.tracking_mode = 2  # Point track
                    hfov_deg, vfov_deg = self.current_fov_deg
                    new_pan = self.target_pan + norm_x * hfov_deg
                    new_tilt = self.target_tilt - norm_y * vfov_deg
                    self.target_pan = (new_pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(new_pan, self.pan_max), self.pan_min)
                    self.target_tilt = max(min(new_tilt, self.tilt_max), self.tilt_min)
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug(
                            "CMD Track mode: track_box=(%.3f, %.3f) -> targets (pan=%.2f, tilt=%.2f)",
                            norm_x, norm_y, self.target_pan, self.target_tilt
                        )

                # Gyro calibration modes: ORION_MODE_CALIBRATION (0x40), ORION_MODE_NULL_GYROS (0x41)
                elif mode == 0x40:  # ORION_MODE_CALIBRATION
                    self.calibration_active = True
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Calibration mode activated")

                elif mode == 0x41:  # ORION_MODE_NULL_GYROS
                    self.target_pan = self.current_pan
                    self.target_tilt = self.current_tilt
                    if hasattr(self, 'physics'):
                        self.physics.pan["vel"] = 0.0
                        self.physics.pan["acc"] = 0.0
                        self.physics.tilt["vel"] = 0.0
                        self.physics.tilt["acc"] = 0.0
                    self.gyro_calibrated = True
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Null Gyros mode: holding position (pan=%.2f, tilt=%.2f)", self.target_pan, self.target_tilt)

                # Position mode: ORION_MODE_POSITION (0x50)
                elif mode == 0x50:
                    pan, tilt = math.degrees(pan_raw / 1000.0), math.degrees(tilt_raw / 1000.0)
                    self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else max(min(pan, self.pan_max), self.pan_min)
                    self.target_tilt = max(min(tilt, self.tilt_max), self.tilt_min)
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Position mode 0x%02X: targets (pan=%.2f, tilt=%.2f)", mode, self.target_pan, self.target_tilt)

                # Position no limits mode: ORION_MODE_POSITION_NO_LIMITS (0x51)
                elif mode == 0x51:
                    pan, tilt = math.degrees(pan_raw / 1000.0), math.degrees(tilt_raw / 1000.0)
                    self.target_pan = (pan + 180.0) % 360.0 - 180.0 if self.pan_continuous else pan
                    self.target_tilt = tilt  # Bypasses soft pan/tilt limits
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Position No Limits mode 0x%02X: targets (pan=%.2f, tilt=%.2f)", mode, self.target_pan, self.target_tilt)

                elif mode == 0x70:  # ORION_MODE_PATH
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Path mode activated")

                elif mode == 0x71:  # ORION_MODE_DOWN
                    self.target_tilt = self.tilt_min
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("CMD Down mode: target_tilt=%.2f", self.target_tilt)

                # If extended command (len >= 10), return OrionCmdExtendedResponse
                if len(packet.data) >= 10:
                    return self.get_cmd_extended_response_packet(packet.data)
            else:
                logger.warning("CMD packet data too short: %d bytes (minimum 4)", len(packet.data))
        
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                self.camera_id = packet.data[0]
                self.camera_ready = False
                logger.info("Switched active camera to index %d", self.camera_id)
            
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
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(
                    "Camera state cmd: zoom=%.2fx, focus=%.2f, cam_id=%d, ready=%s",
                    self.camera_zoom, self.camera_focus, self.camera_id, self.camera_ready
                )
            return self.get_camera_state_packet()
        
        elif packet.packet_id == OrionPktType.LASER_CMD:
            if len(packet.data) >= 4:
                self.laser_power = max(0.0, min(1.0, struct.unpack(">f", packet.data[:4])[0]))
                logger.info("Laser power set to %.2f (%.1f%%)", self.laser_power, self.laser_power * 100.0)

        elif packet.packet_id == OrionPktType.LASER_STATES:
            return self.get_laser_state_packet()

        elif packet.packet_id == OrionPktType.UART_CONFIG:
            if len(packet.data) >= 4:
                self.baud_rate = struct.unpack(">I", packet.data[:4])[0]
                logger.info("UART baud rate configured to %d", self.baud_rate)

        elif packet.packet_id == OrionPktType.LIMITS:
            if len(packet.data) == 8:
                self.pan_limit, self.tilt_limit = struct.unpack(">ff", packet.data[:8])
                self.pan_max = self.pan_limit
                self.pan_min = -self.pan_limit
                self.tilt_max = self.tilt_limit
                self.tilt_min = -self.tilt_limit
                self.pan_continuous = False
                logger.info("Gimbal limits updated: pan=±%.1f deg, tilt=±%.1f deg", self.pan_limit, self.tilt_limit)
            return self.get_limits_packet()

        elif packet.packet_id == OrionPktType.GPS_DATA:
            if not self.is_faulty:
                was_gps = self.gps_received
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
                    if not was_gps:
                        logger.info("GPS fix acquired: lat=%.6f, lon=%.6f, alt=%.1fm", new_lat, new_lon, new_alt)
                    elif logger.isEnabledFor(logging.DEBUG):
                        logger.debug("GPS update: lat=%.6f, lon=%.6f, alt=%.1fm, spd=%.1f kts", new_lat, new_lon, new_alt, self.aircraft_speed)

        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            if len(packet.data) >= 8 and len(packet.data) < 12:
                # Official Orion SDK OrionExtHeadingData packet (8 bytes: extHeading >h, noise >H, flags >H, pitch >h)
                raw_hdg = struct.unpack_from(">h", packet.data, 0)[0]
                raw_pitch = struct.unpack_from(">h", packet.data, 6)[0]
                self.aircraft_heading = math.degrees(raw_hdg / 10430.06004058) % 360.0
                self.aircraft_pitch = math.degrees(raw_pitch / 10430.06004058)
            elif len(packet.data) >= 12:
                self.aircraft_heading, self.aircraft_roll, self.aircraft_pitch = struct.unpack(">fff", packet.data[:12])
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Ext heading: hdg=%.1f deg, pitch=%.1f deg, roll=%.1f deg", self.aircraft_heading, self.aircraft_pitch, self.aircraft_roll)

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

        elif packet.packet_id == OrionPktType.GEOPOINT_CMD:
            target_lat, target_lon, target_alt = None, None, None
            vel_ned = [0.0, 0.0, 0.0]
            joystick_range = 0.0
            options = 0

            if len(packet.data) >= 18:
                raw_lat, raw_lon, raw_alt, vn_raw, ve_raw, vd_raw = struct.unpack_from(">iiihhh", packet.data, 0)
                # targetLat in radians * 572957795.1308233 is degrees * 10000000 (1e7)
                target_lat = raw_lat / 10000000.0
                target_lon = raw_lon / 10000000.0
                target_alt = raw_alt / 10000.0
                vel_ned = [vn_raw / 100.0, ve_raw / 100.0, vd_raw / 100.0]
                if len(packet.data) >= 20:
                    joystick_range = float(struct.unpack_from(">H", packet.data, 18)[0])
                if len(packet.data) >= 21:
                    options = packet.data[20]
            elif len(packet.data) >= 12:
                # Float32 fallback
                target_lat, target_lon, target_alt = struct.unpack(">fff", packet.data[:12])

            if target_lat is not None:
                if self.mode == OrionMode.GEOPOINT and joystick_range > 0:
                    # In joystick velocity mode, update velocity and options only
                    self.geopoint_vel_ned = vel_ned
                    self.geopoint_options = options
                    self.geopoint_joystick_range = joystick_range
                else:
                    self.geopoint_lat = float(target_lat)
                    self.geopoint_lon = float(target_lon)
                    self.geopoint_alt = float(target_alt)
                    self.geopoint_vel_ned = [float(v) for v in vel_ned]
                    self.geopoint_joystick_range = joystick_range
                    self.geopoint_options = options

                self.mode = OrionMode.GEOPOINT
                target_pan, target_tilt = self.calculate_geopoint_pan_tilt()
                self.target_pan = target_pan
                self.target_tilt = target_tilt
                logger.info(
                    "Geopoint target set: lat=%.6f, lon=%.6f, alt=%.1fm -> targets (pan=%.2f, tilt=%.2f)",
                    self.geopoint_lat, self.geopoint_lon, self.geopoint_alt, target_pan, target_tilt
                )

                # If closure mode requested (options bit 1 set), immediately achieve pointing
                if options & 0x02:
                    self.physics.pan["pos"] = target_pan
                    self.physics.tilt["pos"] = target_tilt
                    self.physics.pan["vel"] = 0.0
                    self.physics.tilt["vel"] = 0.0

            return self.get_geopoint_cmd_packet()

        elif packet.packet_id == OrionPktType.PATH:
            self.mode = OrionMode.PATH
            if len(packet.data) >= 1:
                num_points = packet.data[0]
                point_down = bool(packet.data[1] & 0x01) if len(packet.data) >= 2 else False
                if point_down or num_points == 0:
                    self.target_tilt = self.tilt_min
                self.path_points = []
                offset = 2
                for _ in range(min(num_points, 15)):
                    if offset + 9 <= len(packet.data):
                        x = int.from_bytes(packet.data[offset:offset+3], 'big', signed=True)
                        y = int.from_bytes(packet.data[offset+3:offset+6], 'big', signed=True)
                        z = int.from_bytes(packet.data[offset+6:offset+9], 'big', signed=True)
                        self.path_points.append((x, y, z))
                        offset += 9
                self.path_progress = 0.0
                self.stare_time = 0.0
                self.path_from = 0
                self.path_to = min(1, num_points)
            logger.info("Path mode activated via PATH packet (0xD7) with %d points", len(self.path_points))

        elif packet.packet_id == OrionPktType.CROWN_VERSION:
            return self.get_crown_version_packet()

        elif packet.packet_id == OrionPktType.CLEVIS_VERSION:
            return self.get_clevis_version_packet()

        elif packet.packet_id == OrionPktType.PAYLOAD_VERSION:
            return self.get_payload_version_packet()

        elif packet.packet_id == OrionPktType.TRACKER_VERSION:
            return self.get_tracker_version_packet()

        elif packet.packet_id == OrionPktType.LENSCTL_VERSION:
            return self.get_lensctl_version_packet()

        elif packet.packet_id == OrionPktType.RETRACT_VERSION:
            return self.get_retract_version_packet()

        elif packet.packet_id == OrionPktType.PRODUCT:
            return self.get_product_packet()

        elif packet.packet_id == OrionPktType.BOARD:
            b_enum = packet.data[0] if len(packet.data) >= 1 else 2
            return self.get_board_packet(b_enum)

        elif packet.packet_id == OrionPktType.BOARD_HEARTBEAT:
            return self.get_board_heartbeat_packet()

        elif packet.packet_id == OrionPktType.FLIR_SETTINGS:
            if len(packet.data) >= 2:
                bf1 = packet.data[1]
                self.flir_disable_sffc = (bf1 >> 7) & 1
                self.flir_palette = (bf1 >> 4) & 0x07
                self.flir_nuc_type = (bf1 >> 1) & 0x07
                self.flir_black_hot = bf1 & 0x01
            if len(packet.data) >= 3:
                self.flir_max_agc_gain = packet.data[2]
            if len(packet.data) >= 4:
                self.flir_ace_level = struct.unpack_from(">b", packet.data, 3)[0]
            if len(packet.data) >= 5:
                self.flir_dde_threshold = packet.data[4]
            if len(packet.data) >= 6:
                self.flir_agc_midpoint = packet.data[5]
            if len(packet.data) >= 7:
                self.flir_integration_time = 1.0 + (packet.data[6] / 8.793103448275861)
            if len(packet.data) >= 8:
                self.flir_agc_type = packet.data[7]
            if len(packet.data) >= 9:
                self.flir_agc_gamma = 0.5 + (packet.data[8] / 72.85714285714286)
            if len(packet.data) >= 10:
                self.flir_agc_linear_percent = packet.data[9] / 2.55
            if len(packet.data) >= 11:
                self.flir_tce_enable = (packet.data[10] >> 7) & 1
            if len(packet.data) >= 12:
                self.flir_tce_gamma = packet.data[11]
            if len(packet.data) >= 13:
                self.flir_tce_clip_limit = packet.data[12]
            if len(packet.data) >= 14:
                self.flir_tce_alpha = packet.data[13]
            return self.get_flir_settings_packet()

        elif packet.packet_id == OrionPktType.LYNRED_SETTINGS:
            if len(packet.data) >= 3:
                bf = struct.unpack_from(">H", packet.data, 1)[0]
                self.lynred_contrast_mode = (bf >> 15) & 1
                self.lynred_ice_moving_avg = (bf >> 6) & 0x1FF
            if len(packet.data) >= 4:
                self.lynred_ice_min_thresh = packet.data[3]
            if len(packet.data) >= 5:
                self.lynred_ice_max_thresh = packet.data[4]
            return self.get_lynred_settings_packet()

        elif packet.packet_id == OrionPktType.UNIFIED_CAM_ERR_SETTINGS:
            if len(packet.data) >= 1:
                num = packet.data[0]
                new_errs = []
                offset = 1
                for _ in range(num):
                    if offset + 4 <= len(packet.data):
                        p_raw, t_raw = struct.unpack_from(">hh", packet.data, offset)
                        new_errs.append((p_raw / 20860.12008116854, t_raw / 20860.12008116854))
                        offset += 4
                if new_errs:
                    self.cam_errors = new_errs
            return self.get_unified_cam_err_settings_packet()

        elif packet.packet_id == OrionPktType.TRACK_CMD:
            if len(packet.data) >= 1:
                cmd = packet.data[0]
                target_pan = 0.0
                target_tilt = 0.0
                track_idx = 0
                resize = 0.0
                if len(packet.data) >= 5:
                    p_raw, t_raw = struct.unpack_from(">hh", packet.data, 1)
                    target_pan = p_raw / 1000.0
                    target_tilt = t_raw / 1000.0
                if len(packet.data) >= 9:
                    track_idx = struct.unpack_from(">i", packet.data, 5)[0]
                if len(packet.data) >= 11:
                    resize = struct.unpack_from(">h", packet.data, 9)[0] / 1000.0

                if cmd in (0, 1):  # TRACK_START_PRIMARY, TRACK_START_SECONDARY
                    self.mode = OrionMode.TRACK
                    self.primary_track["active"] = True
                    self.primary_track["pan"] = target_pan
                    self.primary_track["tilt"] = target_tilt
                    self.primary_track["id"] = max(1, self.primary_track["id"] + 1)
                    if cmd == 1:
                        self.active_tracks[track_idx] = {
                            "id": track_idx, "pan": target_pan, "tilt": target_tilt, "active": True
                        }
                elif cmd == 2:  # TRACK_STOP_ALL
                    self.mode = OrionMode.RATE
                    self.primary_track["active"] = False
                    self.active_tracks.clear()
                elif cmd == 3:  # TRACK_STOP_ALL_BUT_PRIMARY
                    self.active_tracks.clear()
                elif cmd in (4, 6):  # TRACK_NUDGE_PRIMARY, TRACK_NUDGE_BY_INDEX
                    if cmd == 4:
                        self.primary_track["pan"] += target_pan
                        self.primary_track["tilt"] += target_tilt
                    elif track_idx in self.active_tracks:
                        self.active_tracks[track_idx]["pan"] += target_pan
                        self.active_tracks[track_idx]["tilt"] += target_tilt
                elif cmd in (5, 7):  # TRACK_RESIZE_PRIMARY, TRACK_RESIZE_BY_INDEX
                    if cmd == 5:
                        self.primary_track["size"] = max(0.01, self.primary_track.get("size", 0.05) + resize)
                    elif track_idx in self.active_tracks:
                        self.active_tracks[track_idx]["size"] = max(0.01, self.active_tracks[track_idx].get("size", 0.05) + resize)
                elif cmd == 8:  # TRACK_REMOVE_BY_INDEX
                    self.active_tracks.pop(track_idx, None)
                    if track_idx == 0:
                        self.primary_track["active"] = False
            return self.get_geo_track_status_packet()

        elif packet.packet_id == OrionPktType.GEO_TRACK_STATUS:
            idx = packet.data[0] if len(packet.data) >= 1 else 0
            return self.get_geo_track_status_packet(idx)

        elif packet.packet_id == OrionPktType.TLE_COMMAND:
            if len(packet.data) >= 2:
                self.tle_filter_type = packet.data[0]
                self.tle_running = bool(packet.data[1])
            return self.get_tle_status_packet()

        elif packet.packet_id == OrionPktType.TLE_STATUS:
            return self.get_tle_status_packet()

        elif packet.packet_id == OrionPktType.VIDEORECORD_CMD:
            if len(packet.data) >= 1:
                bf = packet.data[0]
                udp_en = bool(bf & 0x80)
                rec_en = bool(bf & 0x40)
                del_all = bool(bf & 0x08)
                if rec_en and udp_en:
                    self.videorecord_state = 2
                elif rec_en:
                    self.videorecord_state = 1
                elif udp_en:
                    self.videorecord_state = 3
                else:
                    self.videorecord_state = 0
                if del_all:
                    self.videorecord_disk_consumption = 0.01
            if len(packet.data) >= 7:
                self.videorecord_udp_dest_ip, self.videorecord_udp_dest_port = struct.unpack_from(">IH", packet.data, 1)
            return self.get_videorecord_status_packet()

        elif packet.packet_id == OrionPktType.VIDEORECORD_STATUS:
            return self.get_videorecord_status_packet()

        elif packet.packet_id == OrionPktType.VIDEORECORD_CLOCK:
            return self.get_videorecord_clock_packet()

        elif packet.packet_id == OrionPktType.RANGE_DATA:
            if len(packet.data) >= 7:
                rng_raw, age, src = struct.unpack_from(">IHB", packet.data, 0)
                self.slant_range = rng_raw / 100.0
                self.slant_range_max_age_ms = age
                self.slant_range_source = src
                self.slant_range_time = self.uptime
                logger.info("Range data received: %.2fm (source=%d, age=%dms)", self.slant_range, src, age)

        elif packet.packet_id == OrionPktType.INS_OPTIONS:
            if len(packet.data) >= 1:
                self.ins_platform_rotation = bool(packet.data[0] & 0x80)
            if len(packet.data) >= 10:
                r, p, y = struct.unpack_from(">hhh", packet.data, 4)
                self.ins_euler = [r / 10430.06004058, p / 10430.06004058, y / 10430.06004058]
            if len(packet.data) >= 12:
                init_hdg = struct.unpack_from(">h", packet.data, 10)[0]
                self.ins_initial_heading = init_hdg / 10430.06004058
            if len(packet.data) >= 18:
                ax, ay, az = struct.unpack_from(">hhh", packet.data, 12)
                self.ins_gps_lever_arm = [ax / 1000.0, ay / 1000.0, az / 1000.0]
            return self.get_ins_options_packet()

        elif packet.packet_id == OrionPktType.INS_QUALITY:
            return self.get_ins_quality_packet()

        elif packet.packet_id == OrionPktType.CROWN_MODE:
            if len(packet.data) >= 1:
                self.crown_mode = packet.data[0]
            return self.get_crown_mode_packet()

        elif packet.packet_id == OrionPktType.NETWORK_SETTINGS:
            if len(packet.data) >= 18:
                self.net_ip, self.net_mask, self.net_gateway, self.net_low_delay, self.net_mtu, self.net_secondary_port, self.net_low_bandwidth, self.net_max_clients = struct.unpack_from(">IIIBHHBB", packet.data, 0)
            return self.get_network_settings_packet()

        elif packet.packet_id == OrionPktType.RETRACT_CMD:
            if len(packet.data) >= 1:
                self.retract_cmd = packet.data[0]
                self.retract_state = 1 if self.retract_cmd == 1 else 0
            return self.get_retract_status_packet()

        elif packet.packet_id == OrionPktType.RETRACT_STATUS:
            return self.get_retract_status_packet()

        elif packet.packet_id == OrionPktType.GEOID_UNDULATION:
            if len(packet.data) >= 2:
                und_raw = struct.unpack_from(">h", packet.data, 0)[0]
                self.geoid_undulation = und_raw / 100.0

        elif packet.packet_id == OrionPktType.AUTOPILOT_DATA:
            if len(packet.data) >= 12:
                ias, tas = struct.unpack_from(">ff", packet.data, 4)
                if ias > 0:
                    self.aircraft_speed = ias * 1.94384
            
        return None

    def calculate_geopoint_pan_tilt(self) -> Tuple[float, float]:
        """
        Calculate the required pan and tilt angles (degrees) so the camera boresight
        locks directly onto (self.geopoint_lat, self.geopoint_lon, self.geopoint_alt)
        from current aircraft position and attitude.
        """
        d_lat = self.geopoint_lat - self.gps_lat
        d_lon = self.geopoint_lon - self.gps_lon
        lat_m = (self.gps_lat + self.geopoint_lat) / 2.0
        cos_lat = math.cos(math.radians(lat_m))

        north_m = d_lat * 111320.0
        east_m = d_lon * 111320.0 * (cos_lat if abs(cos_lat) > 1e-6 else 1.0)
        down_m = self.gps_alt - self.geopoint_alt
        ground_dist = math.hypot(north_m, east_m)

        if ground_dist < 1e-3:
            # Target is directly above or below
            elevation = -90.0 if down_m > 0 else 90.0
            azimuth = self.aircraft_heading
        else:
            azimuth = (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0
            elevation = math.degrees(math.atan2(-down_m, ground_dist))

        # Camera heading = aircraft_heading + pan  =>  pan = azimuth - aircraft_heading
        pan = (azimuth - self.aircraft_heading + 180.0) % 360.0 - 180.0
        # Camera pitch = aircraft_pitch + tilt  =>  tilt = elevation - aircraft_pitch
        tilt = elevation - self.aircraft_pitch

        if self.pan_continuous:
            pan = (pan + 180.0) % 360.0 - 180.0
        else:
            pan = max(min(pan, self.pan_max), self.pan_min)
        tilt = max(min(tilt, self.tilt_max), self.tilt_min)

        return pan, tilt

    def get_geopoint_cmd_packet(self) -> bytes:
        lat_raw = int(round(self.geopoint_lat * 10000000.0))
        lon_raw = int(round(self.geopoint_lon * 10000000.0))
        alt_raw = int(round(self.geopoint_alt * 10000.0))
        vn_raw = int(round(self.geopoint_vel_ned[0] * 100.0))
        ve_raw = int(round(self.geopoint_vel_ned[1] * 100.0))
        vd_raw = int(round(self.geopoint_vel_ned[2] * 100.0))
        rng_raw = int(round(self.geopoint_joystick_range)) & 0xFFFF
        data = struct.pack(">iiihhhHB", lat_raw, lon_raw, alt_raw, vn_raw, ve_raw, vd_raw, rng_raw, self.geopoint_options & 0xFF)
        return OrionPacket(OrionPktType.GEOPOINT_CMD, data).encode()

    def step(self):
        # Advance simulated aircraft position along heading if speed > 0 and no external GPS active
        if not self.gps_received and self.aircraft_speed > 0.0:
            speed_mps = self.aircraft_speed / 1.94384
            dist_m = speed_mps * self.dt
            hdg_rad = math.radians(self.aircraft_heading)
            self.gps_lat += (dist_m * math.cos(hdg_rad)) / 111320.0
            cos_lat = math.cos(math.radians(self.gps_lat))
            self.gps_lon += (dist_m * math.sin(hdg_rad)) / (111320.0 * (cos_lat if abs(cos_lat) > 1e-6 else 1.0))

        # Propagate target and update lock pan/tilt if in GEOPOINT mode
        if self.mode == OrionMode.GEOPOINT:
            if any(v != 0.0 for v in self.geopoint_vel_ned):
                vn, ve, vd = self.geopoint_vel_ned
                d_lat = (vn * self.dt) / 111320.0
                cos_lat = math.cos(math.radians(self.geopoint_lat))
                d_lon = (ve * self.dt) / (111320.0 * (cos_lat if abs(cos_lat) > 1e-6 else 1.0))
                d_alt = -vd * self.dt
                self.geopoint_lat += d_lat
                self.geopoint_lon += d_lon
                self.geopoint_alt += d_alt

            self.target_pan, self.target_tilt = self.calculate_geopoint_pan_tilt()

        # Mode-specific physics & motion updates
        if self.mode == OrionMode.DISABLED:
            # Motors unpowered - freeze velocities
            if hasattr(self, 'physics'):
                self.physics.pan["vel"] = 0.0
                self.physics.pan["acc"] = 0.0
                self.physics.tilt["vel"] = 0.0
                self.physics.tilt["acc"] = 0.0
        elif self.mode == OrionMode.CALIBRATION and getattr(self, 'calibration_active', False):
            # Gentle calibration sweep
            self.target_pan = (self.initial_pan + 5.0 * math.sin(self.uptime * 2.0))
            self.target_tilt = (self.initial_tilt + 5.0 * math.cos(self.uptime * 2.0))
            self.physics.step(
                self.target_pan,
                self.target_tilt,
                continuous_pan=self.pan_continuous,
                tilt_min=self.tilt_min,
                tilt_max=self.tilt_max
            )
        elif self.mode == OrionMode.POSITION_NO_LIMITS:
            self.physics.step(
                self.target_pan,
                self.target_tilt,
                continuous_pan=self.pan_continuous,
                tilt_min=-180.0,
                tilt_max=180.0
            )
        else:
            self.physics.step(
                self.target_pan, 
                self.target_tilt, 
                continuous_pan=self.pan_continuous,
                tilt_min=self.tilt_min, 
                tilt_max=self.tilt_max
            )

        if self.mode == OrionMode.PATH and getattr(self, 'path_points', None):
            self.path_progress = min(1.0, getattr(self, 'path_progress', 0.0) + 0.05 * self.dt)
            if self.path_progress >= 1.0 and self.path_points:
                self.path_from = len(self.path_points) - 1
                self.path_to = len(self.path_points) - 1
        self.faults.apply_faults(self)
        if self.terrain and self.terrain.enabled:
            terrain_alt = self.terrain.get_elevation(self.gps_lat, self.gps_lon)
            self.terrain_alt = terrain_alt
            if self.gps_alt == 0.0:
                self.gps_alt = terrain_alt
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

        # Mode: 1 if faulty, 0 if disabled, otherwise current state mode
        if self.is_faulty or bool(self.faults.active_faults):
            mode = 1
        elif not self.initialized or getattr(self, 'mode', 0) == OrionMode.DISABLED:
            mode = 0
        else:
            mode = getattr(self, 'mode', 16)

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

        # Compute line-of-sight ECEF vector if in GEOPOINT mode
        if mode == 0x60:  # ORION_MODE_GEOPOINT
            lat_rad_g = math.radians(self.gps_lat)
            lon_rad_g = math.radians(self.gps_lon)
            dlat_m = (self.geopoint_lat - self.gps_lat) * 111320.0
            dlon_m = (self.geopoint_lon - self.gps_lon) * 111320.0 * math.cos(lat_rad_g)
            dalt_m = -(self.geopoint_alt - self.gps_alt)
            s_lat, c_lat = math.sin(lat_rad_g), math.cos(lat_rad_g)
            s_lon, c_lon = math.sin(lon_rad_g), math.cos(lon_rad_g)
            ecef_x = -s_lat * c_lon * dlat_m - s_lon * dlon_m - c_lat * c_lon * dalt_m
            ecef_y = -s_lat * s_lon * dlat_m + c_lon * dlon_m - c_lat * s_lon * dalt_m
            ecef_z = c_lat * dlat_m - s_lat * dalt_m
            los_x = int(round(max(-32767, min(32767, ecef_x))))
            los_y = int(round(max(-32767, min(32767, ecef_y))))
            los_z = int(round(max(-32767, min(32767, ecef_z))))
        else:
            los_x, los_y, los_z = 0, 0, 0

        data = bytearray()
        data.extend(struct.pack(">IIHh", uptime_ms, 0, 0, 0))
        data.extend(struct.pack(">iii", int(round(lat_rad * 572957795.1308)), int(round(lon_rad * 572957795.1308)), int(round(alt_m * 10000.0))))
        data.extend(struct.pack(">hhh", 0, 0, 0))  # velNED
        data.extend(struct.pack(">hhhh", 32767, 0, 0, 0))  # gimbalQuat
        data.extend(struct.pack(">hh", int(round(pan_rad * 10430.06004058)), int(round(tilt_rad * 10430.06004058))))
        data.extend(struct.pack(">HH", int(round(hfov_rad * 10430.21919553)), int(round(vfov_rad * 10430.21919553))))
        data.extend(struct.pack(">hhh", los_x, los_y, los_z))  # losECEF
        data.extend(struct.pack(">HH", width, height))
        path_progress_raw = int(round(max(0.0, min(1.0, getattr(self, 'path_progress', 0.0))) * 255.0))
        stare_time_raw = int(round(max(0.0, min(2.55, getattr(self, 'stare_time', 0.0))) * 100.0))
        path_from = getattr(self, 'path_from', 0) & 0xFF
        path_to = getattr(self, 'path_to', 0) & 0xFF
        data.extend(struct.pack(">BBBBB", mode, path_progress_raw, stare_time_raw, path_from, path_to))
        data.extend(struct.pack(">ii", 0, 0))  # imageShifts
        data.extend(struct.pack(">HB", 0, 0))  # imageShiftDeltaTime, imageShiftConfidence
        data.extend(struct.pack(">hh", 0, 0))  # outputShifts
        range_source = self.slant_range_source if self.slant_range > 0 and (self.uptime - self.slant_range_time < self.slant_range_max_age_ms / 1000.0) else 0
        ins_rot_opt = 1 if self.ins_platform_rotation else 0
        data.extend(struct.pack(">BBbbB", range_source, 18, 0, 0, ins_rot_opt))  # rangeSource, leapSeconds, panAlignment, tiltAlignment, insRotationOption
        data.extend(struct.pack(">h", 0))  # imageRotation
        data.extend(struct.pack(">bB", self.camera_id, 0))  # cameraIndex, hasTrackData

        return OrionPacket(OrionPktType.GEOLOCATE_TELEMETRY_CORE, bytes(data)).encode()

    def get_crown_version_packet(self) -> bytes:
        data = struct.pack(
            ">16s16sI",
            _encode_fixed_string(self.crown_version, 16),
            _encode_fixed_string(self.crown_part_number, 16),
            int(self.uptime / 60.0),
        )
        return OrionPacket(OrionPktType.CROWN_VERSION, data).encode()

    def get_clevis_version_packet(self) -> bytes:
        data = struct.pack(
            ">16s16sI",
            _encode_fixed_string(self.clevis_version, 16),
            _encode_fixed_string(self.clevis_part_number, 16),
            int(self.uptime / 60.0),
        )
        return OrionPacket(OrionPktType.CLEVIS_VERSION, data).encode()

    def get_payload_version_packet(self) -> bytes:
        data = struct.pack(
            ">24s16sIbb",
            _encode_fixed_string(self.payload_version, 24),
            _encode_fixed_string(self.payload_part_number, 16),
            int(self.uptime / 60.0),
            self.payload_hw_type,
            self.payload_hw_rev,
        )
        return OrionPacket(OrionPktType.PAYLOAD_VERSION, data).encode()

    def get_tracker_version_packet(self) -> bytes:
        data = struct.pack(
            ">16s16sI",
            _encode_fixed_string(self.tracker_version, 16),
            _encode_fixed_string(self.tracker_part_number, 16),
            self.tracker_app_bits,
        )
        return OrionPacket(OrionPktType.TRACKER_VERSION, data).encode()

    def get_lensctl_version_packet(self) -> bytes:
        data = struct.pack(">16s", _encode_fixed_string(self.lensctl_version, 16))
        return OrionPacket(OrionPktType.LENSCTL_VERSION, data).encode()

    def get_retract_version_packet(self) -> bytes:
        data = struct.pack(">16s", _encode_fixed_string(self.retract_version, 16))
        return OrionPacket(OrionPktType.RETRACT_VERSION, data).encode()

    def get_product_packet(self) -> bytes:
        data = struct.pack(
            ">16sI64s",
            _encode_fixed_string(self.part_number, 16),
            self.serial_number,
            _encode_fixed_string(self.model_name, 64),
        )
        return OrionPacket(OrionPktType.PRODUCT, data).encode()

    def get_board_packet(self, board_enum: int = 2) -> bytes:
        mfg_date = (24 << 9) | (5 << 5) | 1  # 2024-05-01
        cal_date = (24 << 9) | (5 << 5) | 1
        data = struct.pack(
            ">III3sBHH",
            self.serial_number + board_enum,
            self.serial_number,
            0,
            b"\x00\x00\x00",
            board_enum,
            mfg_date,
            cal_date,
        )
        return OrionPacket(OrionPktType.BOARD, data).encode()

    def get_board_heartbeat_packet(self) -> bytes:
        data = struct.pack(">BB", 2, 0xFC)
        return OrionPacket(OrionPktType.BOARD_HEARTBEAT, data).encode()

    def get_flir_settings_packet(self) -> bytes:
        bf1 = ((self.flir_disable_sffc & 1) << 7) | ((self.flir_palette & 0x07) << 4) | ((self.flir_nuc_type & 0x07) << 1) | (self.flir_black_hot & 1)
        integ_encoded = int(round((max(1.0, min(30.0, self.flir_integration_time)) - 1.0) * 8.793103448275861))
        gamma_encoded = int(round((max(0.5, min(4.0, self.flir_agc_gamma)) - 0.5) * 72.85714285714286))
        lin_encoded = int(round(max(0.0, min(100.0, self.flir_agc_linear_percent)) * 2.55))
        bf_tce = (self.flir_tce_enable & 1) << 7
        cam_idx = 1 if len(self.cameras) > 1 else 0xFF
        data = struct.pack(
            ">BBbBBBBBBBBBBB",
            cam_idx,
            bf1,
            self.flir_max_agc_gain,
            self.flir_ace_level,
            self.flir_dde_threshold,
            self.flir_agc_midpoint,
            integ_encoded,
            self.flir_agc_type,
            gamma_encoded,
            lin_encoded,
            bf_tce,
            self.flir_tce_gamma,
            self.flir_tce_clip_limit,
            self.flir_tce_alpha,
        )
        return OrionPacket(OrionPktType.FLIR_SETTINGS, data).encode()

    def get_lynred_settings_packet(self) -> bytes:
        bf = ((self.lynred_contrast_mode & 1) << 15) | ((self.lynred_ice_moving_avg & 0x1FF) << 6)
        cam_idx = 1 if len(self.cameras) > 1 else 0xFF
        data = struct.pack(">BHBB", cam_idx, bf, self.lynred_ice_min_thresh, self.lynred_ice_max_thresh)
        return OrionPacket(OrionPktType.LYNRED_SETTINGS, data).encode()

    def get_unified_cam_err_settings_packet(self) -> bytes:
        num_cams = len(self.cameras)
        data = bytearray([num_cams])
        for i in range(num_cams):
            pan_rad, tilt_rad = self.cam_errors[i] if i < len(self.cam_errors) else (0.0, 0.0)
            p_raw = int(round(pan_rad * 20860.12008116854))
            t_raw = int(round(tilt_rad * 20860.12008116854))
            p_raw = max(-32768, min(32767, p_raw))
            t_raw = max(-32768, min(32767, t_raw))
            data.extend(struct.pack(">hh", p_raw, t_raw))
        return OrionPacket(OrionPktType.UNIFIED_CAM_ERR_SETTINGS, bytes(data)).encode()

    def get_geo_track_status_packet(self, index: int = 0) -> bytes:
        track = self.primary_track if index == 0 else self.active_tracks.get(index, {"id": index, "status": 0, "active": False})
        lat_rad = math.radians(self.gps_lat)
        lon_rad = math.radians(self.gps_lon)
        alt_m = self.gps_alt

        lat_raw = max(-2147483648, min(2147483647, int(round(lat_rad * 1367130550.516243))))
        lon_raw = max(-2147483648, min(2147483647, int(round(lon_rad * 683565275.2581217))))
        alt_raw = max(-8388608, min(8388607, int(round(alt_m * 100.0))))
        alt_s24 = alt_raw.to_bytes(3, byteorder='big', signed=True)

        status_byte = 1 if track.get("active", False) else 0
        state_byte = 1 if track.get("active", False) else 0

        data = bytearray()
        data.extend(struct.pack(">BIBii", index, track.get("id", 1), status_byte, lat_raw, lon_raw))
        data.extend(alt_s24)
        data.extend(struct.pack(">hhhhhh", 0, 0, 0, 0, 0, 0))
        data.extend(struct.pack(">iii", 0, 0, 0))
        data.extend(struct.pack(">HHH", 0, 0, 0))
        data.extend(struct.pack(">Bh", state_byte, 0))
        return OrionPacket(OrionPktType.GEO_TRACK_STATUS, bytes(data)).encode()

    def get_videorecord_status_packet(self) -> bytes:
        ver_bytes = _encode_fixed_string("1.0.0", 16)
        cams_info = bytearray()
        for i in range(3):
            status = 1 if i < len(self.cameras) else 0
            stream_id = i
            bitrate = 4000 if i < len(self.cameras) else 0
            framestats = 1000
            cams_info.extend(struct.pack(">BBHh", status, stream_id, bitrate, framestats))

        udp_en = 1 if self.videorecord_state in (2, 3) else 0
        rec_en = 1 if self.videorecord_state in (1, 2) else 0
        obr_en = 1 if (udp_en or rec_en) else 0
        bf = (udp_en << 7) | (rec_en << 6) | (1 << 5) | (1 << 4) | (obr_en << 3)
        disk_raw = int(round(self.videorecord_disk_consumption * 10000.0))

        data = bytearray()
        data.extend(ver_bytes)
        data.extend(cams_info)
        data.extend(struct.pack(
            ">BIHIBBhBB",
            bf,
            self.videorecord_udp_dest_ip,
            self.videorecord_udp_dest_port,
            self.videorecord_bitrate,
            self.videorecord_klv_mode,
            self.videorecord_state,
            disk_raw,
            0,
            0,
        ))
        return OrionPacket(OrionPktType.VIDEORECORD_STATUS, bytes(data)).encode()

    def get_videorecord_clock_packet(self) -> bytes:
        pts = int(self.uptime * 90000)
        data = struct.pack(">QQB", pts, pts, self.camera_id)
        return OrionPacket(OrionPktType.VIDEORECORD_CLOCK, data).encode()

    def get_ins_options_packet(self) -> bytes:
        b0 = 0x80 if self.ins_platform_rotation else 0x00
        e_roll = int(round(self.ins_euler[0] * 10430.06004058))
        e_pitch = int(round(self.ins_euler[1] * 10430.06004058))
        e_yaw = int(round(self.ins_euler[2] * 10430.06004058))
        init_hdg = int(round(self.ins_initial_heading * 10430.06004058))
        arm_x = int(round(self.ins_gps_lever_arm[0] * 1000.0))
        arm_y = int(round(self.ins_gps_lever_arm[1] * 1000.0))
        arm_z = int(round(self.ins_gps_lever_arm[2] * 1000.0))
        data = struct.pack(">BBBBhhhhhhhh", b0, 0, 0, 0, e_roll, e_pitch, e_yaw, init_hdg, arm_x, arm_y, arm_z, 0)
        return OrionPacket(OrionPktType.INS_OPTIONS, data).encode()

    def get_ins_quality_packet(self) -> bytes:
        uptime_ms = int(self.uptime * 1000)
        gps_src = 1
        imu_type = 2
        b_src = (gps_src << 5) | (imu_type & 0x07)
        ins_mode = self.ins_quality_mode
        flags = 0x80 | 0x40 | 0x20
        data = struct.pack(
            ">IBBBBBHHHHHHHHHHHH",
            uptime_ms,
            b_src,
            ins_mode,
            flags,
            10,
            10,
            0, 0, 0,
            10, 10, 20,
            10, 10, 20,
            200, 200, 400,
        )
        return OrionPacket(OrionPktType.INS_QUALITY, data).encode()

    def get_crown_mode_packet(self) -> bytes:
        data = struct.pack(">B", self.crown_mode)
        return OrionPacket(OrionPktType.CROWN_MODE, data).encode()

    def get_retract_status_packet(self) -> bytes:
        pos_raw = int(round(self.retract_pos * 1000.0))
        data = struct.pack(">BBhH", self.retract_cmd, self.retract_state, pos_raw, self.retract_flags)
        return OrionPacket(OrionPktType.RETRACT_STATUS, data).encode()

    def get_network_settings_packet(self) -> bytes:
        data = struct.pack(
            ">IIIBHHBB",
            self.net_ip,
            self.net_mask,
            self.net_gateway,
            self.net_low_delay,
            self.net_mtu,
            self.net_secondary_port,
            self.net_low_bandwidth,
            self.net_max_clients,
        )
        return OrionPacket(OrionPktType.NETWORK_SETTINGS, data).encode()

    def get_tle_status_packet(self) -> bytes:
        uptime_ms = int(self.uptime * 1000)
        filter_type = self.tle_filter_type
        state = 1 if self.tle_running else 0
        lat_rad = math.radians(self.gps_lat)
        lon_rad = math.radians(self.gps_lon)
        alt_m = self.gps_alt
        lat_raw = max(-2147483648, min(2147483647, int(round(lat_rad * 1367130550.516243))))
        lon_raw = max(-2147483648, min(2147483647, int(round(lon_rad * 683565275.2581217))))
        alt_raw = max(-8388608, min(8388607, int(round(alt_m * 100.0))))
        alt_s24 = alt_raw.to_bytes(3, byteorder='big', signed=True)
        data = bytearray()
        data.extend(struct.pack(">IBBHHii", uptime_ms, filter_type, state, 0, 10, lat_raw, lon_raw))
        data.extend(alt_s24)
        data.extend(struct.pack(">hhhhhhhhhHHHBB", 10, 10, 15, 10, 10, 0, 0, 0, 0, 10, 10, 10, 100, 0))
        return OrionPacket(OrionPktType.TLE_STATUS, bytes(data)).encode()
