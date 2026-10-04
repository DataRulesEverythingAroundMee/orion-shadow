import unittest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

class TestCommandControl(unittest.TestCase):
    def test_camera_switch(self):
        state = GimbalState()
        # Switch to camera 2
        packet = OrionPacket(OrionPktType.CAMERA_SWITCH, b'\x02')
        state.update_from_command(packet)
        self.assertEqual(state.camera_id, 2)

    def test_laser_command(self):
        state = GimbalState()
        # Set laser power to 0.75
        power = 0.75
        packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", power))
        state.update_from_command(packet)
        self.assertEqual(state.laser_power, 0.75)

    def test_laser_command_clamping(self):
        state = GimbalState()
        # Set laser power to 1.5 (should clamp to 1.0)
        packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", 1.5))
        state.update_from_command(packet)
        self.assertEqual(state.laser_power, 1.0)

        # Set laser power to -0.5 (should clamp to 0.0)
        packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", -0.5))
        state.update_from_command(packet)
        self.assertEqual(state.laser_power, 0.0)

    def test_laser_telemetry_encoding(self):
        state = GimbalState()
        state.laser_power = 0.42
        encoded_packet = state.get_laser_state_packet()
        
        # Packet structure: [Sync0, Sync1, ID, Len, DATA..., Checksum]
        # Data is at offset 4, length is at offset 3
        length = encoded_packet[3]
        self.assertEqual(length, 7)
        data = encoded_packet[4:4+length]
        
        num_lasers, laser_idx, flags, temp, wait_timer = struct.unpack(">BBHBH", data)
        self.assertEqual(num_lasers, 1)
        self.assertEqual(laser_idx, 1)
        self.assertTrue(flags & (1 << 15))  # Enabled

    def test_orion_cmd_rate_mode_pan_preserves_tilt(self):
        """Verify that sending a pan rate command with zero tilt rate does NOT overwrite tilt to 0.0°."""
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)
        self.assertEqual(state.target_tilt, -20.0)

        # Pan rate = 10 deg/s (0.1745 rad/s * 1000 = 175), tilt rate = 0, Mode = 0x10 (ORION_MODE_RATE)
        cmd_data = struct.pack(">hhBBB", 175, 0, 0x10, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_data))

        # Tilt must remain -20.0, pan must advance by 10 deg/s * 0.1s = 1.0 deg
        self.assertEqual(state.target_tilt, -20.0)
        self.assertAlmostEqual(state.target_pan, 1.0, delta=0.05)

    def test_orion_cmd_rate_mode_tilt_integration(self):
        """Verify that sending tilt rate commands integrates rate and zero rate preserves tilt."""
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)

        # Command tilt down at -10 deg/s (-0.1745 rad/s * 1000 = -175), Mode = 0x10
        cmd_down = struct.pack(">hhBBB", 0, -175, 0x10, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_down))

        # Target tilt should become -20.0 + (-10.0 * 0.1) = -21.0
        self.assertAlmostEqual(state.target_tilt, -21.0, delta=0.05)

        # Stop tilting (zero rate command: 0, 0, Mode = 0x10)
        cmd_stop = struct.pack(">hhBBB", 0, 0, 0x10, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_stop))

        # Target tilt must still be -21.0 (must NOT reset to 0.0!)
        self.assertAlmostEqual(state.target_tilt, -21.0, delta=0.05)

    def test_orion_cmd_position_mode(self):
        """Verify that position mode (Mode = 0x50) commands absolute angles."""
        import math
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)

        # Position mode: pan=15.0 deg (0.2618 rad * 1000 = 262), tilt=-35.0 deg (-0.6109 rad * 1000 = -611)
        pan_raw = int(math.radians(15.0) * 1000.0)
        tilt_raw = int(math.radians(-35.0) * 1000.0)
        cmd_pos = struct.pack(">hhBBB", pan_raw, tilt_raw, 0x50, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_pos))

        self.assertAlmostEqual(state.target_pan, 15.0, delta=0.1)
        self.assertAlmostEqual(state.target_tilt, -35.0, delta=0.1)

    def test_orion_cmd_extended_response(self):
        """Verify that OrionCmdExtended (10 bytes) produces a standard 28-byte OrionCmdExtendedResponse."""
        state = GimbalState(dt=0.1, tilt=-20.0, pan=10.0)

        # 10 bytes: pan_raw, tilt_raw, mode, stabilized, impulse, seq_num, flags
        cmd_ext = struct.pack(">hhBBBHB", 0, 0, 0x10, 1, 0, 101, 0)
        resp = state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_ext))

        self.assertIsNotNone(resp)
        # OrionPacket: [Sync0, Sync1, ID, Length, Payload(28), Checksum(2)] -> Total 34 bytes
        self.assertEqual(len(resp), 34)
        self.assertEqual(resp[2], OrionPktType.CMD)
        self.assertEqual(resp[3], 28)

    def test_zoom_command_preserves_tilt(self):
        """Verify that zooming does not alter gimbal tilt."""
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)
        self.assertEqual(state.target_tilt, -20.0)

        # Send 5x zoom packet (CAMERA_CMD / CAMERA_STATE)
        zoom_raw = int(5.0 * 100.0)
        zoom_pkt = OrionPacket(OrionPktType.CAMERA_CMD, struct.pack(">hh", zoom_raw, 0))
        state.update_from_command(zoom_pkt)

        self.assertEqual(state.camera_zoom, 5.0)
        self.assertEqual(state.target_tilt, -20.0)

    def test_orion_cmd_geo_rate_mode(self):
        """Verify that ORION_MODE_GEO_RATE (0x11) integrates slew rates, supports impulse time, echoes mode, and reports telemetry."""
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=10.0)

        # 1. Pan rate = 10 deg/s (175 mrad/s), tilt rate = 0, Mode = 0x11 (ORION_MODE_GEO_RATE)
        cmd_pan = struct.pack(">hhBBB", 175, 0, OrionMode.GEO_RATE, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_pan))
        self.assertEqual(state.mode, OrionMode.GEO_RATE)
        self.assertAlmostEqual(state.target_pan, 11.0, delta=0.05)
        self.assertEqual(state.target_tilt, -20.0)

        # 2. Tilt rate with impulse time (5 = 0.5s): tilt down at -10 deg/s (-175 mrad/s)
        cmd_impulse = struct.pack(">hhBBBH", 0, -175, OrionMode.GEO_RATE, 0, 5, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_impulse))
        # -20.0 + (-10.0 * 0.5) = -25.0
        self.assertAlmostEqual(state.target_tilt, -25.0, delta=0.05)

        # 3. Extended response echoes mode 0x11
        cmd_ext = struct.pack(">hhBBBHB", 0, 0, OrionMode.GEO_RATE, 1, 0, 102, 0)
        resp = state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_ext))
        self.assertIsNotNone(resp)
        self.assertEqual(resp[8], OrionMode.GEO_RATE)

        # 4. Geolocate telemetry reports mode 0x11 (17)
        telemetry = state.get_geolocate_telemetry_core_packet()
        self.assertEqual(telemetry[60], OrionMode.GEO_RATE)

    def test_orion_cmd_scene_mode(self):
        """Verify that ORION_MODE_SCENE (0x30) integrates slew rates, supports impulse time, echoes mode, and reports telemetry."""
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=10.0)

        # 1. Pan rate = 10 deg/s (175 mrad/s), tilt rate = 0, Mode = 0x30 (ORION_MODE_SCENE)
        cmd_pan = struct.pack(">hhBBB", 175, 0, OrionMode.SCENE, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_pan))
        self.assertEqual(state.mode, OrionMode.SCENE)
        self.assertAlmostEqual(state.target_pan, 11.0, delta=0.05)
        self.assertEqual(state.target_tilt, -20.0)

        # 2. Tilt rate with impulse time (5 = 0.5s): tilt down at -10 deg/s (-175 mrad/s)
        cmd_impulse = struct.pack(">hhBBBH", 0, -175, OrionMode.SCENE, 0, 5, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_impulse))
        # -20.0 + (-10.0 * 0.5) = -25.0
        self.assertAlmostEqual(state.target_tilt, -25.0, delta=0.05)

        # 3. Extended response echoes mode 0x30
        cmd_ext = struct.pack(">hhBBBHB", 0, 0, OrionMode.SCENE, 1, 0, 103, 0)
        resp = state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_ext))
        self.assertIsNotNone(resp)
        self.assertEqual(resp[8], OrionMode.SCENE)

        # 4. Geolocate telemetry reports mode 0x30 (48)
        telemetry = state.get_geolocate_telemetry_core_packet()
        self.assertEqual(telemetry[60], OrionMode.SCENE)

    def test_rate_modes_cancel_geopoint(self):
        """Verify that sending rate mode commands (0x10, 0x11, 0x30) cancels GEOPOINT mode."""
        from orion_shadow.core.protocol import OrionMode
        for mode in (OrionMode.RATE, OrionMode.GEO_RATE, OrionMode.SCENE):
            state = GimbalState()
            state.mode = OrionMode.GEOPOINT
            cmd = struct.pack(">hhBBB", 0, 0, mode, 0, 0)
            state.update_from_command(OrionPacket(OrionPktType.CMD, cmd))
            self.assertEqual(state.mode, mode)
            self.assertNotEqual(state.mode, OrionMode.GEOPOINT)

    def test_orion_cmd_disabled_mode(self):
        """Verify that ORION_MODE_DISABLED (0x00) disables motors and reports mode 0."""
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=10.0)
        state.physics.pan["vel"] = 15.0
        state.physics.tilt["vel"] = -10.0

        cmd_disabled = struct.pack(">hhBBB", 0, 0, OrionMode.DISABLED, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_disabled))

        self.assertEqual(state.mode, OrionMode.DISABLED)
        self.assertEqual(state.physics.pan["vel"], 0.0)
        self.assertEqual(state.physics.tilt["vel"], 0.0)

        # Telemetry reports mode 0 when disabled
        telemetry = state.get_geolocate_telemetry_core_packet()
        self.assertEqual(telemetry[60], 0)

    def test_orion_cmd_ffc_modes(self):
        """Verify ORION_MODE_FFC_AUTO (0x20) and ORION_MODE_FFC_MANUAL (0x21)."""
        import math
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=10.0)

        # 1. FFC Auto (0x20): drives to blackbody position
        cmd_auto = struct.pack(">hhBBB", 0, 0, OrionMode.FFC_AUTO, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_auto))
        self.assertEqual(state.mode, OrionMode.FFC_AUTO)
        self.assertTrue(state.ffc_active)
        self.assertEqual(state.target_pan, 0.0)
        self.assertEqual(state.target_tilt, min(25.0, state.tilt_max))
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x20)

        # 2. FFC Manual (0x21): drives to commanded payload position
        pan_raw = int(math.radians(12.0) * 1000.0)
        tilt_raw = int(math.radians(-15.0) * 1000.0)
        cmd_manual = struct.pack(">hhBBB", pan_raw, tilt_raw, OrionMode.FFC_MANUAL, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_manual))
        self.assertEqual(state.mode, OrionMode.FFC_MANUAL)
        self.assertAlmostEqual(state.target_pan, 12.0, delta=0.1)
        self.assertAlmostEqual(state.target_tilt, -15.0, delta=0.1)
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x21)

    def test_orion_cmd_track_mode(self):
        """Verify ORION_MODE_TRACK (0x31) normalized bounding and FOV proportional offset."""
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)

        # Command track box offset: norm_x = 0.25 (250), norm_y = -0.1 (-100)
        cmd_track = struct.pack(">hhBBB", 250, -100, OrionMode.TRACK, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_track))

        self.assertEqual(state.mode, OrionMode.TRACK)
        self.assertAlmostEqual(state.track_box[0], 0.25, delta=0.001)
        self.assertAlmostEqual(state.track_box[1], -0.1, delta=0.001)
        self.assertEqual(state.tracking_mode, 2)

        # Pan and tilt should adjust relative to FOV
        hfov, vfov = state.current_fov_deg
        expected_pan = 0.25 * hfov
        expected_tilt = -20.0 - (-0.1 * vfov)
        self.assertAlmostEqual(state.target_pan, expected_pan, delta=0.2)
        self.assertAlmostEqual(state.target_tilt, expected_tilt, delta=0.2)

        # Telemetry reports mode 0x31
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x31)

    def test_orion_cmd_calibration_and_null_gyros(self):
        """Verify ORION_MODE_CALIBRATION (0x40) and ORION_MODE_NULL_GYROS (0x41)."""
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=5.0)

        # Calibration (0x40)
        cmd_cal = struct.pack(">hhBBB", 0, 0, OrionMode.CALIBRATION, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_cal))
        self.assertEqual(state.mode, OrionMode.CALIBRATION)
        self.assertTrue(state.calibration_active)
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x40)

        # Null Gyros (0x41)
        cmd_null = struct.pack(">hhBBB", 0, 0, OrionMode.NULL_GYROS, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_null))
        self.assertEqual(state.mode, OrionMode.NULL_GYROS)
        self.assertTrue(state.gyro_calibrated)
        self.assertEqual(state.physics.pan["vel"], 0.0)
        self.assertEqual(state.physics.tilt["vel"], 0.0)
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x41)

    def test_orion_cmd_position_no_limits(self):
        """Verify ORION_MODE_POSITION_NO_LIMITS (0x51) allows angles beyond software soft limits."""
        import math
        from orion_shadow.core.protocol import OrionMode
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)

        # Soft limit for tilt is [-80, 28]. Command -88 degrees (-1.5359 rad * 1000 = -1536)
        pan_raw = int(math.radians(0.0) * 1000.0)
        tilt_raw = int(math.radians(-88.0) * 1000.0)
        cmd_no_limits = struct.pack(">hhBBB", pan_raw, tilt_raw, OrionMode.POSITION_NO_LIMITS, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_no_limits))

        self.assertEqual(state.mode, OrionMode.POSITION_NO_LIMITS)
        self.assertAlmostEqual(state.target_tilt, -88.0, delta=0.2)

        # Standard position mode (0x50) clamps to -80.0
        cmd_standard = struct.pack(">hhBBB", pan_raw, tilt_raw, OrionMode.POSITION, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_standard))
        self.assertEqual(state.target_tilt, -80.0)

    def test_orion_cmd_path_mode(self):
        """Verify ORION_MODE_PATH (0x70) command and ORION_PKT_PATH (0xD7) packet."""
        from orion_shadow.core.protocol import OrionMode, OrionPktType
        state = GimbalState(dt=0.1, tilt=-20.0, pan=0.0)

        # 1. Path command via CMD mode 0x70
        cmd_path = struct.pack(">hhBBB", 0, 0, OrionMode.PATH, 0, 0)
        state.update_from_command(OrionPacket(OrionPktType.CMD, cmd_path))
        self.assertEqual(state.mode, OrionMode.PATH)
        self.assertEqual(state.get_geolocate_telemetry_core_packet()[60], 0x70)

        # 2. Path packet (0xD7) with pointDown set
        path_pkt_data = bytes([0, 1]) # numPoints=0, pointDown=1
        state.update_from_command(OrionPacket(OrionPktType.PATH, path_pkt_data))
        self.assertEqual(state.mode, OrionMode.PATH)
        self.assertEqual(state.target_tilt, state.tilt_min)

if __name__ == '__main__':
    unittest.main()
