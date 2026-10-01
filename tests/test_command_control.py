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

if __name__ == '__main__':
    unittest.main()
