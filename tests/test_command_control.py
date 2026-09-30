import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_camera_switch():
    state = GimbalState()
    # Switch to camera 2
    packet = OrionPacket(OrionPktType.CAMERA_SWITCH, b'\x02')
    state.update_from_command(packet)
    assert state.camera_id == 2

def test_laser_command():
    state = GimbalState()
    # Set laser power to 0.75
    power = 0.75
    packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", power))
    state.update_from_command(packet)
    assert state.laser_power == 0.75

def test_laser_command_clamping():
    state = GimbalState()
    # Set laser power to 1.5 (should clamp to 1.0)
    packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", 1.5))
    state.update_from_command(packet)
    assert state.laser_power == 1.0

    # Set laser power to -0.5 (should clamp to 0.0)
    packet = OrionPacket(OrionPktType.LASER_CMD, struct.pack(">f", -0.5))
    state.update_from_command(packet)
    assert state.laser_power == 0.0

def test_laser_telemetry_encoding():
    state = GimbalState()
    state.laser_power = 0.42
    encoded_packet = state.get_laser_state_packet()
    
    # Packet structure: [Sync0, Sync1, ID, Len, DATA..., Checksum]
    # Data is at offset 4, length is at offset 3
    length = encoded_packet[3]
    data = encoded_packet[4:4+length]
    
    power = struct.unpack(">f", data)[0]
    assert abs(power - 0.42) < 0.0001
