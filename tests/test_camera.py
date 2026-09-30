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

def test_camera_state_telemetry():
    state = GimbalState()
    state.camera_id = 1
    state.camera_zoom = 5.5
    state.camera_focus = 12.3
    state.camera_ready = True
    
    packet = state.get_camera_state_packet()
    
    # Packet structure: [Sync0, Sync1, ID, Len, DATA (zoom:f32, focus:f32, ready:u8), Checksum]
    length = packet[3]
    data = packet[4:4+length]
    
    zoom, focus, ready = struct.unpack(">ffB", data)
    assert abs(zoom - 5.5) < 0.0001
    assert abs(focus - 12.3) < 0.0001
    assert ready == 1

def test_camera_state_not_ready():
    state = GimbalState()
    state.camera_ready = False
    
    packet = state.get_camera_state_packet()
    length = packet[3]
    data = packet[4:4+length]
    
    _, _, ready = struct.unpack(">ffB", data)
    assert ready == 0
