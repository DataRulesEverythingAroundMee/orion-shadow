import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_cameras_query_and_telemetry():
    state = GimbalState()
    state.camera_id = 5
    
    # Test camera query packet generation
    cam_packet = state.get_cameras_packet()
    # The encode() method returns bytes. We need to extract the payload.
    # Packet structure: Sync0(1), Sync1(1), ID(1), Length(1), Payload(L), Checksum(2)
    # Total len = 6 + L
    payload_len = len(cam_packet) - 6
    payload = cam_packet[4:4+payload_len]
    
    assert len(payload) == 3
    
    # Unpack: Count, ID1, ID2
    count, id1, id2 = struct.unpack(">BBB", payload)
    assert count == 2
    assert id1 == 5
    assert id2 == 0

def test_faults_telemetry():
    state = GimbalState()
    
    # Test no faults
    fault_packet = state.get_faults_packet()
    payload_len = len(fault_packet) - 6
    payload = fault_packet[4:4+payload_len]
    
    assert len(payload) == 1
    count = struct.unpack(">B", payload)[0]
    assert count == 0
    
    # Test with faults
    state.faults.inject_fault('motor_overcurrent', 0.5)
    state.faults.inject_fault('sensor_timeout', 1.0)
    
    fault_packet = state.get_faults_packet()
    payload_len = len(fault_packet) - 6
    payload = fault_packet[4:4+payload_len]
    
    assert len(payload) == 3
    count, f1, f2 = struct.unpack(">BBB", payload)
    assert count == 2
    assert f1 in [1, 2, 3]
    assert f2 in [1, 2, 3]
