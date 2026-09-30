import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_camera_switch_and_cmd():
    state = GimbalState()
    
    # 1. Test Switch
    switch_packet = OrionPacket(OrionPktType.CAMERA_SWITCH, b'\x02')
    state.update_from_command(switch_packet)
    assert state.camera_id == 2
    assert state.camera_ready is False
    
    # 2. Test Command (Zoom/Focus)
    # zoom=2.5, focus=0.5
    cmd_data = struct.pack(">ff", 2.5, 0.5)
    cmd_packet = OrionPacket(OrionPktType.CAMERA_CMD, cmd_data)
    state.update_from_command(cmd_packet)
    
    assert state.camera_zoom == 2.5
    assert state.camera_focus == 0.5
    assert state.camera_ready is True

def test_camera_telemetry():
    state = GimbalState()
    state.camera_zoom = 3.0
    state.camera_focus = 0.7
    state.camera_ready = True
    
    telemetry_bytes = state.get_camera_state_packet()
    
    # Decode to verify
    # Packet structure: [SYNC0, SYNC1, PKT_ID, LEN, ZOOM, FOCUS, READY] + CHECKSUM
    # We skip sync and header (4 bytes)
    # The packet.encode() adds header and checksum.
    # Let's use the protocol logic to decode or just check content.
    
    # Re-parse the payload (manually since OrionPacket doesn't have decode)
    # header is 4 bytes, payload is 9 bytes, checksum is 2 bytes.
    # Total = 15 bytes.
    assert len(telemetry_bytes) == 15
    
    # Unpack payload part: zoom (f32), focus (f32), ready (u8)
    # Offset 4 is the start of payload
    zoom, focus, ready = struct.unpack(">ffB", telemetry_bytes[4:13])
    
    assert zoom == 3.0
    assert abs(focus - 0.7) < 1e-6
    assert ready == 1
