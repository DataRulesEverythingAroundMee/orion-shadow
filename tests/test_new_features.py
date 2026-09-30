import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_laser_command_and_state():
    state = GimbalState()
    # Test laser power command
    power_val = 0.75
    cmd_data = struct.pack(">f", power_val)
    packet = OrionPacket(OrionPktType.LASER_CMD, cmd_data)
    state.update_from_command(packet)
    assert state.laser_power == power_val
    
    # Test telemetry
    telemetry = state.get_laser_state_packet()
    # Unpack: check if the power value is in the packet
    # The packet structure is: header(4) + payload(4) + checksum(2) = 10 bytes
    # Payload is at offset 4
    unpacked_power = struct.unpack(">f", telemetry[4:8])[0]
    assert abs(unpacked_power - power_val) < 1e-5

def test_faults_broadcast_diagnostics():
    state = GimbalState()
    # Simulate a fault
    state.fault_count = 5
    state.error_count = 2
    state.uptime = 123.45
    
    diag_packet = state.get_diagnostics_packet()
    # Header(4) + uptime(4) + error(4) + fault(4) + checksum(2) = 18 bytes
    # Payload: uptime(4), error(4), fault(4)
    uptime, errors, faults = struct.unpack(">fII", diag_packet[4:16])
    
    assert abs(uptime - 123.45) < 1e-3
    assert errors == 2
    assert faults == 5

def test_video_and_tracking_command_ingestion():
    state = GimbalState()
    
    # Video options
    video_data = struct.pack(">HHB", 1280, 720, 60)
    state.update_from_command(OrionPacket(OrionPktType.VIDEO_OPTIONS, video_data))
    assert state.video_resolution_width == 1280
    assert state.video_resolution_height == 720
    assert state.video_fps == 60
    
    # Tracking options
    track_data = struct.pack(">IB", 99, 2)
    state.update_from_command(OrionPacket(OrionPktType.TRACK_OPTIONS, track_data))
    assert state.tracking_target_id == 99
    assert state.tracking_mode == 2

def test_camera_command_ingestion():
    state = GimbalState()
    # Camera Command (Zoom, Focus)
    cam_cmd_data = struct.pack(">ff", 5.0, 0.5)
    state.update_from_command(OrionPacket(OrionPktType.CAMERA_CMD, cam_cmd_data))
    assert state.camera_zoom == 5.0
    assert state.camera_focus == 0.5
    assert state.camera_ready is True
