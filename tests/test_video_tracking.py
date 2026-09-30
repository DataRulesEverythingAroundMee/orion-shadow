import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_video_options_update():
    state = GimbalState()
    # width(u16), height(u16), fps(u8)
    # 1280, 720, 60
    data = struct.pack(">HHB", 1280, 720, 60)
    packet = OrionPacket(OrionPktType.VIDEO_OPTIONS, data)
    state.update_from_command(packet)
    
    assert state.video_resolution_width == 1280
    assert state.video_resolution_height == 720
    assert state.video_fps == 60

def test_tracking_options_update():
    state = GimbalState()
    # target_id(u32), mode(u8)
    # ID=42, Mode=2
    data = struct.pack(">IB", 42, 2)
    packet = OrionPacket(OrionPktType.TRACK_OPTIONS, data)
    state.update_from_command(packet)
    
    assert state.tracking_target_id == 42
    assert state.tracking_mode == 2
