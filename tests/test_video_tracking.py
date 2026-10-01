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


def test_video_server_dynamic_updates_tilt_and_heading():
    from orion_shadow.engine.video_server import VideoServer
    import numpy as np

    state = GimbalState()
    server = VideoServer(state)

    state.gps_lat = 39.7774
    state.gps_lon = -84.0819
    state.gps_alt = 2000.0
    state.aircraft_heading = 90.0
    state.target_tilt = -30.0
    state.physics.tilt["pos"] = -30.0

    frame_base = server._generate_synthetic_background()
    assert isinstance(frame_base, np.ndarray)

    # Change tilt: frame must update
    state.target_tilt = 10.0
    state.physics.tilt["pos"] = 10.0
    frame_tilt = server._generate_synthetic_background()
    assert not np.array_equal(frame_base, frame_tilt), "Video frame should update when gimbal tilts"

    # Change heading: frame must update
    state.aircraft_heading = 180.0
    frame_hdg = server._generate_synthetic_background()
    assert not np.array_equal(frame_tilt, frame_hdg), "Video frame should update when aircraft/gimbal heading changes"


def test_video_server_dynamic_updates_adsb_movement():
    from orion_shadow.engine.video_server import VideoServer
    import numpy as np

    state = GimbalState()
    server = VideoServer(state)

    state.gps_lat = 39.7774
    state.gps_lon = -84.0819
    state.gps_alt = 3000.0
    state.aircraft_heading = 240.0
    state.target_tilt = -20.0
    state.physics.tilt["pos"] = -20.0

    frame_initial = server._generate_synthetic_background()

    # Move aircraft forward along flight path
    state.gps_lat += 0.005
    state.gps_lon -= 0.005
    frame_moved = server._generate_synthetic_background()

    assert not np.array_equal(frame_initial, frame_moved), "Video frame should update as attached aircraft flies"


def test_video_server_hud_rendering():
    from orion_shadow.engine.video_server import VideoServer
    import numpy as np

    state = GimbalState()
    server = VideoServer(state)
    state.gps_lat = 39.7774
    state.gps_lon = -84.0819
    state.gps_alt = 1500.0

    bg = server._generate_synthetic_background()
    hud = server._draw_hud(bg.copy())
    assert not np.array_equal(bg, hud), "HUD should render overlay on the video frame"


def test_video_server_pure_python_fallback():
    import orion_shadow.engine.video_server as vs

    orig_np = vs.np
    try:
        vs.np = None
        state = GimbalState()
        server = vs.VideoServer(state)
        frame = server._generate_synthetic_background()
        assert isinstance(frame, bytes)
        assert len(frame) == 640 * 480 * 3
    finally:
        vs.np = orig_np
