import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_lifecycle_reset():
    state = GimbalState(dt=0.5)
    state.initialized = True
    state.camera_id = 5
    state.laser_power = 0.8
    
    # Reset command
    reset_packet = OrionPacket(OrionPktType.RESET, b'')
    state.update_from_command(reset_packet)
    
    assert state.initialized is True
    assert state.camera_id == 0
    assert state.laser_power == 0.0
    assert state.dt == 0.5

def test_startup_cmd():
    state = GimbalState()
    state.initialized = False
    
    startup_packet = OrionPacket(OrionPktType.STARTUP_CMD, b'')
    state.update_from_command(startup_packet)
    
    assert state.initialized is True

def test_diagnostics_telemetry():
    state = GimbalState(dt=1.0)
    state.uptime = 123.45
    state.error_count = 5
    state.fault_count = 2
    
    diag_packet = state.get_diagnostics_packet()
    
    # Packet structure: [Sync0, Sync1, ID, Len, uptime(f32), error(u32), fault(u32)] + Checksum
    # Header (4) + Payload (12) + Checksum (2) = 18 bytes
    assert len(diag_packet) == 18
    
    # Unpack payload (offset 4, length 12)
    uptime, error, fault = struct.unpack(">fII", diag_packet[4:16])
    
    assert abs(uptime - 123.45) < 0.01
    assert error == 5
    assert fault == 2
