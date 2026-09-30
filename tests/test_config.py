import pytest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_uart_config_update():
    state = GimbalState()
    # Set baud rate to 921600
    packet = OrionPacket(OrionPktType.UART_CONFIG, struct.pack(">I", 921600))
    state.update_from_command(packet)
    assert state.baud_rate == 921600

def test_limits_update():
    state = GimbalState()
    # Set new pan/tilt limits: pan=30.0, tilt=30.0
    packet = OrionPacket(OrionPktType.LIMITS, struct.pack(">ff", 30.0, 30.0))
    state.update_from_command(packet)
    assert state.pan_limit == 30.0
    assert state.tilt_limit == 30.0

def test_cmd_respects_limits():
    state = GimbalState()
    # Set limits to 10.0
    packet = OrionPacket(OrionPktType.LIMITS, struct.pack(">ff", 10.0, 10.0))
    state.update_from_command(packet)
    
    # Send command for 45.0 (exceeding limit)
    cmd_packet = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 45.0, 45.0))
    state.update_from_command(cmd_packet)
    assert state.target_pan == 10.0
    assert state.target_tilt == 10.0

def test_cmd_respects_dynamic_limits():
    state = GimbalState()
    # Send command for 45.0 (initial limit is 45.0)
    cmd_packet = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 45.0, 45.0))
    state.update_from_command(cmd_packet)
    assert state.target_pan == 45.0
    
    # Change limits to 20.0
    limit_packet = OrionPacket(OrionPktType.LIMITS, struct.pack(">ff", 20.0, 20.0))
    state.update_from_command(limit_packet)
    
    # Next command should be clamped to new limits
    cmd_packet_new = OrionPacket(OrionPktType.CMD, struct.pack(">ff", 45.0, 45.0))
    state.update_from_command(cmd_packet_new)
    assert state.target_pan == 20.0
    assert state.target_tilt == 20.0
