
import pytest
import struct
import asyncio
from orion_shadow.server import (
    OrionPacket, 
    ProtocolEngine, 
    GimbalState, 
    PhysicsEngine, 
    OrionPktType
)

def test_packet_encoding_and_checksum():
    # Test the Fletcher-16 mod 251 checksum implementation
    data = b"\x01\x02\x03"
    packet = OrionPacket(0x01, data)
    encoded = packet.encode()
    
    # Sync0, Sync1, ID, Len, Data, Checksum(2) = 4 + 3 + 2 = 9 bytes
    assert len(encoded) == 9
    assert encoded[0] == 0xD0
    assert encoded[1] == 0x0D
    assert encoded[2] == 0x01
    assert encoded[3] == 3
    assert encoded[4:7] == b"\x01\x02\x03"

def test_physics_engine_movement():
    # Test if physics engine actually moves towards a target
    physics = PhysicsEngine(dt=0.1)
    
    # Set a target
    physics.step(target_pan=10.0, target_tilt=0.0)
    
    # After one step, pan position should be > 0
    assert physics.pan["pos"] > 0
    assert physics.pan["vel"] > 0

    # Simulate movement over many steps
    for _ in range(100):
        physics.step(target_pan=10.0, target_tilt=0.0)
    
    # Should be very close to 10.0
    assert abs(physics.pan["pos"] - 10.0) < 0.1

def test_gimbal_state_logic():
    state = GimbalState(dt=0.1)
    
    # Test Initialization
    init_packet = OrionPacket(OrionPktType.INITIALIZE, b"")
    state.update_from_command(init_packet)
    assert state.initialized is True

    # Test Command (Pan/Tilt)
    # Target: pan=45.0, tilt=-45.0
    cmd_data = struct.pack(">ff", 45.0, -45.0)
    cmd_packet = OrionPacket(OrionPktType.CMD, cmd_data)
    state.update_from_command(cmd_packet)
    
    assert state.target_pan == 45.0
    assert state.target_tilt == -45.0

def test_telemetry_packet_format():
    state = GimbalState(dt=0.1)
    state.physics.pan["pos"] = 12.34
    state.physics.tilt["pos"] = -56.78
    
    packet = state.get_telemetry_packet()
    # Decode it to verify
    engine = ProtocolEngine()
    parsed = engine.parse(packet)
    
    assert parsed.packet_id == OrionPktType.POSITIONS
    # Decode the data part
    pan, tilt = struct.unpack(">ff", parsed.data)
    assert abs(pan - 12.34) < 0.001
    assert abs(tilt - (-56.78)) < 0.001
