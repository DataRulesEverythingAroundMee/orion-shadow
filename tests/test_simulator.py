
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
    data = struct.pack(">ff", 10.5, -20.0)
    packet = OrionPacket(0x01, data)
    encoded = packet.encode()
    
    # Check Sync bytes
    assert encoded[0] == 0xD0
    assert encoded[1] == 0x0D
    # Check ID
    assert encoded[2] == 0x01
    # Check Length
    assert encoded[3] == 8
    # Check Data
    assert encoded[4:12] == data

def test_protocol_engine_parse():
    engine = ProtocolEngine()
    data = struct.pack(">ff", 10.5, -20.0)
    packet = OrionPacket(0x01, data)
    encoded = packet.encode()
    
    parsed = engine.parse(encoded)
    assert parsed.packet_id == 0x01
    assert parsed.data == data

def test_gimbal_state_updates():
    state = GimbalState(dt=0.1)
    # CMD packet (0x01) with pan=45.0, tilt=-10.0
    data = struct.pack(">ff", 45.0, -10.0)
    packet = OrionPacket(0x01, data)
    
    state.update_from_command(packet)
    
    # The physics engine integrates over time. 
    # We need to call step() to move the position towards the target.
    # Increase iterations to ensure convergence given damping
    for _ in range(1000):
        state.step()
    
    # After 1000 steps, it should be very close to 45.0
    assert abs(state.physics.pan["pos"] - 45.0) < 0.1
    assert abs(state.physics.tilt["pos"] - (-10.0)) < 0.1

    # Initialize packet (0x00)
    init_packet = OrionPacket(0x00, b"")
    state.update_from_command(init_packet)
    assert state.initialized is True

def test_navigation_data_ingestion():
    state = GimbalState(dt=0.1)
    
    # Test GPS Ingestion
    gps_data = struct.pack(">fff", 34.0522, -118.2437, 150.5)
    gps_packet = OrionPacket(OrionPktType.GPS_DATA, gps_data)
    state.update_from_command(gps_packet)
    
    assert abs(state.gps_lat - 34.0522) < 0.0001
    assert abs(state.gps_lon - (-118.2437)) < 0.0001
    assert abs(state.gps_alt - 150.5) < 0.0001

    # Test Attitude/Heading Ingestion
    heading_data = struct.pack(">fff", 90.0, 5.0, -2.0) # Heading, Roll, Pitch
    heading_packet = OrionPacket(OrionPktType.EXT_HEADING_DATA, heading_data)
    state.update_from_command(heading_packet)
    
    assert state.aircraft_heading == 90.0
    assert state.aircraft_roll == 5.0
    assert state.aircraft_pitch == -2.0

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
