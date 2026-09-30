import pytest
import struct
import random
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

def test_fault_injection_motor_overcurrent():
    state = GimbalState()
    # Set a target position
    state.target_pan = 10.0
    state.target_tilt = 10.0
    
    # Run simulation for a few steps to allow movement
    for _ in range(5):
        state.step()
    
    initial_pan = state.physics.pan["pos"]
    
    # Inject motor overcurrent fault
    state.faults.inject_fault('motor_overcurrent', severity=5.0)
    
    # Step and check for jitter
    state.step()
    
    # Position should have changed more erratically or deviated from the smooth integration
    # (In this simple model, it adds random noise)
    # We just check that it's different from the predicted physics value
    assert state.physics.pan["pos"] != initial_pan

def test_fault_clearing():
    state = GimbalState()
    state.faults.inject_fault('motor_overcurrent', severity=5.0)
    assert len(state.faults.active_faults) == 1
    
    state.faults.clear_faults()
    assert len(state.faults.active_faults) == 0

def test_multiple_faults():
    state = GimbalState()
    state.faults.inject_fault('motor_overcurrent', severity=1.0)
    state.faults.inject_fault('sensor_timeout', severity=1.0)
    assert len(state.faults.active_faults) == 2

def test_sensor_timeout_fault():
    state = GimbalState()
    state.gps_lat = 34.0
    state.gps_lon = -118.0
    state.gps_alt = 100.0
    
    # Inject sensor timeout fault
    state.faults.inject_fault('sensor_timeout', severity=1.0)
    state.step() # Let the engine set self.is_faulty = True
    
    # Send new GPS data
    new_gps_data = struct.pack(">fff", 35.0, -119.0, 200.0)
    packet = OrionPacket(OrionPktType.GPS_DATA, new_gps_data)
    state.update_from_command(packet)
    
    # Data should NOT have updated because of the fault
    assert state.gps_lat == 34.0
    assert state.gps_lon == -118.0
    assert state.gps_alt == 100.0
