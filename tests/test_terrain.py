
import pytest
from orion_shadow.server import TerrainEngine, GimbalState

def test_terrain_engine_disabled():
    engine = TerrainEngine(dted_path=None)
    assert engine.enabled is False
    assert engine.get_elevation(34.0, -118.0) == 0.0

def test_terrain_engine_enabled_lowland():
    # Using the lowland.csv we created
    engine = TerrainEngine(dted_path="tests/terrain_data/lowland.csv")
    assert engine.enabled is True
    # Test nearest neighbor for 34.0, -118.0
    assert engine.get_elevation(34.0, -118.0) == 10.0
    # Test nearest neighbor for 34.1, -118.1
    assert engine.get_elevation(34.1, -118.1) == 18.0

def test_terrain_engine_enabled_mountain():
    engine = TerrainEngine(dted_path="tests/terrain_data/mountain.csv")
    assert engine.enabled is True
    assert engine.get_elevation(34.0, -118.0) == 1500.0
    assert engine.get_elevation(34.1, -118.1) == 3000.0

def test_gimbal_altitude_update():
    # Verify GimbalState correctly uses TerrainEngine to update altitude
    engine = TerrainEngine(dted_path="tests/terrain_data/lowland.csv")
    state = GimbalState(dt=0.1, terrain_engine=engine)
    
    # Set GPS to a known point in lowland.csv
    state.gps_lat = 34.0
    state.gps_lon = -118.0
    state.gps_alt = 500.0 # Start with a different altitude
    
    # Step the simulation
    state.step()
    
    # Altitude should now be updated from terrain
    assert state.gps_alt == 10.0
