
import pytest
import os
import math
from orion_shadow.server import TerrainEngine, GimbalState

def test_terrain_engine_disabled():
    engine = TerrainEngine(dted_path=None)
    assert engine.enabled is False
    assert engine.get_elevation(34.0, -118.0) == 0.0

def test_terrain_engine_binary_flat():
    # Path to the mock binary file we created
    path = "tests/terrain_data/flat.dt0"
    engine = TerrainEngine(dted_path=path)
    assert engine.enabled is True
    # Test center of the grid
    assert engine.get_elevation(34.5, -118.5) == 100.0

def test_terrain_engine_binary_slope():
    path = "tests/terrain_data/slope.dt1"
    engine = TerrainEngine(dted_path=path)
    assert engine.enabled is True
    # Test point at lat 34.5 (halfway through lat range 34-35)
    # elevation = (34.5 - 34.0) * 1000.0 = 500.0
    # Allow for small floating point precision error
    assert math.isclose(engine.get_elevation(34.5, -118.5), 500.0, abs_tol=1e-5)

def test_gimbal_altitude_update_binary():
    path = "tests/terrain_data/flat.dt0"
    engine = TerrainEngine(dted_path=path)
    state = GimbalState(dt=0.1, terrain_engine=engine)
    
    state.gps_lat = 34.5
    state.gps_lon = -118.5
    state.gps_alt = 0.0
    
    state.step()
    
    assert state.gps_alt == 100.0
