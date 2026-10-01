try:
    import pytest
except ImportError:
    pytest = None

import os
import math
from orion_shadow.core.protocol import OrionPacket, OrionPktType, ORION_SYNC0, ORION_SYNC1
from orion_shadow.core.engine import ProtocolEngine
from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.physics import PhysicsEngine

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

def test_terrain_engine_directory_search():
    # Pass directory path containing multiple .dt* files
    path = "tests/terrain_data"
    engine = TerrainEngine(dted_path=path)
    assert engine.enabled is True
    assert len(engine.tiles) >= 2
    # Ensure query on flat.dt0 works
    assert engine.get_elevation(34.5, -118.5) in (100.0, 500.0)

