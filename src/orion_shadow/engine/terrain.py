import os
import struct
import math
import numpy as np
from typing import Optional

class TerrainEngine:
    """Handles elevation queries from binary DTED data (.dt0, .dt1, .dt2)."""
    # DTED Header Format (Simplified for Simulation)
    # 0-3: Magic 'DTED'
    # 4-7: Rows (uint32)
    # 8-11: Cols (uint32)
    # 12-19: Lat_min (float64)
    # 20-27: Lat_max (float64)
    # 28-35: Lon_min (float64)
    # 36-43: Lon_max (float64)
    # 44-127: Padding
    HEADER_SIZE = 128
    MAGIC = b'DTED'

    def __init__(self, dted_path: Optional[str] = None):
        self.enabled = dted_path is not None
        self.dted_path = dted_path
        self.rows = 0
        self.cols = 0
        self.lat_min = 0.0
        self.lat_max = 0.0
        self.lon_min = 0.0
        self.lon_max = 0.0
        self.grid = None
        
        if self.enabled:
            print(f"[*] TerrainEngine enabled with path: {self.dted_path}")
            self._load_dted()
        else:
            print("[*] TerrainEngine disabled (no DTED path provided)")

    def _load_dted(self):
        if not os.path.exists(self.dted_path):
            print(f"[!] DTED file not found: {self.dted_path}")
            return

        try:
            with open(self.dted_path, 'rb') as f:
                header = f.read(self.HEADER_SIZE)
                if len(header) < self.HEADER_SIZE or header[:4] != self.MAGIC:
                    raise ValueError("Invalid DTED magic header")
                
                self.rows, self.cols = struct.unpack(">II", header[4:12])
                self.lat_min, self.lat_max = struct.unpack(">dd", header[12:28])
                self.lon_min, self.lon_max = struct.unpack(">dd", header[28:44])
                
                # Read elevation data (uint16)
                data_bytes = f.read()
                self.grid = np.frombuffer(data_bytes, dtype='>u2').reshape((self.rows, self.cols))
                print(f"[*] Loaded DTED grid: {self.rows}x{self.cols} ({len(data_bytes)} bytes)")
        except Exception as e:
            print(f"[!] Failed to parse DTED file: {e}")

    def get_elevation(self, lat: float, lon: float) -> float:
        if not self.enabled or self.grid is None:
            return 0.0
        
        # Clamp input to bounds
        lat = max(self.lat_min, min(self.lat_max, lat))
        lon = max(self.lon_min, min(self.lon_max, lon))

        # Map Lat/Lon to Grid index
        # Row index (Lat)
        row_frac = (lat - self.lat_min) / (self.lat_max - self.lat_min)
        # Col index (Lon)
        col_frac = (lon - self.lon_min) / (self.lon_max - self.lon_min)
        
        row = row_frac * (self.rows - 1)
        col = col_frac * (self.cols - 1)

        # Bilinear Interpolation
        r0, c0 = int(math.floor(row)), int(math.floor(col))
        r1, c1 = min(r0 + 1, self.rows - 1), min(c0 + 1, self.cols - 1)
        
        dr = row - r0
        dc = col - c0
        
        v00 = self.grid[r0, c0]
        v01 = self.grid[r0, c1]
        v10 = self.grid[r1, c0]
        v11 = self.grid[r1, c1]
        
        # Interpolate
        res = (1-dr)*(1-dc)*v00 + (1-dr)*dc*v01 + dr*(1-dc)*v10 + dr*dc*v11
        return float(res)
