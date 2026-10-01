import os
import struct
import math
from typing import Optional, List, Any

try:
    import numpy as np
except ImportError:
    np = None


class DTEDTile:
    """Represents a single parsed DTED elevation cell."""
    def __init__(self, file_path: str, lat_min: float, lat_max: float, 
                 lon_min: float, lon_max: float, rows: int, cols: int, grid: Any):
        self.file_path = file_path
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.lon_min = lon_min
        self.lon_max = lon_max
        self.rows = rows
        self.cols = cols
        self.grid = grid

    def contains(self, lat: float, lon: float) -> bool:
        return self.lat_min <= lat <= self.lat_max and self.lon_min <= lon <= self.lon_max

    def _get_val(self, r: int, c: int) -> float:
        if hasattr(self.grid, 'shape'):
            return float(self.grid[r, c])
        return float(self.grid[r][c])

    def get_elevation(self, lat: float, lon: float) -> float:
        # Clamp input to bounds of this tile
        lat = max(self.lat_min, min(self.lat_max, lat))
        lon = max(self.lon_min, min(self.lon_max, lon))

        # Map Lat/Lon to Grid index
        row_frac = (lat - self.lat_min) / (self.lat_max - self.lat_min) if self.lat_max != self.lat_min else 0.0
        col_frac = (lon - self.lon_min) / (self.lon_max - self.lon_min) if self.lon_max != self.lon_min else 0.0

        row = row_frac * (self.rows - 1)
        col = col_frac * (self.cols - 1)

        # Bilinear Interpolation
        r0, c0 = int(math.floor(row)), int(math.floor(col))
        r1, c1 = min(r0 + 1, self.rows - 1), min(c0 + 1, self.cols - 1)

        dr = row - r0
        dc = col - c0

        v00 = self._get_val(r0, c0)
        v01 = self._get_val(r0, c1)
        v10 = self._get_val(r1, c0)
        v11 = self._get_val(r1, c1)

        res = (1 - dr) * (1 - dc) * v00 + (1 - dr) * dc * v01 + dr * (1 - dc) * v10 + dr * dc * v11
        return float(res)


class TerrainEngine:
    """Handles elevation queries from binary DTED data (.dt0, .dt1, .dt2).
    
    Supports loading from a single file or scanning a directory tree for all DTED files.
    """
    HEADER_SIZE = 128
    MAGIC = b'DTED'

    def __init__(self, dted_path: Optional[str] = None):
        self.enabled = dted_path is not None
        self.dted_path = dted_path
        self.tiles: List[DTEDTile] = []
        
        # Backwards compatibility attributes
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

    def _parse_dted_coord(self, s: str, is_lat: bool = False) -> float:
        """Parses DTED DMS coordinates (e.g. '1190000W' or '0340000N')."""
        s = s.strip()
        hemi = s[-1].upper()
        num = s[:-1]
        if len(num) >= 7:
            deg = float(num[0:3])
            minutes = float(num[3:5])
            sec = float(num[5:7])
        else:
            deg = float(num[0:2])
            minutes = float(num[2:4])
            sec = float(num[4:6]) if len(num) >= 6 else 0.0
        val = deg + minutes / 60.0 + sec / 3600.0
        return -val if hemi in ('S', 'W') else val

    def _load_single_file(self, file_path: str) -> Optional[DTEDTile]:
        """Attempts to load a DTED tile from a mock binary or standard DTED file."""
        try:
            with open(file_path, 'rb') as f:
                header = f.read(self.HEADER_SIZE)
                if len(header) < 128:
                    return None

                # 1. Check for simulation mock DTED format
                if header[:4] == self.MAGIC:
                    rows, cols = struct.unpack(">II", header[4:12])
                    lat_min, lat_max = struct.unpack(">dd", header[12:28])
                    lon_min, lon_max = struct.unpack(">dd", header[28:44])

                    data_bytes = f.read()
                    if np is not None:
                        grid = np.frombuffer(data_bytes, dtype='>u2').reshape((rows, cols))
                    else:
                        num_points = rows * cols
                        vals = struct.unpack(f">{num_points}H", data_bytes[:num_points * 2])
                        grid = [list(vals[r * cols:(r + 1) * cols]) for r in range(rows)]
                    return DTEDTile(file_path, lat_min, lat_max, lon_min, lon_max, rows, cols, grid)

                # 2. Check for standard MIL-PRF-89020B DTED format (UHL header)
                elif header[:3] == b'UHL':
                    lon_str = header[4:12].decode('ascii', errors='ignore')
                    lat_str = header[12:20].decode('ascii', errors='ignore')
                    cols = int(header[47:51].decode('ascii', errors='ignore'))
                    rows = int(header[51:55].decode('ascii', errors='ignore'))

                    lon_sw = self._parse_dted_coord(lon_str, is_lat=False)
                    lat_sw = self._parse_dted_coord(lat_str, is_lat=True)
                    lat_min, lat_max = lat_sw, lat_sw + 1.0
                    lon_min, lon_max = lon_sw, lon_sw + 1.0

                    # Data records begin at offset 3428 (UHL 80 + DSI 648 + ACC 2700)
                    record_len = 8 + rows * 2 + 4
                    if np is not None:
                        grid = np.zeros((rows, cols), dtype=np.float32)
                        for c in range(cols):
                            f.seek(3428 + c * record_len + 8)
                            col_data = np.frombuffer(f.read(rows * 2), dtype='>i2').astype(np.float32)
                            col_data[col_data < -1000] = 0.0
                            grid[:, c] = col_data
                    else:
                        grid = [[0.0] * cols for _ in range(rows)]
                        for c in range(cols):
                            f.seek(3428 + c * record_len + 8)
                            raw = f.read(rows * 2)
                            unpacked = struct.unpack(f">{rows}h", raw)
                            for r in range(rows):
                                val = float(unpacked[r])
                                grid[r][c] = 0.0 if val < -1000 else val

                    return DTEDTile(file_path, lat_min, lat_max, lon_min, lon_max, rows, cols, grid)

        except Exception as e:
            # File is not a valid DTED file or failed to read
            return None
        return None

    def _find_dted_files(self, base_path: str) -> List[str]:
        """Discovers all candidate DTED files from a path or directory tree."""
        if os.path.isfile(base_path):
            return [base_path]
        if not os.path.isdir(base_path):
            return []

        matched = []
        for root, _, files in os.walk(base_path):
            for file in sorted(files):
                ext = os.path.splitext(file)[1].lower()
                if ext in ('.dt0', '.dt1', '.dt2', '.dted'):
                    matched.append(os.path.join(root, file))

        # If no standard extension matched, search all regular files
        if not matched:
            for root, _, files in os.walk(base_path):
                for file in sorted(files):
                    fp = os.path.join(root, file)
                    if os.path.isfile(fp):
                        matched.append(fp)
        return matched

    def _load_dted(self):
        if not os.path.exists(self.dted_path):
            print(f"[!] DTED path not found: {self.dted_path}")
            return

        candidates = self._find_dted_files(self.dted_path)
        if not candidates:
            print(f"[!] No candidate DTED files found in: {self.dted_path}")
            return

        for path in candidates:
            tile = self._load_single_file(path)
            if tile is not None:
                self.tiles.append(tile)

        if not self.tiles:
            print(f"[!] No valid DTED files could be loaded from: {self.dted_path}")
            return

        # Setup primary reference attributes for backwards compatibility
        self.grid = self.tiles[0].grid
        self.rows = self.tiles[0].rows
        self.cols = self.tiles[0].cols
        self.lat_min = min(t.lat_min for t in self.tiles)
        self.lat_max = max(t.lat_max for t in self.tiles)
        self.lon_min = min(t.lon_min for t in self.tiles)
        self.lon_max = max(t.lon_max for t in self.tiles)

        print(f"[*] Loaded {len(self.tiles)} DTED tile(s) from {self.dted_path} "
              f"(Lat: [{self.lat_min:.2f}, {self.lat_max:.2f}], Lon: [{self.lon_min:.2f}, {self.lon_max:.2f}])")

    def get_elevation(self, lat: float, lon: float) -> float:
        if not self.enabled or not self.tiles:
            return 0.0

        # Query the tile containing coordinates
        for tile in self.tiles:
            if tile.contains(lat, lon):
                return tile.get_elevation(lat, lon)

        # Fallback to nearest tile
        closest_tile = min(
            self.tiles,
            key=lambda t: (
                max(0.0, t.lat_min - lat, lat - t.lat_max) ** 2 +
                max(0.0, t.lon_min - lon, lon - t.lon_max) ** 2
            )
        )
        return closest_tile.get_elevation(lat, lon)
