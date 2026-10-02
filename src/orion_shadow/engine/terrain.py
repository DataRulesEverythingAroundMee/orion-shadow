import os
import struct
import math
import logging
from typing import Optional, List, Any

logger = logging.getLogger(__name__)

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
        # Fix 4a: Pre-cast to float32 once at load time so get_elevations never
        # pays the np.asarray(dtype=float32) conversion cost at render time
        # (that call occurred on every one of the 20 ray-march steps per frame).
        if np is not None:
            if hasattr(grid, 'dtype') and grid.dtype == np.float32:
                self.grid = grid
            elif hasattr(grid, 'shape'):
                self.grid = np.asarray(grid, dtype=np.float32)
            else:
                self.grid = np.array(grid, dtype=np.float32)
        else:
            self.grid = grid

    def contains(self, lat: float, lon: float) -> bool:
        return self.lat_min <= lat <= self.lat_max and self.lon_min <= lon <= self.lon_max

    def _get_val(self, r: int, c: int) -> float:
        if hasattr(self.grid, 'shape'):
            return float(self.grid[r, c])
        return float(self.grid[r][c])

    def get_elevation(self, lat: float, lon: float) -> float:
        # Fix 4b: delegate to the vectorised batch path to avoid Python-level
        # _get_val() loops and share the same fast NumPy bilinear code.
        if np is not None:
            lats = np.array([lat], dtype=np.float32)
            lons = np.array([lon], dtype=np.float32)
            return float(self.get_elevations(lats, lons)[0])
        # Pure-Python fallback (numpy unavailable)
        lat = max(self.lat_min, min(self.lat_max, lat))
        lon = max(self.lon_min, min(self.lon_max, lon))
        lat_range = self.lat_max - self.lat_min
        lon_range = self.lon_max - self.lon_min
        row_frac = (lat - self.lat_min) / lat_range if lat_range != 0 else 0.0
        col_frac = (lon - self.lon_min) / lon_range if lon_range != 0 else 0.0
        row = row_frac * (self.rows - 1)
        col = col_frac * (self.cols - 1)
        r0, c0 = int(math.floor(row)), int(math.floor(col))
        r1, c1 = min(r0 + 1, self.rows - 1), min(c0 + 1, self.cols - 1)
        dr = row - r0
        dc = col - c0
        v00 = self._get_val(r0, c0)
        v01 = self._get_val(r0, c1)
        v10 = self._get_val(r1, c0)
        v11 = self._get_val(r1, c1)
        return float((1 - dr) * (1 - dc) * v00 + (1 - dr) * dc * v01
                     + dr * (1 - dc) * v10 + dr * dc * v11)

    def get_elevations(self, lats: Any, lons: Any) -> Any:
        """Batch elevation interpolation for arrays or lists of coordinates."""
        if np is not None and isinstance(lats, np.ndarray):
            lats_clamped = np.clip(lats, self.lat_min, self.lat_max)
            lons_clamped = np.clip(lons, self.lon_min, self.lon_max)

            lat_range = self.lat_max - self.lat_min
            lon_range = self.lon_max - self.lon_min

            row_frac = (lats_clamped - self.lat_min) / lat_range if lat_range != 0 else np.zeros_like(lats_clamped)
            col_frac = (lons_clamped - self.lon_min) / lon_range if lon_range != 0 else np.zeros_like(lons_clamped)

            row = row_frac * (self.rows - 1)
            col = col_frac * (self.cols - 1)

            r0 = np.floor(row).astype(np.int32)
            c0 = np.floor(col).astype(np.int32)
            r1 = np.minimum(r0 + 1, self.rows - 1)
            c1 = np.minimum(c0 + 1, self.cols - 1)

            dr = row - r0
            dc = col - c0

            # Fix 4a: grid is pre-cast to float32 at construction — no conversion needed here
            grid = self.grid

            v00 = grid[r0, c0]
            v01 = grid[r0, c1]
            v10 = grid[r1, c0]
            v11 = grid[r1, c1]

            return (1.0 - dr) * (1.0 - dc) * v00 + (1.0 - dr) * dc * v01 + dr * (1.0 - dc) * v10 + dr * dc * v11
        else:
            return [self.get_elevation(float(la), float(lo)) for la, lo in zip(lats, lons)]



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
        self.spatial_index: dict[tuple[int, int], DTEDTile] = {}
        
        # Backwards compatibility attributes
        self.rows = 0
        self.cols = 0
        self.lat_min = 0.0
        self.lat_max = 0.0
        self.lon_min = 0.0
        self.lon_max = 0.0
        self.grid = None

        if self.enabled:
            logger.info("[*] TerrainEngine enabled with path: %s", self.dted_path)
            self._load_dted()
        else:
            logger.info("[*] TerrainEngine disabled (no DTED path provided)")

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
            logger.warning("[!] DTED path not found: %s", self.dted_path)
            return

        candidates = self._find_dted_files(self.dted_path)
        if not candidates:
            logger.warning("[!] No candidate DTED files found in: %s", self.dted_path)
            return

        for path in candidates:
            tile = self._load_single_file(path)
            if tile is not None:
                self.tiles.append(tile)

        if not self.tiles:
            logger.warning("[!] No valid DTED files could be loaded from: %s", self.dted_path)
            return

        # Setup primary reference attributes for backwards compatibility
        self.grid = self.tiles[0].grid
        self.rows = self.tiles[0].rows
        self.cols = self.tiles[0].cols
        self.lat_min = min(t.lat_min for t in self.tiles)
        self.lat_max = max(t.lat_max for t in self.tiles)
        self.lon_min = min(t.lon_min for t in self.tiles)
        self.lon_max = max(t.lon_max for t in self.tiles)

        # Build O(1) spatial index by integer 1x1 degree cells
        self.spatial_index.clear()
        for tile in self.tiles:
            lat_start = int(math.floor(tile.lat_min))
            lat_end = int(math.ceil(tile.lat_max))
            lon_start = int(math.floor(tile.lon_min))
            lon_end = int(math.ceil(tile.lon_max))
            for la in range(lat_start, max(lat_start + 1, lat_end)):
                for lo in range(lon_start, max(lon_start + 1, lon_end)):
                    self.spatial_index[(la, lo)] = tile

        logger.info("[*] Loaded %d DTED tile(s) from %s (Lat: [%.2f, %.2f], Lon: [%.2f, %.2f])",
                    len(self.tiles), self.dted_path, self.lat_min, self.lat_max, self.lon_min, self.lon_max)

    def get_elevation(self, lat: float, lon: float) -> float:
        if not self.enabled or not self.tiles:
            return 0.0

        # Fast O(1) spatial index lookup
        cell_key = (int(math.floor(lat)), int(math.floor(lon)))
        tile = self.spatial_index.get(cell_key)
        if tile is not None and tile.contains(lat, lon):
            return tile.get_elevation(lat, lon)

        # Secondary search across tiles if coordinates are near boundary
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

    def get_elevations(self, lats: Any, lons: Any) -> Any:
        """Batch elevation interpolation across loaded DTED tiles with O(1) spatial cell routing."""
        if not self.enabled or not self.tiles:
            if np is not None and isinstance(lats, np.ndarray):
                return np.zeros_like(lats, dtype=np.float32)
            return [0.0] * len(lats)

        if len(self.tiles) == 1:
            return self.tiles[0].get_elevations(lats, lons)

        if np is not None and isinstance(lats, np.ndarray):
            result = np.zeros_like(lats, dtype=np.float32)
            assigned = np.zeros_like(lats, dtype=bool)

            # Determine unique integer 1x1 deg cells present in the input coordinates
            cell_lat = np.floor(lats).astype(np.int32)
            cell_lon = np.floor(lons).astype(np.int32)
            cell_ids = (cell_lat.astype(np.int64) << 32) | (cell_lon.astype(np.int64) & 0xFFFFFFFF)
            unique_ids = np.unique(cell_ids)

            for u_id in unique_ids:
                la = int(u_id >> 32)
                lo = int(np.int32(u_id & 0xFFFFFFFF))
                tile = self.spatial_index.get((la, lo))
                if tile is not None:
                    mask = (cell_lat == la) & (cell_lon == lo)
                    if np.any(mask):
                        result[mask] = tile.get_elevations(lats[mask], lons[mask])
                        assigned[mask] = True

            unassigned = ~assigned
            if np.any(unassigned):
                for tile in self.tiles:
                    mask = unassigned & (lats >= tile.lat_min) & (lats <= tile.lat_max) & (lons >= tile.lon_min) & (lons <= tile.lon_max)
                    if np.any(mask):
                        result[mask] = tile.get_elevations(lats[mask], lons[mask])
                        assigned[mask] = True
                        unassigned = ~assigned
                        if not np.any(unassigned):
                            break

            if np.any(~assigned):
                unassigned = ~assigned
                result[unassigned] = self.tiles[0].get_elevations(lats[unassigned], lons[unassigned])
            return result
        else:
            return [self.get_elevation(float(la), float(lo)) for la, lo in zip(lats, lons)]


