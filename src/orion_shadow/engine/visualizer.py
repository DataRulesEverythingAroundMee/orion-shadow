import math
try:
    import numpy as np
except ImportError:
    np = None
from typing import Tuple, List, Optional


class TileVisualizer:
    """
    Handles the logic of mapping GimbalState (lat, lon, pan, tilt, zoom) 
    to XYZ tiles from an OSM/XYZ tile server.
    """
    def __init__(self, tile_url_template: str, zoom_min: int = 0, zoom_max: int = 20):
        self.tile_url_template = tile_url_template # e.g., "https://{z}/{x}/{y}.png"
        self.zoom_min = zoom_min
        self.zoom_max = zoom_max

    def latlon_to_tile(self, lat: float, lon: float, z: int) -> Tuple[int, int]:
        """Converts latitude/longitude to tile X/Y for a given zoom level."""
        lat_rad = math.radians(lat)
        n = 2.0 ** z
        x = int((lon + 180.0) / 360.0 * n)
        y = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return x, y

    def get_visible_tiles(self, lat: float, lon: float, pan: float, tilt: float, zoom: int) -> List[Tuple[int, int, int]]:
        """
        Calculates which Z/X/Y tiles are visible.
        This is a simplified placeholder for the complex geometric projection 
        required for a true gimbal FOV. 
        Currently returns the center tile and immediate neighbors.
        """
        if not (self.zoom_min <= zoom <= self.zoom_max):
            return []

        zx, zy = self.latlon_to_tile(lat, lon, zoom)
        
        # In a real implementation, we would use the gimbal orientation (pan/tilt)
        # and the camera FOV to calculate the exact footprint on the tile grid.
        # For this implementation, we return the center tile + 8 neighbors in row-major order.
        tiles = []
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                tiles.append((zoom, zx + dx, zy + dy))
        
        return tiles

