import math
try:
    import numpy as np
except ImportError:
    np = None
from typing import Tuple, List, Optional, Dict, Any


class TileVisualizer:
    """
    Handles the logic of mapping GimbalState (lat, lon, pan, tilt, zoom) 
    to XYZ tiles from an OSM/XYZ tile server.
    Supports perspective-correct ground footprint calculation for oblique views.
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

    def latlon_to_tile_frac(self, lat: float, lon: float, z: int) -> Tuple[float, float]:
        """Converts latitude/longitude to fractional tile X/Y for a given zoom level."""
        lat_rad = math.radians(lat)
        n = 2.0 ** z
        x = (lon + 180.0) / 360.0 * n
        y = (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n
        return x, y

    def _corner_rays_ned(self, cam_hdg: float, cam_pitch: float,
                         hfov: float, vfov: float
                         ) -> List[Tuple[float, float, float]]:
        """
        Compute the 4 FOV corner ray directions in NED (North-East-Down) frame.

        Uses proper 3D rotation from camera frame to NED, accounting for the
        perspective projection geometry of the camera sensor.

        Camera frame convention: X=right, Y=down, Z=forward (boresight).
        NED frame: X=North, Y=East, Z=Down.

        Returns list of (N, E, D) unit vectors for [TL, TR, BL, BR] image corners.
        """
        # Half-extents on the normalized image plane (focal length = 1)
        hw = math.tan(math.radians(hfov / 2.0))
        hh = math.tan(math.radians(vfov / 2.0))

        # Corner directions in camera frame (X=right, Y=down, Z=forward)
        corners_cam = [
            (-hw, -hh, 1.0),  # Top-left:     left,  up
            ( hw, -hh, 1.0),  # Top-right:    right, up
            (-hw,  hh, 1.0),  # Bottom-left:  left,  down
            ( hw,  hh, 1.0),  # Bottom-right: right, down
        ]

        hdg_rad = math.radians(cam_hdg)
        pitch_rad = math.radians(cam_pitch)
        cos_h, sin_h = math.cos(hdg_rad), math.sin(hdg_rad)
        cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)

        # Camera basis vectors expressed in NED
        # Forward (boresight) in NED: azimuth=cam_hdg, elevation=cam_pitch
        fwd = (cos_p * cos_h, cos_p * sin_h, -sin_p)
        # Right: horizontal perpendicular (heading + 90°), assumes zero roll
        right = (-sin_h, cos_h, 0.0)
        # Down (camera Y-axis): fwd × right
        down = (sin_p * cos_h, sin_p * sin_h, cos_p)

        rays = []
        for cx, cy, cz in corners_cam:
            # Transform camera-frame direction to NED
            n = cx * right[0] + cy * down[0] + cz * fwd[0]
            e = cx * right[1] + cy * down[1] + cz * fwd[1]
            d = cx * right[2] + cy * down[2] + cz * fwd[2]
            mag = math.sqrt(n * n + e * e + d * d)
            if mag > 1e-12:
                rays.append((n / mag, e / mag, d / mag))
            else:
                rays.append((0.0, 0.0, 1.0))
        return rays

    def compute_distance_zoom(self, lat: float, lon: float, alt: float,
                              tile_lat: float, tile_lon: float,
                              base_zoom: int,
                              ref_dist: float,
                              zoom_min: Optional[int] = None) -> int:
        """
        Calculates distance-dependent zoom level (LOD) for a ground point/tile.
        Distant tiles get lower zoom levels, reducing tile count and matching screen GSD.
        """
        min_z = max(self.zoom_min if zoom_min is None else zoom_min, base_zoom - 2)
        if ref_dist <= 0:
            return base_zoom
        cos_lat = math.cos(math.radians(lat))
        dn = (tile_lat - lat) * 111320.0
        de = (tile_lon - lon) * 111320.0 * cos_lat
        slant_dist = math.sqrt(dn * dn + de * de + alt * alt)

        ratio = slant_dist / max(1.0, ref_dist)
        if ratio <= 1.0:
            return base_zoom
        drop = min(2, int(math.floor(math.log2(ratio))))
        return max(min_z, base_zoom - drop)

    @staticmethod
    def _point_to_segment_dist_sq(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 == 0:
            return (px - x1) ** 2 + (py - y1) ** 2
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return (px - proj_x) ** 2 + (py - proj_y) ** 2

    @staticmethod
    def _point_in_quad(px: float, py: float, quad: List[Tuple[float, float]]) -> bool:
        n = len(quad)
        inside = False
        p1x, p1y = quad[0]
        for i in range(n + 1):
            p2x, p2y = quad[i % n]
            if py > min(p1y, p2y):
                if py <= max(p1y, p2y):
                    if px <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (py - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or px <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def _cell_overlaps_quad(self, tx: int, ty: int, quad: List[Tuple[float, float]], margin: float = 1.2) -> bool:
        cx = tx + 0.5
        cy = ty + 0.5
        if self._point_in_quad(cx, cy, quad):
            return True
        m2 = margin * margin
        n = len(quad)
        for i in range(n):
            x1, y1 = quad[i]
            x2, y2 = quad[(i + 1) % n]
            if self._point_to_segment_dist_sq(cx, cy, x1, y1, x2, y2) <= m2:
                return True
        return False

    def compute_footprint(self, lat: float, lon: float, alt: float,
                          cam_hdg: float, cam_pitch: float,
                          hfov: float, vfov: float, zoom: int,
                          max_ground_range: float = 20000.0,
                          max_tiles: int = 400,
                          distance_lod: bool = True
                          ) -> Optional[Dict[str, Any]]:
        """
        Compute the camera's perspective ground footprint as a quadrilateral.

        Casts rays from the camera through each of the four image corners to a
        flat ground plane, then determines which map tiles fall within the
        resulting trapezoid.

        At oblique angles (e.g. -45° tilt) the far edge of the image sees more
        ground than the near edge, producing the characteristic trapezoidal
        footprint that distinguishes a perspective view from a nadir view.

        When distance_lod is True, tiles farther away from the camera are fetched
        at lower zoom levels (e.g. Z-1, Z-2), drastically reducing network/cache
        footprint while matching the perspective GSD of the display.

        Args:
            lat, lon: Aircraft position in degrees
            alt: Altitude above ground in meters (must be > 0)
            cam_hdg: Camera heading in degrees (0=North, 90=East)
            cam_pitch: Camera pitch in degrees (negative = looking down)
            hfov, vfov: Horizontal/vertical FOV in degrees
            zoom: Base tile zoom level (used for near ground and canvas coordinates)
            max_ground_range: Cap for near-horizontal rays (meters)
            max_tiles: Maximum tiles to return (bounding box is shrunk to fit)
            distance_lod: Whether to step down tile zoom level with distance (LOD)

        Returns:
            Dict with:
              corners_latlon      – [(lat,lon)] for TL, TR, BL, BR image corners
              corners_tile_frac   – [(tx,ty)]   fractional tile coordinates at base zoom
              tile_bounds         – (min_tx, min_ty, max_tx, max_ty) at base zoom
              tiles               – [(z, x, y)] list of tiles to fetch
            or None if alt <= 0 or zoom is out of range.
        """
        if alt <= 0:
            return None
        if not (self.zoom_min <= zoom <= self.zoom_max):
            return None

        rays = self._corner_rays_ned(cam_hdg, cam_pitch, hfov, vfov)

        limit_range = max(65000.0, float(max_ground_range or 65000.0))

        # Check near-ground visibility from bottom rays
        # If both bottom rays point into the sky or horizontal, no ground is visible
        if rays[2][2] <= 1e-5 and rays[3][2] <= 1e-5:
            return None

        # Nearest ground distance along bottom rays
        min_dep = max(0.1, max(rays[2][2], rays[3][2]))
        d_near = alt / min_dep
        if d_near > limit_range:
            # Nearest visible ground is beyond realistic visual range
            return None

        cos_lat = math.cos(math.radians(lat))
        if abs(cos_lat) < 1e-10:
            cos_lat = 1e-10

        # Physical 3D ground plane ray intersections
        # Camera is at height alt above the ground plane in NED frame.
        # Ray direction (n, e, d): if d > 1e-4, ray intersects flat earth at t = alt / d.
        # If d <= 1e-4 or ground distance exceeds limit_range, bound at limit_range along azimuth.
        ground_pts = []
        for n, e, d in rays:
            horiz_mag = math.sqrt(n * n + e * e)
            u_n = n / horiz_mag if horiz_mag > 1e-12 else 1.0
            u_e = e / horiz_mag if horiz_mag > 1e-12 else 0.0

            if d > 1e-4:
                t = alt / d
                gn = n * t
                ge = e * t
                if math.hypot(gn, ge) > limit_range:
                    gn = u_n * limit_range
                    ge = u_e * limit_range
            else:
                gn = u_n * limit_range
                ge = u_e * limit_range
            ground_pts.append([gn, ge])

        # Iteratively constrain far edge so the tile bounding box fits within max_tiles
        # Keeps near-ground tiles fixed under the camera while pulling far horizon closer if needed.
        # When distance_lod is active, tile count is aggregated hierarchically, so we constrain
        # max_dim to 64 to keep the 2048px canvas size bounded without prematurely truncating the horizon.
        for attempt in range(5):
            corners_latlon = []
            corners_tile_frac = []

            for gn, ge in ground_pts:
                c_lat = lat + gn / 111320.0
                c_lon = lon + ge / (111320.0 * cos_lat)
                corners_latlon.append((c_lat, c_lon))

                c_lat_clamped = max(-85.05, min(85.05, c_lat))
                tx, ty = self.latlon_to_tile_frac(c_lat_clamped, c_lon, zoom)
                corners_tile_frac.append((tx, ty))

            all_tx = [c[0] for c in corners_tile_frac]
            all_ty = [c[1] for c in corners_tile_frac]
            min_tx = int(math.floor(min(all_tx)))
            max_tx = int(math.floor(max(all_tx)))
            min_ty = int(math.floor(min(all_ty)))
            max_ty = int(math.floor(max(all_ty)))

            n_cols = max_tx - min_tx + 1
            n_rows = max_ty - min_ty + 1
            if distance_lod:
                if max(n_cols, n_rows) <= 64 or attempt == 4:
                    break
            else:
                if n_cols * n_rows <= max_tiles or attempt == 4:
                    break

            # Pull far edge (TL and TR: indices 0, 1) closer towards near edge (BL and BR: indices 2, 3)
            ground_pts[0][0] = ground_pts[2][0] + (ground_pts[0][0] - ground_pts[2][0]) * 0.7
            ground_pts[0][1] = ground_pts[2][1] + (ground_pts[0][1] - ground_pts[2][1]) * 0.7
            ground_pts[1][0] = ground_pts[3][0] + (ground_pts[1][0] - ground_pts[3][0]) * 0.7
            ground_pts[1][1] = ground_pts[3][1] + (ground_pts[1][1] - ground_pts[3][1]) * 0.7

        # Check for degenerate footprint (e.g. collinear points or zero area quad)
        # Order: 0=TL, 1=TR, 2=BL, 3=BR -> polygon vertices: 0, 1, 3, 2
        c = corners_tile_frac
        quad = [c[0], c[1], c[3], c[2]]
        area = 0.5 * abs(sum(quad[i][0] * quad[(i + 1) % 4][1] - quad[(i + 1) % 4][0] * quad[i][1] for i in range(4)))
        if area < 1e-6:
            return None

        max_tile_idx = 2 ** zoom - 1
        min_tx = max(0, min_tx)
        max_tx = min(max_tile_idx, max_tx)
        min_ty = max(0, min_ty)
        max_ty = min(max_tile_idx, max_ty)

        if not distance_lod:
            tiles = []
            for ty in range(min_ty, max_ty + 1):
                for tx in range(min_tx, max_tx + 1):
                    tiles.append((zoom, tx, ty))
        else:
            # Distance-dependent LOD:
            # Tiles closer to the aircraft use base zoom (e.g. 17).
            # Tiles farther away step down (e.g. 16, 15, 14), reducing tile count by up to 90%.
            ref_dist = max(100.0, d_near)
            min_zoom = max(self.zoom_min, zoom - 2)
            unique_tiles = set()
            inv_n = 1.0 / (2.0 ** zoom)

            for ty in range(min_ty, max_ty + 1):
                cell_y = ty + 0.5
                sinh_val = math.sinh(math.pi * (1.0 - 2.0 * cell_y * inv_n))
                cell_lat = math.degrees(math.atan(sinh_val))
                dn = (cell_lat - lat) * 111320.0

                for tx in range(min_tx, max_tx + 1):
                    if not self._cell_overlaps_quad(tx, ty, quad):
                        continue
                    cell_x = tx + 0.5
                    cell_lon = (cell_x * inv_n) * 360.0 - 180.0
                    de = (cell_lon - lon) * 111320.0 * cos_lat
                    slant_dist = math.sqrt(dn * dn + de * de + alt * alt)

                    ratio = slant_dist / ref_dist
                    drop = min(2, max(0, int(math.floor(math.log2(ratio))))) if ratio >= 1.0 else 0
                    z_lod = max(min_zoom, zoom - drop)

                    dz = zoom - z_lod
                    px = tx >> dz
                    py = ty >> dz
                    unique_tiles.add((z_lod, px, py))

            tiles = list(unique_tiles)
            if len(tiles) > max_tiles:
                # Prioritize broad coverage by keeping lower zoom tiles (which cover vast areas
                # with very few tiles), then prioritizing the closest fine tiles.
                def tile_priority(t):
                    tz, tpx, tpy = t
                    inv_tz = 1.0 / (2.0 ** tz)
                    t_lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * (tpy + 0.5) * inv_tz))))
                    t_lon = ((tpx + 0.5) * inv_tz) * 360.0 - 180.0
                    tdn = (t_lat - lat) * 111320.0
                    tde = (t_lon - lon) * 111320.0 * cos_lat
                    dist_sq = tdn * tdn + tde * tde
                    return (tz, dist_sq)
                tiles.sort(key=tile_priority)
                tiles = tiles[:max_tiles]

        return {
            'corners_latlon': corners_latlon,
            'corners_tile_frac': corners_tile_frac,
            'tile_bounds': (min_tx, min_ty, max_tx, max_ty),
            'tiles': tiles,
        }

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

    def get_prefetch_tiles(self, lat: float, lon: float, alt: float,
                           ac_hdg: float, cam_hdg: float, cam_pitch: float,
                           hfov: float, vfov: float, zoom: int,
                           lookahead_distance: float = 3000.0,
                           steps: int = 4,
                           max_tiles: int = 300,
                           distance_lod: bool = True
                           ) -> List[Tuple[int, int, int]]:
        """
        Calculates XYZ tiles in front of the aircraft along its flight path and camera view
        to prefetch into cache before they become visible.

        Samples multiple points ahead of the aircraft (up to lookahead_distance)
        and computes the future camera footprints along the aircraft's track.
        """
        if not (self.zoom_min <= zoom <= self.zoom_max):
            return []

        tiles_set = set()
        hdg_rad = math.radians(ac_hdg)
        cos_hdg = math.cos(hdg_rad)
        sin_hdg = math.sin(hdg_rad)

        cos_lat = math.cos(math.radians(lat))
        if abs(cos_lat) < 1e-10:
            cos_lat = 1e-10

        # Sample points along forward flight track
        step_dist = max(200.0, lookahead_distance / max(1, steps))
        curr_dist = step_dist
        while curr_dist <= lookahead_distance + 1.0:
            dn = curr_dist * cos_hdg
            de = curr_dist * sin_hdg
            fwd_lat = lat + dn / 111320.0
            fwd_lon = lon + de / (111320.0 * cos_lat)

            # 1. Perspective footprint at predicted future position
            if alt >= 10.0:
                fp = self.compute_footprint(
                    fwd_lat, fwd_lon, alt,
                    cam_hdg, cam_pitch,
                    hfov, vfov, zoom,
                    max_tiles=max_tiles,
                    distance_lod=distance_lod
                )
                if fp and fp['tiles']:
                    tiles_set.update(fp['tiles'])

            # 2. Direct ground track tile + neighbors
            zx, zy = self.latlon_to_tile(fwd_lat, fwd_lon, zoom)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    tiles_set.add((zoom, zx + dx, zy + dy))

            curr_dist += step_dist

        return list(tiles_set)
