import asyncio
import math
import os
import shutil
import socket
import subprocess
import sys
import time
from typing import Optional, List, Tuple, Any, Dict, Set

from orion_shadow.core.state import GimbalState
from orion_shadow.engine.visualizer import TileVisualizer

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    import aiohttp
except ImportError:
    aiohttp = None


class VideoServer:
    """
    Multicast video streaming server that broadcasts synthesized gimbal camera
    imagery (stitched satellite tiles and/or dynamic synthetic HUD & terrain) over UDP multicast.
    Clients can connect without a web browser using VLC, ffplay, GStreamer, etc.:
        vlc udp://@<multicast_group>:<port>
        ffplay udp://<multicast_group>:<port>
    """

    def __init__(
        self,
        state: GimbalState,
        tile_url_template: Optional[str] = None,
        multicast_group: str = '239.255.0.1',
        port: int = 5004,
        host: str = '0.0.0.0',
        fps: int = 24,
        width: int = 1280,
        height: int = 720,
        max_tile_zoom: int = 17,
        tile_zoom: Optional[int] = None,
        prefetch_enabled: bool = True,
        prefetch_distance: float = 3000.0
    ):
        self.state = state
        self.tile_url_template = tile_url_template
        self.max_tile_zoom = max_tile_zoom
        self.fixed_tile_zoom = tile_zoom
        self.prefetch_enabled = prefetch_enabled
        self.prefetch_distance = prefetch_distance
        self.visualizer = TileVisualizer(tile_url_template, zoom_max=max(20, max_tile_zoom)) if tile_url_template else None
        self.multicast_group = multicast_group
        self.port = port
        self.host = host
        self.fps = fps
        self.width = width
        self.height = height

        self.session = None
        self.current_frame = None
        self.proc: Optional[Any] = None
        self.sock: Optional[socket.socket] = None
        self.has_ffmpeg = shutil.which("ffmpeg") is not None
        self.tile_cache: Dict[Tuple[int, int, int], Any] = {}
        self._pending_tile_fetches: Set[Tuple[int, int, int]] = set()
        self._fetch_semaphore: Optional[asyncio.Semaphore] = None
        self._prefetch_task: Optional[asyncio.Task] = None

        # Pre-allocated output frame buffer (avoids allocation every frame)
        self._frame_buf: Optional[Any] = None

        # Composite cache: avoids re-stitching and re-warping when the
        # camera hasn't moved enough to change the visible tile set
        self._cached_tile_keys: Optional[frozenset] = None
        self._cached_ground: Optional[Any] = None
        self._cached_tile_px: int = 0
        self._cached_tile_bounds: Optional[Tuple[int, int, int, int]] = None
        self._cached_warp_matrix: Optional[Any] = None
        self._cached_corners_frac: Optional[List[Tuple[float, float]]] = None
        self._cached_y_top: int = -9999
        self._cached_warped_frame: Optional[Any] = None

        # Resized-tile cache: avoids cv2.resize on every tile every frame
        self._resized_cache: Dict[Tuple[Tuple[int, int, int], int], Any] = {}

    def _detect_local_ip(self) -> str:
        """Determines best local IP for multicast interface routing."""
        if self.host and self.host not in ('0.0.0.0', ''):
            return self.host
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    async def _fetch_tile_worker(self, z: int, x: int, y: int):
        """Asynchronously fetches and decodes a tile in the background without blocking video rendering."""
        if self.session is None or self.visualizer is None or cv2 is None or np is None:
            return
        cache_key = (z, x, y)
        if cache_key in self.tile_cache:
            self._pending_tile_fetches.discard(cache_key)
            return

        if self._fetch_semaphore is None:
            self._fetch_semaphore = asyncio.Semaphore(16)

        url = self.visualizer.tile_url_template.format(z=z, x=x, y=y)
        try:
            async with self._fetch_semaphore:
                headers = {"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) OrionShadow/1.0"}
                async with self.session.get(url, headers=headers, timeout=aiohttp.ClientTimeout(total=3)) as response:
                    if response.status == 200:
                        content = await response.read()
                        nparr = np.frombuffer(content, np.uint8)
                        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        if img is not None:
                            if len(self.tile_cache) > 2048:
                                self.tile_cache.pop(next(iter(self.tile_cache)))
                            self.tile_cache[cache_key] = img
        except Exception:
            pass
        finally:
            self._pending_tile_fetches.discard(cache_key)

    async def _fetch_tile(self, z: int, x: int, y: int) -> Optional[Any]:
        """Fetches a single tile with caching (legacy / direct fetch helper)."""
        cache_key = (z, x, y)
        if cache_key in self.tile_cache:
            return self.tile_cache[cache_key]
        await self._fetch_tile_worker(z, x, y)
        return self.tile_cache.get(cache_key)

    def _get_tile_or_parent(self, z: int, x: int, y: int) -> Optional[Any]:
        """
        Retrieves tile (z, x, y) from cache. If not present, checks lower zoom level parent
        tiles in the cache and extracts the scaled sub-region so imagery is never pitch black.
        """
        if (z, x, y) in self.tile_cache:
            return self.tile_cache[(z, x, y)]

        if cv2 is None or np is None:
            return None

        # Check up to 5 zoom levels up for an available parent tile
        min_z = max(1, z - 5)
        for pz in range(z - 1, min_z - 1, -1):
            dz = z - pz
            px = x >> dz
            py = y >> dz
            if (pz, px, py) in self.tile_cache:
                parent_img = self.tile_cache[(pz, px, py)]
                if parent_img is None or not isinstance(parent_img, np.ndarray):
                    continue
                ph, pw = parent_img.shape[:2]
                sub_w = pw / float(1 << dz)
                sub_h = ph / float(1 << dz)
                sub_x = (x & ((1 << dz) - 1)) * sub_w
                sub_y = (y & ((1 << dz) - 1)) * sub_h
                x0 = int(round(sub_x))
                y0 = int(round(sub_y))
                x1 = max(x0 + 1, int(round(sub_x + sub_w)))
                y1 = max(y0 + 1, int(round(sub_y + sub_h)))
                cropped = parent_img[y0:y1, x0:x1]
                if cropped.size > 0:
                    return cv2.resize(cropped, (pw, ph), interpolation=cv2.INTER_LINEAR)
        return None

    async def _prefetch_loop(self):
        """
        Background task that periodically calculates future aircraft positions
        and camera view areas along the aircraft's flight path, pre-populating the tile cache
        so tiles are already in memory before being rendered.
        """
        while True:
            try:
                await asyncio.sleep(0.5)
                if not self.prefetch_enabled or self.visualizer is None or self.session is None or self.session.closed:
                    continue

                telem = self._get_telemetry()
                lat = telem['lat']
                lon = telem['lon']
                alt = telem['alt']
                ac_hdg = telem['ac_hdg']
                cam_hdg = telem['cam_hdg']
                cam_pitch = telem['cam_pitch']
                zoom_val = telem['zoom']

                # Skip prefetch if no valid coordinates yet
                if abs(lat) < 0.0001 and abs(lon) < 0.0001 and alt <= 0.0:
                    continue

                hfov, vfov, _, _ = self._get_fov(zoom_val)
                tile_zoom = self._compute_tile_zoom(lat, max(10.0, alt), cam_pitch, hfov) if alt >= 10.0 else (self.fixed_tile_zoom or min(17, self.max_tile_zoom))

                # Compute lookahead tiles along aircraft heading and camera orientation
                future_tiles = self.visualizer.get_prefetch_tiles(
                    lat=lat,
                    lon=lon,
                    alt=alt,
                    ac_hdg=ac_hdg,
                    cam_hdg=cam_hdg,
                    cam_pitch=cam_pitch,
                    hfov=hfov,
                    vfov=vfov,
                    zoom=tile_zoom,
                    lookahead_distance=self.prefetch_distance,
                    steps=4
                )

                # Filter for tiles not yet in cache or pending
                to_prefetch = [
                    t for t in future_tiles
                    if t not in self.tile_cache and t not in self._pending_tile_fetches
                ]

                # Queue background downloads in manageable batches
                for key in to_prefetch[:48]:
                    self._pending_tile_fetches.add(key)
                    asyncio.create_task(self._fetch_tile_worker(*key))

            except asyncio.CancelledError:
                break
            except Exception:
                pass

    def _get_telemetry(self) -> Dict[str, Any]:
        """Extracts current gimbal, aircraft, and sensor telemetry."""
        pan = getattr(self.state, 'current_pan', None)
        if pan is None:
            if hasattr(self.state, 'physics') and isinstance(self.state.physics.pan, dict) and 'pos' in self.state.physics.pan:
                pan = float(self.state.physics.pan['pos'])
            else:
                pan = float(getattr(self.state, 'target_pan', 0.0))

        tilt = getattr(self.state, 'current_tilt', None)
        if tilt is None:
            if hasattr(self.state, 'physics') and isinstance(self.state.physics.tilt, dict) and 'pos' in self.state.physics.tilt:
                tilt = float(self.state.physics.tilt['pos'])
            else:
                tilt = float(getattr(self.state, 'target_tilt', 0.0))

        ac_hdg = float(getattr(self.state, 'aircraft_heading', 0.0))
        ac_pitch = float(getattr(self.state, 'aircraft_pitch', 0.0))
        ac_roll = float(getattr(self.state, 'aircraft_roll', 0.0))
        lat = float(getattr(self.state, 'gps_lat', 0.0))
        lon = float(getattr(self.state, 'gps_lon', 0.0))
        alt = float(getattr(self.state, 'gps_alt', 0.0))
        speed = float(getattr(self.state, 'aircraft_speed', 0.0))
        zoom = float(max(1.0, getattr(self.state, 'camera_zoom', 1.0)))

        cam_pitch = ac_pitch + tilt
        cam_hdg = (ac_hdg + pan) % 360.0
        cam_roll = ac_roll

        return {
            'pan': pan,
            'tilt': tilt,
            'ac_hdg': ac_hdg,
            'ac_pitch': ac_pitch,
            'ac_roll': ac_roll,
            'cam_pitch': cam_pitch,
            'cam_hdg': cam_hdg,
            'cam_roll': cam_roll,
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'speed': speed,
            'zoom': zoom,
        }

    def _get_fov(self, zoom: float) -> Tuple[float, float, float, float]:
        """Computes HFOV, VFOV, and pixels-per-degree."""
        hfov = getattr(self.state, 'max_hfov_deg', 47.7) / max(1.0, zoom)
        vfov = hfov * (self.height / self.width)
        ppd_v = self.height / vfov
        ppd_h = self.width / hfov
        return hfov, vfov, ppd_h, ppd_v

    def _compute_tile_zoom(self, lat: float, alt: float,
                           cam_pitch: float, hfov: float) -> int:
        """
        Compute the optimal XYZ tile zoom level so tile resolution matches
        the camera's ground sample distance (GSD) at the boresight.

        Higher camera zoom (narrower FOV) or lower altitude produces a higher
        tile zoom level, yielding sharper satellite/map imagery.
        """
        if self.fixed_tile_zoom is not None:
            return self.fixed_tile_zoom

        # Depression angle at boresight (degrees below horizontal)
        depression = max(5.0, -cam_pitch)
        boresight_dist = alt / math.tan(math.radians(depression))
        # Ground width visible across the full image
        ground_width = 2.0 * boresight_dist * math.tan(math.radians(hfov / 2.0))
        # Output GSD: meters per output pixel
        output_gsd = ground_width / max(1, self.width)

        # Tile GSD at zoom z: C·cos(lat) / (2^z · 256)
        # Solve for z where tile GSD ≤ output GSD
        EARTH_CIRCUMFERENCE = 40075016.686
        cos_lat = math.cos(math.radians(lat))
        if output_gsd > 0:
            z = math.log2(EARTH_CIRCUMFERENCE * cos_lat / (256.0 * output_gsd))
            z = int(round(z))
        else:
            z = self.max_tile_zoom
        return max(14, min(self.max_tile_zoom, z))

    def _generate_synthetic_background(self) -> Any:
        """
        Generates realistic dynamic synthesized gimbal camera view with:
        - Horizon elevation shifting with gimbal tilt and aircraft pitch
        - Horizon roll tilting with aircraft bank angle
        - Distant terrain / mountain silhouette scrolling with gimbal/aircraft heading
        - Ground grid and patchwork farmland scrolling in real-time with aircraft movement
        - Sky gradient with atmospheric haze
        """
        telem = self._get_telemetry()
        zoom = telem['zoom']
        hfov, vfov, ppd_h, ppd_v = self._get_fov(zoom)

        cx = self.width // 2
        cy = self.height // 2

        cam_pitch = telem['cam_pitch']
        cam_hdg = telem['cam_hdg']
        cam_roll = telem['cam_roll']
        lat = telem['lat']
        lon = telem['lon']
        alt = max(50.0, telem['alt'])

        # Calculate horizon Y relative to camera boresight
        # Looking down (negative pitch) moves horizon UP (towards y=0 and above)
        # Looking up (positive pitch) moves horizon DOWN (towards y=height and below)
        horizon_cy = cy + cam_pitch * ppd_v

        if np is None:
            # Pure Python fallback
            hy = int(max(0, min(self.height, horizon_cy)))
            sky_row = bytes([80, 50, 30] * self.width)
            gnd_row = bytes([25, 45, 20] * self.width)
            return sky_row * hy + gnd_row * (self.height - hy)

        frame = np.empty((self.height, self.width, 3), dtype=np.uint8)

        # Base horizon Y clamped to frame boundaries for horizontal case
        hy = int(np.clip(horizon_cy, 0, self.height))

        # 1. Sky Gradient: Zenith (deep blue) to Horizon (atmospheric haze)
        if hy > 0:
            sky_vals = np.linspace(110, 185, hy, dtype=np.float32)[:, np.newaxis]
            frame[:hy, :, 0] = sky_vals.astype(np.uint8)
            frame[:hy, :, 1] = (sky_vals * 0.72).astype(np.uint8)
            frame[:hy, :, 2] = (sky_vals * 0.45).astype(np.uint8)

        # 2. Ground Base Gradient: Horizon (haze/olive) to Nadir (rich earth green)
        if hy < self.height:
            g_len = self.height - hy
            g_vals = np.linspace(72, 38, g_len, dtype=np.float32)[:, np.newaxis]
            frame[hy:, :, 0] = (g_vals * 0.55).astype(np.uint8)
            frame[hy:, :, 1] = g_vals.astype(np.uint8)
            frame[hy:, :, 2] = (g_vals * 0.45).astype(np.uint8)

        if cv2 is not None:
            # 3. Distant mountain / terrain silhouette along the horizon (scrolls with heading)
            if -80 < horizon_cy < self.height + 80:
                mtn_pts = []
                for x in range(0, self.width + 12, 10):
                    deg_offset = (x - cx) / ppd_h
                    az = (cam_hdg + deg_offset) % 360.0
                    m_elev = (
                        math.sin(math.radians(az * 3.5)) * 16.0 +
                        math.sin(math.radians(az * 7.2 + 45.0)) * 10.0 +
                        math.cos(math.radians(az * 14.1)) * 5.0 + 12.0
                    )
                    my = int(horizon_cy - m_elev)
                    mtn_pts.append((x, max(0, min(self.height - 1, my))))

                if len(mtn_pts) >= 2:
                    for i in range(len(mtn_pts) - 1):
                        cv2.line(frame, mtn_pts[i], mtn_pts[i + 1], (75, 95, 65), 2)

            # 4. Perspective Ground Grid & Terrain Motion (scrolls with aircraft flight)
            x_m = lon * 111320.0 * math.cos(math.radians(lat))
            y_m = lat * 111320.0
            hdg_rad = math.radians(cam_hdg)

            grid_scale = max(1.0, zoom)
            grid_spacing = 500.0 / (2 ** min(6, int(math.log2(grid_scale))))
            d_forward = x_m * math.sin(hdg_rad) + y_m * math.cos(hdg_rad)
            d_lateral = x_m * math.cos(hdg_rad) - y_m * math.sin(hdg_rad)

            fwd_offset = d_forward % grid_spacing
            lat_offset = d_lateral % grid_spacing

            # Draw transverse ground lines (distance ahead)
            dep_boresight = max(1.0, -cam_pitch)
            nom_dist = alt / math.tan(math.radians(dep_boresight))
            for k in range(-15, 25):
                dist_ahead = nom_dist + k * grid_spacing - fwd_offset
                if dist_ahead < 10.0:
                    continue
                dep_angle = math.degrees(math.atan2(alt, dist_ahead))
                rel_pitch = cam_pitch + dep_angle
                sy = int(cy + rel_pitch * ppd_v)
                if hy <= sy < self.height:
                    dist_frac = min(1.0, float(sy - hy) / max(1.0, float(self.height - hy)))
                    color_val = int(45 + 55 * dist_frac)
                    cv2.line(frame, (0, sy), (self.width, sy), (color_val // 2, color_val, color_val // 2), 1)

            # Draw longitudinal ground lines (converging to vanishing point on horizon)
            vanish_y = int(np.clip(horizon_cy, -25000, self.height + 25000))
            for m in range(-20, 21):
                ground_x = m * grid_spacing - lat_offset
                rel_az_deg = math.degrees(math.atan2(ground_x, max(10.0, nom_dist)))
                sx_bottom = int(cx + rel_az_deg * ppd_h)
                if -self.width < sx_bottom < self.width * 2:
                    cv2.line(frame, (cx, vanish_y), (sx_bottom, self.height), (35, 65, 30), 1)

            # 5. Horizon dividing line
            if 0 <= hy < self.height:
                cv2.line(frame, (0, hy), (self.width, hy), (120, 180, 110), 2)

            # 6. Apply roll rotation if aircraft is banking
            if abs(cam_roll) > 0.1:
                rot_mat = cv2.getRotationMatrix2D((cx, cy), -cam_roll, 1.0)
                frame = cv2.warpAffine(frame, rot_mat, (self.width, self.height), borderMode=cv2.BORDER_REFLECT)

            # 7. Moving Compass Tape at top of HUD (scrolls with camera heading)
            tape_y = 35
            tape_w = 280
            cv2.rectangle(frame, (cx - tape_w // 2, tape_y - 14), (cx + tape_w // 2, tape_y + 12), (15, 25, 15), -1)
            cv2.rectangle(frame, (cx - tape_w // 2, tape_y - 14), (cx + tape_w // 2, tape_y + 12), (0, 180, 0), 1)
            cv2.drawMarker(frame, (cx, tape_y + 16), (0, 255, 0), cv2.MARKER_TRIANGLE_UP, 6, 1)

            tape_ppd = 3.2
            cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
            for deg in range(int(cam_hdg - 50), int(cam_hdg + 51)):
                norm_deg = deg % 360
                tx = int(cx + (deg - cam_hdg) * tape_ppd)
                if cx - tape_w // 2 + 5 <= tx <= cx + tape_w // 2 - 5:
                    if norm_deg % 30 == 0:
                        cv2.line(frame, (tx, tape_y - 12), (tx, tape_y - 4), (0, 255, 0), 1)
                        lbl = cardinals.get(norm_deg, f"{norm_deg:03d}")
                        cv2.putText(frame, lbl, (tx - 6, tape_y + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 0), 1)
                    elif norm_deg % 10 == 0:
                        cv2.line(frame, (tx, tape_y - 9), (tx, tape_y - 4), (0, 200, 0), 1)
                    elif norm_deg % 5 == 0:
                        cv2.line(frame, (tx, tape_y - 7), (tx, tape_y - 4), (0, 140, 0), 1)

            # 8. Pitch Ladder Bars
            for p in range(-80, 31, 10):
                if p == 0:
                    continue
                sy = int(cy + (cam_pitch - p) * ppd_v)
                if 40 < sy < self.height - 40:
                    bar_w = 40 if abs(p) % 20 == 0 else 24
                    cv2.line(frame, (cx - bar_w, sy), (cx - 10, sy), (60, 150, 60), 1)
                    cv2.line(frame, (cx + 10, sy), (cx + bar_w, sy), (60, 150, 60), 1)
                    tick_dir = 4 if p < 0 else -4
                    cv2.line(frame, (cx - bar_w, sy), (cx - bar_w, sy + tick_dir), (60, 150, 60), 1)
                    cv2.line(frame, (cx + bar_w, sy), (cx + bar_w, sy + tick_dir), (60, 150, 60), 1)
                    cv2.putText(frame, f"{abs(p)}", (cx + bar_w + 3, sy + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (60, 150, 60), 1)

        return frame

    def _draw_hud(self, frame: Any) -> Any:
        """Draws gimbal OSD / HUD overlay on the frame."""
        if cv2 is None or np is None or not isinstance(frame, np.ndarray):
            return frame

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        color = (0, 255, 0)  # Gimbal green
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.38
        thick = 1

        telem = self._get_telemetry()
        lat = telem['lat']
        lon = telem['lon']
        alt = telem['alt']
        speed = telem.get('speed', 0.0)
        pan = telem['pan']
        tilt = telem['tilt']
        cam_hdg = telem['cam_hdg']
        ac_hdg = telem['ac_hdg']
        zoom = telem['zoom']

        # Center reticle
        cv2.line(frame, (cx - 22, cy), (cx - 7, cy), color, 1)
        cv2.line(frame, (cx + 7, cy), (cx + 22, cy), color, 1)
        cv2.line(frame, (cx, cy - 22), (cx, cy - 7), color, 1)
        cv2.line(frame, (cx, cy + 7), (cx, cy + 22), color, 1)
        cv2.circle(frame, (cx, cy), 3, color, 1)

        # Left-margin Airspeed Indicator Box
        spd_val = int(round(speed))
        spd_text = f"{spd_val:3d} KTS"
        bx1, bx2 = 18, 92
        by1, by2 = cy - 11, cy + 11
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (15, 25, 15), -1)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 1)
        cv2.putText(frame, spd_text, (bx1 + 5, cy + 4), font, scale, color, thick)
        cv2.line(frame, (bx2, cy), (bx2 + 8, cy), color, 1)

        # Top Header
        is_attached = getattr(self.state, 'gps_received', False)
        title = "ORION SHADOW - FLIGHT ATTACHED (ADS-B)" if is_attached else "ORION SHADOW - SIMULATOR"
        cv2.putText(frame, title, (15, 20), font, scale, color, thick)
        status = "ACTIVE" if self.state.initialized else "STANDBY"
        status_color = (0, 255, 0) if self.state.initialized else (0, 165, 255)
        cv2.putText(frame, f"MODE: {status}", (w - 130, 20), font, scale, status_color, thick)

        # Telemetry footer
        alt_ft = alt * 3.28084
        nav_str = f"LAT: {lat:+.5f}  LON: {lon:+.5f}  ALT: {alt:.1f}m ({alt_ft:.0f}ft)"
        orient_str = f"A/C TRK: {ac_hdg:03.0f} deg   CAM HDG: {cam_hdg:03.0f} deg   SPD: {speed:03.0f} kts"
        gimbal_str = f"PAN: {pan:+.1f} deg   TILT: {tilt:+.1f} deg   ZOOM: {zoom:.1f}x"
        cv2.putText(frame, nav_str, (15, h - 45), font, scale, color, thick)
        cv2.putText(frame, orient_str, (15, h - 28), font, scale, color, thick)
        cv2.putText(frame, gimbal_str, (15, h - 11), font, scale, color, thick)

        # Laser status
        if getattr(self.state, 'laser_power', 0.0) > 0:
            cv2.putText(frame, "LASER ARMED", (w - 130, 40), font, scale, (0, 0, 255), thick)

        return frame

    async def _update_frame(self):
        """Stitches tiles into a frame or renders synthetic gimbal view."""
        if not self.state.initialized:
            # Standby test frame
            base_frame = self._generate_synthetic_background()
            self.current_frame = self._draw_hud(base_frame)
            return

        # If visualizer is enabled, attempt perspective-correct tile rendering
        if self.visualizer is not None and np is not None and cv2 is not None:
            telem = self._get_telemetry()
            hfov, vfov, ppd_h, ppd_v = self._get_fov(telem['zoom'])
            alt = telem['alt']

            # Adaptive zoom: pick tile zoom level matching the camera's GSD
            zoom = self._compute_tile_zoom(
                telem['lat'], max(10.0, alt), telem['cam_pitch'], hfov
            ) if alt >= 10.0 else (self.fixed_tile_zoom or min(17, self.max_tile_zoom))

            # Use perspective footprint when altitude is sufficient
            footprint = None
            if alt >= 10.0:
                footprint = self.visualizer.compute_footprint(
                    telem['lat'], telem['lon'], alt,
                    telem['cam_hdg'], telem['cam_pitch'],
                    hfov, vfov, zoom, max_tiles=600
                )

            tiles_to_fetch = []
            if footprint and footprint['tiles']:
                tiles_to_fetch = footprint['tiles']
            else:
                # Fallback: center + neighbors when footprint unavailable
                tiles_to_fetch = self.visualizer.get_visible_tiles(
                    telem['lat'], telem['lon'],
                    telem['pan'], telem['tilt'], zoom
                )

            # Retrieve cached tiles (with parent fallback) immediately without blocking rendering
            tile_results = {}
            missing_tiles = []
            for t in tiles_to_fetch:
                img = self._get_tile_or_parent(*t)
                if img is not None:
                    tile_results[t] = img
                if t not in self.tile_cache and t not in self._pending_tile_fetches:
                    missing_tiles.append(t)

            # Trigger background asynchronous fetch for missing tiles
            if missing_tiles and self.session is not None and not self.session.closed:
                for key in missing_tiles[:32]:
                    self._pending_tile_fetches.add(key)
                    asyncio.create_task(self._fetch_tile_worker(*key))

            has_valid_tiles = any(img is not None for img in tile_results.values())
            if has_valid_tiles and footprint:
                # --- Perspective-correct tile compositing (cached & bounded) ---
                min_tx, min_ty, max_tx, max_ty = footprint['tile_bounds']
                n_cols = max_tx - min_tx + 1
                n_rows = max_ty - min_ty + 1

                # Dynamically size tile_px so the ground canvas stays under 2048px while preserving high resolution.
                max_dim = max(n_cols, n_rows)
                target_canvas_dim = 2048
                tile_px = max(32, min(256, int(target_canvas_dim / max(1, max_dim))))

                # Check if tile set changed since last frame
                current_tile_keys = frozenset(
                    k for k, v in tile_results.items() if v is not None
                )
                corners_frac = footprint['corners_tile_frac']
                tile_bounds = footprint['tile_bounds']

                tiles_changed = (current_tile_keys != self._cached_tile_keys
                                 or tile_bounds != self._cached_tile_bounds
                                 or tile_px != self._cached_tile_px)

                # Rebuild ground image only when tile set changes
                if tiles_changed or self._cached_ground is None:
                    ground = np.zeros((n_rows * tile_px, n_cols * tile_px, 3),
                                      dtype=np.uint8)
                    ground[:, :] = (38, 72, 45)  # Natural earth-green base to avoid black voids
                    for (z, tx, ty), img in tile_results.items():
                        if img is not None:
                            col = tx - min_tx
                            row = ty - min_ty
                            if 0 <= col < n_cols and 0 <= row < n_rows:
                                # Cache resized tiles to avoid redundant resize
                                rk = ((z, tx, ty), tile_px)
                                if rk not in self._resized_cache:
                                    if tile_px == 256 and img.shape[0] == 256 and img.shape[1] == 256:
                                        self._resized_cache[rk] = img
                                    else:
                                        self._resized_cache[rk] = cv2.resize(
                                            img, (tile_px, tile_px),
                                            interpolation=cv2.INTER_LINEAR)
                                    # Evict old resized entries
                                    if len(self._resized_cache) > 2048:
                                        oldest = next(iter(self._resized_cache))
                                        del self._resized_cache[oldest]
                                ground[row * tile_px:(row + 1) * tile_px,
                                       col * tile_px:(col + 1) * tile_px] = \
                                    self._resized_cache[rk]

                    self._cached_ground = ground
                    self._cached_tile_keys = current_tile_keys
                    self._cached_tile_px = tile_px
                    self._cached_tile_bounds = tile_bounds

                horizon_y = int(self.height / 2.0 + telem['cam_pitch'] * ppd_v)
                # sky_top is the pixel row where sky ends and ground begins (for sky compositing only)
                sky_top = max(0, min(self.height - 2, horizon_y)) if horizon_y > 0 else 0

                corners_changed = (corners_frac != self._cached_corners_frac or sky_top != self._cached_y_top)

                # Recompute perspective matrix only when footprint corners change
                if tiles_changed or corners_changed or self._cached_warped_frame is None:
                    try:
                        src_pts = np.float32([
                            [(c[0] - min_tx) * tile_px, (c[1] - min_ty) * tile_px]
                            for c in corners_frac
                        ])
                        # The far footprint corners (TL, TR) represent the horizon ground —
                        # the farthest visible point along the camera's forward view.
                        # Physically, this ground appears at the horizon line in the image
                        # (row sky_top). Mapping TL/TR to row sky_top is the geometrically
                        # correct transform: the perspective warp then correctly interpolates
                        # tile content between the far (horizon) and near (bottom) ground
                        # across the rows sky_top..height-1, and the sky compositing
                        # covers rows 0..sky_top-1.
                        #
                        # Mapping far corners to row 0 (previous approach) was wrong: the
                        # sky overlay would hide those tiles, and the slice of the warp
                        # visible below sky_top started from an intermediate distance —
                        # much closer than the actual horizon — creating the false altitude-
                        # gain illusion exactly when sky first became visible.
                        dst_pts = np.float32([
                            [0,              sky_top],
                            [self.width - 1, sky_top],
                            [0,              self.height - 1],
                            [self.width - 1, self.height - 1],
                        ])
                        self._cached_warp_matrix = cv2.getPerspectiveTransform(
                            src_pts, dst_pts)
                        self._cached_corners_frac = corners_frac
                        self._cached_y_top = sky_top

                        # Warp into pre-allocated buffer
                        if (self._frame_buf is None
                                or self._frame_buf.shape != (self.height, self.width, 3)):
                            self._frame_buf = np.empty(
                                (self.height, self.width, 3), dtype=np.uint8)

                        cv2.warpPerspective(self._cached_ground,
                                            self._cached_warp_matrix,
                                            (self.width, self.height),
                                            dst=self._frame_buf,
                                            borderMode=cv2.BORDER_CONSTANT,
                                            borderValue=(38, 72, 45))

                        if sky_top > 0:
                            # Composite synthetic sky above the horizon into the warped frame
                            base_frame = self._generate_synthetic_background()
                            base_frame[sky_top:, :] = self._frame_buf[sky_top:, :]
                            self._cached_warped_frame = base_frame
                        else:
                            self._cached_warped_frame = self._frame_buf.copy()
                    except Exception:
                        self._cached_warped_frame = None

                # Apply HUD on a copy of the cached warp (HUD updates every frame)
                if self._cached_warped_frame is not None:
                    frame = self._cached_warped_frame.copy()
                    self.current_frame = self._draw_hud(frame)
                    return

        # Default synthetic background + HUD
        base_frame = self._generate_synthetic_background()
        self.current_frame = self._draw_hud(base_frame)

    def _init_fallback_socket(self):
        """Initializes fallback UDP socket for multicast video streaming."""
        if self.sock is not None:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        try:
            self.sock.setsockopt(
                socket.IPPROTO_IP, 
                socket.IP_MULTICAST_IF, 
                socket.inet_aton(self._detect_local_ip())
            )
        except Exception:
            pass

    async def _start_ffmpeg(self) -> Optional[Any]:
        """Launches ffmpeg to encode raw video to MPEG-TS over UDP multicast."""
        if not self.has_ffmpeg:
            return None

        local_ip = self._detect_local_ip()
        dest_url = f"udp://{self.multicast_group}:{self.port}?pkt_size=1316&ttl=2"
        if local_ip:
            dest_url += f"&localaddr={local_ip}"

        cmd = [
            "ffmpeg",
            "-y",
            "-nostats",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-f", "mpegts",
            dest_url
        ]

        try:
            return await asyncio.create_subprocess_exec(
                *cmd,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL
            )
        except Exception as e:
            print(f"[!] Warning: Failed to spawn ffmpeg: {e}")
            return None

    async def start(self):
        """Starts the multicast video streaming loop."""
        if self.visualizer and aiohttp is not None:
            headers = {"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) OrionShadow/1.0"}
            self.session = aiohttp.ClientSession(headers=headers)

        print(f"[+] Multicast Video Stream broadcasting on udp://@{self.multicast_group}:{self.port}")
        print(f"[*] Connect via ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 -framedrop -sync video udp://@{self.multicast_group}:{self.port}")

        self.proc = await self._start_ffmpeg()

        # Start predictive background prefetcher if enabled
        if self.prefetch_enabled and self.visualizer:
            self._prefetch_task = asyncio.create_task(self._prefetch_loop())

        # Fallback socket if ffmpeg is unavailable
        if self.proc is None:
            print("[*] Using direct UDP multicast socket fallback")
            self._init_fallback_socket()

        frame_interval = 1.0 / self.fps
        next_frame_time = time.perf_counter()

        try:
            while True:
                try:
                    await self._update_frame()
                except Exception as e:
                    # Log rendering error but keep stream loop alive
                    print(f"[!] Video rendering error: {e}")

                if self.current_frame is not None:
                    if hasattr(self.current_frame, 'tobytes'):
                        raw_bytes = self.current_frame.tobytes()
                    elif isinstance(self.current_frame, bytes):
                        raw_bytes = self.current_frame
                    else:
                        raw_bytes = bytes(self.current_frame)

                    if self.proc and self.proc.stdin:
                        try:
                            self.proc.stdin.write(raw_bytes)
                            await asyncio.wait_for(self.proc.stdin.drain(), timeout=1.0)
                        except (BrokenPipeError, ConnectionResetError, OSError, asyncio.TimeoutError) as e:
                            print(f"[!] ffmpeg pipe error ({e}), attempting restart...")
                            try:
                                if self.proc.stdin:
                                    self.proc.stdin.close()
                            except Exception:
                                pass
                            try:
                                self.proc.terminate()
                            except Exception:
                                pass
                            self.proc = await self._start_ffmpeg()
                            if self.proc is None and self.sock is None:
                                print("[*] Switching to direct UDP multicast socket fallback")
                                self._init_fallback_socket()
                    elif self.sock and cv2 is not None and isinstance(self.current_frame, np.ndarray):
                        # Send compressed JPEG over UDP
                        _, enc = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                        data = enc.tobytes()
                        if len(data) < 65000:
                            try:
                                self.sock.sendto(data, (self.multicast_group, self.port))
                            except Exception:
                                pass

                # Precise frame timing with monotonic clock
                next_frame_time += frame_interval
                now = time.perf_counter()
                sleep_duration = next_frame_time - now
                if sleep_duration > 0:
                    await asyncio.sleep(sleep_duration)
                else:
                    if sleep_duration < -frame_interval:
                        next_frame_time = now
                    await asyncio.sleep(0.001)

        finally:
            if self._prefetch_task and not self._prefetch_task.done():
                self._prefetch_task.cancel()
            if self.proc:
                try:
                    self.proc.terminate()
                except ProcessLookupError:
                    pass
                except Exception:
                    pass
                try:
                    if self.proc.stdin:
                        self.proc.stdin.close()
                        await asyncio.wait_for(self.proc.stdin.wait_closed(), timeout=0.5)
                except Exception:
                    pass
                try:
                    await asyncio.wait_for(self.proc.wait(), timeout=0.5)
                except (asyncio.TimeoutError, Exception):
                    try:
                        self.proc.kill()
                        await asyncio.wait_for(self.proc.wait(), timeout=0.5)
                    except Exception:
                        pass
                self.proc = None
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            if self.session and not self.session.closed:
                await self.session.close()
