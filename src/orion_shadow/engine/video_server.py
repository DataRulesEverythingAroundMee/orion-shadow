import asyncio
import os
import shutil
import socket
import subprocess
import time
from typing import Optional, List, Tuple, Any

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
    imagery (stitched satellite tiles and/or synthetic HUD) over UDP multicast.
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
        fps: int = 10,
        width: int = 640,
        height: int = 480
    ):
        self.state = state
        self.tile_url_template = tile_url_template
        self.visualizer = TileVisualizer(tile_url_template) if tile_url_template else None
        self.multicast_group = multicast_group
        self.port = port
        self.host = host
        self.fps = fps
        self.width = width
        self.height = height

        self.session = None
        self.current_frame = None
        self.proc: Optional[subprocess.Popen] = None
        self.sock: Optional[socket.socket] = None
        self.has_ffmpeg = shutil.which("ffmpeg") is not None

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

    async def _fetch_tile(self, z: int, x: int, y: int) -> Optional[Any]:
        """Fetches a single tile from the XYZ tile server."""
        if self.session is None or self.visualizer is None or cv2 is None or np is None:
            return None
        url = self.visualizer.tile_url_template.format(z=z, x=x, y=y)
        try:
            async with self.session.get(url, timeout=aiohttp.ClientTimeout(total=2)) as response:
                if response.status == 200:
                    content = await response.read()
                    nparr = np.frombuffer(content, np.uint8)
                    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        except Exception:
            pass
        return None

    def _generate_synthetic_background(self) -> Any:
        """Generates synthetic horizon and terrain when tile server is not used."""
        if np is None:
            return bytes(self.width * self.height * 3)

        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Ground (dark green/brown) and Sky (slate blue)
        roll = getattr(self.state, 'aircraft_roll', 0.0)
        pitch = getattr(self.state, 'aircraft_pitch', 0.0)
        
        horizon_y = int(self.height / 2 + pitch * 2)
        horizon_y = max(0, min(self.height, horizon_y))
        
        # Sky
        frame[:horizon_y, :] = [80, 50, 30]
        # Ground
        frame[horizon_y:, :] = [25, 45, 20]

        if cv2 is not None:
            # Horizon line
            cv2.line(frame, (0, horizon_y), (self.width, horizon_y), (100, 140, 90), 2)
            # Pitch ladder / angle ticks
            for p in range(-30, 31, 10):
                if p == 0:
                    continue
                y = int(self.height / 2 + (pitch - p) * 4)
                if 20 < y < self.height - 20:
                    cv2.line(frame, (self.width // 2 - 40, y), (self.width // 2 + 40, y), (60, 100, 60), 1)

        return frame

    def _draw_hud(self, frame: Any) -> Any:
        """Draws gimbal OSD / HUD overlay on the frame."""
        if cv2 is None or np is None or not isinstance(frame, np.ndarray):
            return frame

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        color = (0, 255, 0)  # Gimbal green
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.4
        thick = 1

        # Center reticle
        cv2.line(frame, (cx - 25, cy), (cx - 7, cy), color, 1)
        cv2.line(frame, (cx + 7, cy), (cx + 25, cy), color, 1)
        cv2.line(frame, (cx, cy - 25), (cx, cy - 7), color, 1)
        cv2.line(frame, (cx, cy + 7), (cx, cy + 25), color, 1)
        cv2.circle(frame, (cx, cy), 3, color, 1)

        # Top Header
        cv2.putText(frame, "ORION SHADOW - SIMULATOR", (15, 25), font, scale, color, thick)
        status = "ACTIVE" if self.state.initialized else "STANDBY"
        status_color = (0, 255, 0) if self.state.initialized else (0, 165, 255)
        cv2.putText(frame, f"MODE: {status}", (w - 140, 25), font, scale, status_color, thick)

        # Telemetry footer
        lat = getattr(self.state, 'gps_lat', 0.0)
        lon = getattr(self.state, 'gps_lon', 0.0)
        alt = getattr(self.state, 'gps_alt', 0.0)
        pan = getattr(self.state, 'target_pan', 0.0)
        tilt = getattr(self.state, 'target_tilt', 0.0)

        nav_str = f"LAT: {lat:+.5f}  LON: {lon:+.5f}  ALT: {alt:.1f}m"
        gimbal_str = f"PAN: {pan:+.1f} deg   TILT: {tilt:+.1f} deg"
        cv2.putText(frame, nav_str, (15, h - 35), font, scale, color, thick)
        cv2.putText(frame, gimbal_str, (15, h - 15), font, scale, color, thick)

        # Laser status
        if getattr(self.state, 'laser_power', 0.0) > 0:
            cv2.putText(frame, "LASER ARMED", (w - 140, 45), font, scale, (0, 0, 255), thick)

        return frame

    async def _update_frame(self):
        """Stitches tiles into a frame or renders synthetic gimbal view."""
        if not self.state.initialized:
            # Standby test frame
            base_frame = self._generate_synthetic_background()
            self.current_frame = self._draw_hud(base_frame)
            return

        # If visualizer is enabled, attempt tile stitching
        if self.visualizer is not None and np is not None and cv2 is not None:
            zoom = 15
            tiles_to_fetch = self.visualizer.get_visible_tiles(
                self.state.gps_lat,
                self.state.gps_lon,
                self.state.target_pan,
                self.state.target_tilt,
                zoom
            )

            results = []
            if tiles_to_fetch and self.session is not None:
                tasks = [self._fetch_tile(z, x, y) for z, x, y in tiles_to_fetch]
                results = await asyncio.gather(*tasks)

            has_valid_tiles = any(img is not None for img in results)
            if has_valid_tiles:
                frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                tile_size = 160
                start_x = (self.width - (3 * tile_size)) // 2
                start_y = (self.height - (3 * tile_size)) // 2

                idx = 0
                for dy in range(3):
                    for dx in range(3):
                        if idx < len(results):
                            img = results[idx]
                            if img is not None:
                                resized = cv2.resize(img, (tile_size, tile_size))
                                frame[start_y + dy * tile_size: start_y + (dy + 1) * tile_size,
                                      start_x + dx * tile_size: start_x + (dx + 1) * tile_size] = resized
                        idx += 1
                self.current_frame = self._draw_hud(frame)
                return

        # Default synthetic background + HUD
        base_frame = self._generate_synthetic_background()
        self.current_frame = self._draw_hud(base_frame)

    def _start_ffmpeg(self) -> Optional[subprocess.Popen]:
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
            return subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
        except Exception as e:
            print(f"[!] Warning: Failed to spawn ffmpeg: {e}")
            return None

    async def start(self):
        """Starts the multicast video streaming loop."""
        if self.visualizer and aiohttp is not None:
            self.session = aiohttp.ClientSession()

        print(f"[+] Multicast Video Stream broadcasting on udp://@{self.multicast_group}:{self.port}")
        print(f"[*] Connect via VLC/ffplay:  vlc udp://@{self.multicast_group}:{self.port}")

        self.proc = self._start_ffmpeg()

        # Fallback socket if ffmpeg is unavailable
        if self.proc is None:
            print("[*] Using direct UDP multicast socket fallback")
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

        frame_interval = 1.0 / self.fps

        try:
            while True:
                start_time = asyncio.get_event_loop().time()
                await self._update_frame()

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
                            self.proc.stdin.flush()
                        except (BrokenPipeError, OSError):
                            print("[!] ffmpeg pipe disconnected, attempting restart...")
                            self.proc = self._start_ffmpeg()
                    elif self.sock and cv2 is not None and isinstance(self.current_frame, np.ndarray):
                        # Send compressed JPEG over UDP
                        _, enc = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                        data = enc.tobytes()
                        if len(data) < 65000:
                            try:
                                self.sock.sendto(data, (self.multicast_group, self.port))
                            except Exception:
                                pass

                elapsed = asyncio.get_event_loop().time() - start_time
                await asyncio.sleep(max(0.005, frame_interval - elapsed))

        finally:
            if self.proc:
                try:
                    if self.proc.stdin:
                        self.proc.stdin.close()
                    self.proc.terminate()
                    self.proc.wait(timeout=1.0)
                except Exception:
                    pass
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            if self.session and not self.session.closed:
                await self.session.close()
