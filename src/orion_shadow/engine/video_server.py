import asyncio
import cv2
import numpy as np
import aiohttp
import time
from typing import Optional
from orion_shadow.engine.visualizer import TileVisualizer
from orion_shadow.core.state import GimbalState

class VideoServer:
    """
    An asynchronous MJPEG server that streams 'stitched' tiles based 
    on the current GimbalState.
    """
    def __init__(self, state: GimbalState, tile_url_template: str, host: str = '0.0.0.0', port: int = 5001):
        self.state = state
        self.visualizer = TileVisualizer(tile_url_template)
        self.host = host
        self.port = port
        self.session: Optional[aiohttp.ClientSession] = None
        self.current_frame: Optional[np.ndarray] = None

    async def _fetch_tile(self, z: int, x: int, y: int) -> Optional[np.ndarray]:
        """Fetches a single tile from the XYZ server."""
        url = self.visualizer.tile_url_template.format(z=z, x=x, y=y)
        if self.session is None:
            return None
        try:
            async with self.session.get(url, timeout=aiohttp.ClientTimeout(total=2)) as response:
                if response.status == 200:
                    content = await response.read()
                    # Decode image from bytes
                    nparr = np.frombuffer(content, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    return img
        except Exception:
            pass
        return None

    async def _update_frame(self):
        """Stitches tiles into a single frame based on current GimbalState."""
        if not self.state.initialized:
            # Return a black frame if not initialized
            self.current_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            return

        # 1. Calculate visible tiles
        # We use a fixed zoom for the 'video' feed for stability in this demo
        zoom = 15 
        tiles_to_fetch = self.visualizer.get_visible_tiles(
            self.state.gps_lat, 
            self.state.gps_lon, 
            self.state.target_pan, 
            self.state.target_tilt, 
            zoom
        )

        # 2. Fetch all tiles in parallel
        tasks = [self._fetch_tile(z, x, y) for z, x, y in tiles_to_fetch]
        results = await asyncio.gather(*tasks)

        # 3. Stitching (Simplified: 3x3 grid)
        # We expect 9 tiles for the [-1,0,1] grid
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # For simplicity, we resize all fetched tiles to 224x224 
        # and place them in a 3x3 grid in the center of the 640x480 frame.
        tile_size = 160
        start_x = (640 - (3 * tile_size)) // 2
        start_y = (480 - (3 * tile_size)) // 2
        
        idx = 0
        for dy in range(3):
            for dx in range(3):
                if idx < len(results):
                    img = results[idx]
                    if img is not None:
                        resized = cv2.resize(img, (tile_size, tile_size))
                        frame[start_y + dy*tile_size : start_y + (dy+1)*tile_size, 
                             start_x + dx*tile_size : start_x + (dx+1)*tile_size] = resized
                idx += 1
        
        self.current_frame = frame

    async def start(self):
        """Starts the MJPEG server."""
        self.session = aiohttp.ClientSession()
        
        # Background task to update the frame
        asyncio.create_task(self._frame_update_loop())

        print(f"[+] Video Stream (MJPEG) active on {self.host}:{self.port}")
        
        # Use a simple aiohttp web server for the MJPEG stream
        from aiohttp import web
        
        async def mjpeg_handler(request):
            response = web.StreamResponse(
                status=200,
                reason='OK',
                headers={
                    'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                }
            )
            await response.prepare(request)
            
            while True:
                if self.current_frame is not None:
                    # Encode frame as JPEG
                    _, jpeg = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    body = (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' +
                        jpeg.tobytes() + b'\r\n'
                    )
                    await response.write(body)
                await asyncio.sleep(0.1) # ~10 FPS

            return response

        app = web.Application()
        app.router.add_get('/', mjpeg_handler)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()

    async def _frame_update_loop(self):
        while True:
            await self._update_frame()
            await asyncio.sleep(self.state.dt)
