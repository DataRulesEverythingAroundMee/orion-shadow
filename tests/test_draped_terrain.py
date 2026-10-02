import os
import math
import unittest

from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.terrain_renderer import TerrainDraper
from orion_shadow.engine.video_server import VideoServer
from orion_shadow.server import OrionServer

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None


class TestDrapedTerrain(unittest.TestCase):
    def setUp(self):
        self.flat_dted = "tests/terrain_data/flat.dt0"
        self.slope_dted = "tests/terrain_data/slope.dt1"
        self.tile_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

    def test_cli_flags_wiring(self):
        """Verify is_3d_terrain_active only when BOTH tile-url and dted-path are provided."""
        # 1. Both provided -> 3D draped terrain should be active
        server_both = OrionServer(
            dted_path=self.flat_dted,
            tile_url=self.tile_url,
            video_enabled=True,
            port=0,
            udp_in_port=0,
            tcp_port=None
        )
        self.assertIsNotNone(server_both.video_server)
        self.assertTrue(server_both.video_server.is_3d_terrain_active)
        self.assertIsNotNone(server_both.video_server.terrain_draper)

        # 2. Only tile-url provided -> flat visualization, not 3D draped
        server_tile_only = OrionServer(
            dted_path=None,
            tile_url=self.tile_url,
            video_enabled=True,
            port=0,
            udp_in_port=0,
            tcp_port=None
        )
        self.assertIsNotNone(server_tile_only.video_server)
        self.assertFalse(server_tile_only.video_server.is_3d_terrain_active)

        # 3. Only dted-path provided -> synthetic HUD, not 3D draped (no tiles)
        server_dted_only = OrionServer(
            dted_path=self.flat_dted,
            tile_url=None,
            video_enabled=True,
            port=0,
            udp_in_port=0,
            tcp_port=None
        )
        self.assertIsNotNone(server_dted_only.video_server)
        self.assertFalse(server_dted_only.video_server.is_3d_terrain_active)

        # 4. Neither provided -> synthetic HUD
        server_neither = OrionServer(
            dted_path=None,
            tile_url=None,
            video_enabled=True,
            port=0,
            udp_in_port=0,
            tcp_port=None
        )
        self.assertIsNotNone(server_neither.video_server)
        self.assertFalse(server_neither.video_server.is_3d_terrain_active)

    def test_vectorized_get_elevations_matches_scalar(self):
        """Ensure get_elevations returns identical results to scalar get_elevation."""
        engine = TerrainEngine(self.slope_dted)
        self.assertTrue(engine.enabled)

        lats = [34.1, 34.3, 34.5, 34.7, 34.9]
        lons = [-118.9, -118.7, -118.5, -118.3, -118.1]

        scalar_elevs = [engine.get_elevation(la, lo) for la, lo in zip(lats, lons)]

        if np is not None:
            lats_arr = np.array(lats, dtype=np.float32)
            lons_arr = np.array(lons, dtype=np.float32)
            batch_elevs = engine.get_elevations(lats_arr, lons_arr)
            for s, b in zip(scalar_elevs, batch_elevs):
                self.assertAlmostEqual(s, float(b), places=4)
        else:
            batch_elevs = engine.get_elevations(lats, lons)
            for s, b in zip(scalar_elevs, batch_elevs):
                self.assertAlmostEqual(s, float(b), places=4)

    def test_terrain_draper_camera_rays(self):
        """Verify camera ray generation dimensions and normalization."""
        if np is None:
            self.skipTest("numpy not installed")

        engine = TerrainEngine(self.slope_dted)
        draper = TerrainDraper(engine, sub_sample=4)

        rays = draper._get_camera_rays(width=1280, height=720, hfov=47.7, vfov=35.8)
        self.assertIsNotNone(rays)
        self.assertEqual(rays.shape, (180, 320, 3))

        # Check that all ray vectors are normalized unit vectors
        norms = np.linalg.norm(rays, axis=-1)
        self.assertTrue(np.allclose(norms, 1.0, atol=1e-5))

    def test_terrain_draper_render_output(self):
        """Verify 3D draped terrain renderer generates valid frame with tiles."""
        if np is None or cv2 is None:
            self.skipTest("cv2 or numpy not installed")

        engine = TerrainEngine(self.slope_dted)
        draper = TerrainDraper(engine, sub_sample=4)

        # Mock ground texture (e.g. 512x512 green pattern)
        ground = np.full((512, 512, 3), 60, dtype=np.uint8)
        ground[:, :, 1] = 140  # greenish

        telem = {
            'lat': 34.2,
            'lon': -118.5,
            'alt': 1200.0,
            'cam_pitch': -25.0,
            'cam_hdg': 0.0,
            'cam_roll': 0.0,
            'zoom': 1.0
        }
        tile_bounds = (10, 20, 13, 23)
        tile_px = 128
        zoom = 15

        base_sky = np.full((720, 1280, 3), 180, dtype=np.uint8)
        out = draper.render(
            ground_texture=ground,
            tile_bounds=tile_bounds,
            tile_px=tile_px,
            zoom=zoom,
            telem=telem,
            width=1280,
            height=720,
            hfov=47.7,
            vfov=35.8,
            base_sky_frame=base_sky
        )

        self.assertIsNotNone(out)
        self.assertEqual(out.shape, (720, 1280, 3))
        # Ensure image is not empty / all black
        self.assertTrue(np.mean(out) > 10.0)

    def test_video_server_frame_generation_with_3d_terrain(self):
        """Verify VideoServer renders 3D draped frames into current_frame."""
        if np is None or cv2 is None:
            self.skipTest("cv2 or numpy not installed")

        engine = TerrainEngine(self.slope_dted)
        state = GimbalState(dt=0.1, terrain_engine=engine, lat=34.2, lon=-118.5, alt=1200.0, tilt=-25.0)
        state.initialized = True

        vs = VideoServer(
            state,
            tile_url_template=self.tile_url,
            terrain_engine=engine,
            width=640,
            height=360
        )
        self.assertTrue(vs.is_3d_terrain_active)

        # Prepopulate tile cache with mock tile images
        mock_tile = np.full((256, 256, 3), 80, dtype=np.uint8)
        mock_tile[:, :, 1] = 160
        for tx in range(5000, 5020):
            for ty in range(5000, 5020):
                vs.tile_cache[(14, tx, ty)] = mock_tile

        import asyncio
        asyncio.run(vs._update_frame())

        self.assertIsNotNone(vs.current_frame)
        self.assertEqual(vs.current_frame.shape, (360, 640, 3))

    def test_video_server_fallback_when_libraries_missing(self):
        """Verify that when 3D draped terrain is enabled but libraries are unavailable, fallback generates frame gracefully."""
        engine = TerrainEngine(self.slope_dted)
        state = GimbalState(dt=0.1, terrain_engine=engine, lat=34.2, lon=-118.5, alt=1200.0, tilt=-25.0)
        state.initialized = True

        vs = VideoServer(
            state,
            tile_url_template=self.tile_url,
            terrain_engine=engine,
            width=640,
            height=360
        )
        self.assertTrue(vs.is_3d_terrain_active)

        import asyncio
        asyncio.run(vs._update_frame())

        self.assertIsNotNone(vs.current_frame)


if __name__ == "__main__":
    unittest.main()

