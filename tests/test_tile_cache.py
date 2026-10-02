import os
import shutil
import tempfile
import unittest

from orion_shadow.core.state import GimbalState
from orion_shadow.engine.video_server import VideoServer

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None


class TestTileDiskCache(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp(prefix="orion_tile_cache_")
        self.state = GimbalState()

    def tearDown(self):
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_tile_disk_path_generation(self):
        """Verify standard XYZ directory hierarchy file path generation."""
        vs = VideoServer(self.state, tile_cache_dir=self.temp_dir)
        path = vs._get_tile_disk_path(14, 2500, 3600)
        expected = os.path.join(self.temp_dir, "14", "2500", "3600.png")
        self.assertEqual(path, expected)

    def test_load_existing_tile_from_disk(self):
        """Verify that pre-cached tile on disk is loaded into memory without network."""
        if cv2 is None or np is None:
            self.skipTest("cv2 or numpy not installed")

        vs = VideoServer(self.state, tile_cache_dir=self.temp_dir)

        # Manually create a cached tile file on disk
        z, x, y = 15, 100, 200
        tile_path = vs._get_tile_disk_path(z, x, y)
        os.makedirs(os.path.dirname(tile_path), exist_ok=True)

        mock_tile = np.zeros((256, 256, 3), dtype=np.uint8)
        mock_tile[:, :] = (120, 150, 90)
        cv2.imwrite(tile_path, mock_tile)

        # Ensure tile is NOT yet in memory cache
        self.assertNotIn((z, x, y), vs.tile_cache)

        # Retrieve via _get_tile_or_parent -> should find on disk and populate memory cache
        loaded = vs._get_tile_or_parent(z, x, y)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.shape, (256, 256, 3))
        self.assertIn((z, x, y), vs.tile_cache)

    def test_load_parent_tile_from_disk(self):
        """Verify fallback to lower zoom level parent tile stored on disk."""
        if cv2 is None or np is None:
            self.skipTest("cv2 or numpy not installed")

        vs = VideoServer(self.state, tile_cache_dir=self.temp_dir)

        # Create parent tile at zoom 14 (child is at zoom 15)
        # For child (15, 200, 400), parent at z=14 is (14, 200 >> 1, 400 >> 1) = (14, 100, 200)
        pz, px, py = 14, 100, 200
        parent_path = vs._get_tile_disk_path(pz, px, py)
        os.makedirs(os.path.dirname(parent_path), exist_ok=True)

        mock_parent = np.zeros((256, 256, 3), dtype=np.uint8)
        mock_parent[:, :] = (70, 110, 160)
        cv2.imwrite(parent_path, mock_parent)

        # Request child tile (15, 200, 400) which is missing on disk and memory
        child = vs._get_tile_or_parent(15, 200, 400)
        self.assertIsNotNone(child)
        self.assertEqual(child.shape, (256, 256, 3))

    def test_fetch_worker_checks_disk_before_network(self):
        """Verify _fetch_tile_worker loads from disk if present without attempting network request."""
        if cv2 is None or np is None:
            self.skipTest("cv2 or numpy not installed")

        vs = VideoServer(self.state, tile_url_template="http://invalid.nowhere/{z}/{x}/{y}.png", tile_cache_dir=self.temp_dir)
        z, x, y = 14, 50, 60
        tile_path = vs._get_tile_disk_path(z, x, y)
        os.makedirs(os.path.dirname(tile_path), exist_ok=True)

        mock_img = np.full((256, 256, 3), 42, dtype=np.uint8)
        cv2.imwrite(tile_path, mock_img)

        import asyncio
        asyncio.run(vs._fetch_tile_worker(z, x, y))

        self.assertIn((z, x, y), vs.tile_cache)
        self.assertEqual(vs.tile_cache[(z, x, y)][0, 0, 0], 42)


if __name__ == "__main__":
    unittest.main()
