import unittest
import struct
from orion_shadow.core.protocol import OrionPacket, OrionPktType
from orion_shadow.core.state import GimbalState

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None


class TestVideoTracking(unittest.TestCase):
    def test_video_options_update(self):
        state = GimbalState()
        # width(u16), height(u16), fps(u8): 1280, 720, 60
        data = struct.pack(">HHB", 1280, 720, 60)
        packet = OrionPacket(OrionPktType.VIDEO_OPTIONS, data)
        state.update_from_command(packet)

        self.assertEqual(state.video_resolution_width, 1280)
        self.assertEqual(state.video_resolution_height, 720)
        self.assertEqual(state.video_fps, 60)

    def test_tracking_options_update(self):
        state = GimbalState()
        # target_id(u32), mode(u8): ID=42, Mode=2
        data = struct.pack(">IB", 42, 2)
        packet = OrionPacket(OrionPktType.TRACK_OPTIONS, data)
        state.update_from_command(packet)

        self.assertEqual(state.tracking_target_id, 42)
        self.assertEqual(state.tracking_mode, 2)

    @unittest.skipIf(np is None, "numpy not installed")
    def test_video_server_dynamic_updates_tilt_and_heading(self):
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState()
        server = VideoServer(state)

        state.gps_lat = 39.7774
        state.gps_lon = -84.0819
        state.gps_alt = 2000.0
        state.aircraft_heading = 90.0
        state.target_tilt = -30.0
        state.physics.tilt["pos"] = -30.0

        frame_base = server._generate_synthetic_background()
        self.assertIsInstance(frame_base, np.ndarray)

        # Change tilt: frame must update
        state.target_tilt = 10.0
        state.physics.tilt["pos"] = 10.0
        frame_tilt = server._generate_synthetic_background()
        self.assertFalse(np.array_equal(frame_base, frame_tilt), "Video frame should update when gimbal tilts")

        # Change heading: frame must update
        state.aircraft_heading = 180.0
        frame_hdg = server._generate_synthetic_background()
        self.assertFalse(np.array_equal(frame_tilt, frame_hdg), "Video frame should update when aircraft/gimbal heading changes")

    @unittest.skipIf(np is None, "numpy not installed")
    def test_video_server_dynamic_updates_adsb_movement(self):
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState()
        server = VideoServer(state)

        state.gps_lat = 39.7774
        state.gps_lon = -84.0819
        state.gps_alt = 3000.0
        state.aircraft_heading = 240.0
        state.target_tilt = -20.0
        state.physics.tilt["pos"] = -20.0

        frame_initial = server._generate_synthetic_background()

        # Move aircraft forward along flight path
        state.gps_lat += 0.005
        state.gps_lon -= 0.005
        frame_moved = server._generate_synthetic_background()

        self.assertFalse(np.array_equal(frame_initial, frame_moved), "Video frame should update as attached aircraft flies")

    @unittest.skipIf(np is None, "numpy not installed")
    def test_video_server_zoom_rendering_stability(self):
        """Test that synthetic background renders cleanly across various zoom levels."""
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState()
        server = VideoServer(state, width=640, height=480)
        state.gps_lat = 39.7774
        state.gps_lon = -84.0819
        state.gps_alt = 2000.0
        state.target_tilt = -15.0
        state.physics.tilt["pos"] = -15.0

        # Wide angle 1.0x
        state.camera_zoom = 1.0
        frame_wide = server._generate_synthetic_background()
        self.assertIsInstance(frame_wide, np.ndarray)
        self.assertEqual(frame_wide.shape, (480, 640, 3))

        # Zoom in to 5.0x
        state.camera_zoom = 5.0
        frame_zoomed = server._generate_synthetic_background()
        self.assertIsInstance(frame_zoomed, np.ndarray)
        self.assertEqual(frame_zoomed.shape, (480, 640, 3))
        self.assertFalse(np.array_equal(frame_wide, frame_zoomed))

        # Zoom in further to 20.0x
        state.camera_zoom = 20.0
        frame_high_zoom = server._generate_synthetic_background()
        self.assertIsInstance(frame_high_zoom, np.ndarray)
        self.assertEqual(frame_high_zoom.shape, (480, 640, 3))

        # Zoom out back to 1.0x
        state.camera_zoom = 1.0
        frame_restored = server._generate_synthetic_background()
        self.assertIsInstance(frame_restored, np.ndarray)
        self.assertEqual(frame_restored.shape, (480, 640, 3))
        self.assertTrue(np.array_equal(frame_wide, frame_restored), "Wide frame should match when zoomed back to 1.0x")

    @unittest.skipIf(np is None or cv2 is None, "numpy or cv2 not installed")
    def test_video_server_hud_rendering(self):
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState()
        server = VideoServer(state)
        state.gps_lat = 39.7774
        state.gps_lon = -84.0819
        state.gps_alt = 1500.0

        bg = server._generate_synthetic_background()
        hud = server._draw_hud(bg.copy())
        self.assertFalse(np.array_equal(bg, hud), "HUD should render overlay on the video frame")

    def test_video_server_pure_python_fallback(self):
        import orion_shadow.engine.video_server as vs

        orig_np = vs.np
        try:
            vs.np = None
            state = GimbalState()
            server = vs.VideoServer(state, width=640, height=480)
            frame = server._generate_synthetic_background()
            self.assertIsInstance(frame, bytes)
            self.assertEqual(len(frame), 640 * 480 * 3)
        finally:
            vs.np = orig_np

    def test_video_server_telemetry_tilt_consistency(self):
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState(dt=0.1)
        server = VideoServer(state)

        # Command -45.0 tilt (typical ADS-B attachment angle)
        state.target_tilt = -45.0

        # Physics should settle onto target within 30 steps without overshoot
        overshot = False
        for _ in range(30):
            state.step()
            if state.physics.tilt["pos"] < -45.5:
                overshot = True
                break
        self.assertFalse(overshot, f"Tilt overshot -45.0: {state.physics.tilt['pos']}")
        self.assertAlmostEqual(state.physics.tilt["pos"], -45.0, delta=0.1)

        # Telemetry should now show settled position
        telem_settled = server._get_telemetry()
        self.assertAlmostEqual(telem_settled['tilt'], -45.0, places=1)
        self.assertAlmostEqual(telem_settled['cam_pitch'], -45.0, places=1)

    def test_footprint_shallow_pitch_zoom_continuity(self):
        """Verify that compute_footprint does not collapse or return None past 5x zoom at shallow pitch."""
        from orion_shadow.engine.visualizer import TileVisualizer
        from orion_shadow.engine.video_server import VideoServer

        state = GimbalState()
        server = VideoServer(state, tile_url_template="http://dummy/{z}/{x}/{y}.png")
        vis = server.visualizer

        lat, lon, alt = 39.7774, -84.0819, 1000.0

        for pitch in [-1.0, -5.0, -15.0, -30.0]:
            for zoom in [1.0, 3.0, 5.0, 6.0, 10.0, 20.0]:
                hfov, vfov, _, _ = server._get_fov(zoom)
                tile_z = server._compute_tile_zoom(lat, alt, pitch, hfov)
                fp = vis.compute_footprint(lat, lon, alt, 0.0, pitch, hfov, vfov, tile_z)

                self.assertIsNotNone(fp, f"Footprint returned None at pitch={pitch}, zoom={zoom}x")
                self.assertIn("corners_tile_frac", fp)
                c = fp["corners_tile_frac"]
                self.assertEqual(len(c), 4)

                dx = max(p[0] for p in c) - min(p[0] for p in c)
                dy = max(p[1] for p in c) - min(p[1] for p in c)
                span = max(dx, dy)
                self.assertGreater(span, 1e-4, f"Footprint collapsed into collinear line at zoom {zoom}x")

        # Pitch 0.0 (level) should return valid footprint at wide/moderate zooms (1x-5x)
        for zoom in [1.0, 3.0, 5.0]:
            hfov, vfov, _, _ = server._get_fov(zoom)
            tile_z = server._compute_tile_zoom(lat, alt, 0.0, hfov)
            fp = vis.compute_footprint(lat, lon, alt, 0.0, 0.0, hfov, vfov, tile_z)
            self.assertIsNotNone(fp, f"Footprint returned None at pitch=0.0, zoom={zoom}x")


if __name__ == "__main__":
    unittest.main()
