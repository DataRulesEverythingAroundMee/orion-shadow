import unittest
import math
import struct
import time
import asyncio
import threading
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts")))

from unittest.mock import MagicMock, patch

from adsb_attach import (
    haversine_nm,
    calculate_bearing,
    degrees_to_cardinal,
    extrapolate_position,
    ADSBClient,
    Aircraft,
    OrionBridge,
    ADSBTrackerTUI
)
from orion_shadow.server import OrionServer
from orion_shadow.core.protocol import OrionPacket, OrionPktType


class TestADSBAttach(unittest.TestCase):

    def test_haversine_distance(self):
        # Known points: LAX (33.9425, -118.4081) to SFO (37.6213, -122.3790) ~ 293 NM
        dist = haversine_nm(33.9425, -118.4081, 37.6213, -122.3790)
        self.assertAlmostEqual(dist, 293.5, delta=1.0)

        # Same point distance should be 0
        self.assertAlmostEqual(haversine_nm(34.0, -118.0, 34.0, -118.0), 0.0, places=5)

    def test_calculate_bearing_and_cardinal(self):
        # Due north
        brg_n = calculate_bearing(34.0, -118.0, 35.0, -118.0)
        self.assertAlmostEqual(brg_n, 0.0, delta=0.5)
        self.assertEqual(degrees_to_cardinal(brg_n), "N")

        # Due east
        brg_e = calculate_bearing(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(brg_e, 90.0, delta=0.5)
        self.assertEqual(degrees_to_cardinal(brg_e), "E")

        # Due south
        brg_s = calculate_bearing(35.0, -118.0, 34.0, -118.0)
        self.assertAlmostEqual(brg_s, 180.0, delta=0.5)
        self.assertEqual(degrees_to_cardinal(brg_s), "S")

    def test_extrapolate_position(self):
        # Flying due North at 360 knots for 10 seconds -> 1 NM north
        # 1 NM latitude ~ 1/60 degree ~ 0.016667 degrees
        lat, lon = extrapolate_position(34.0, -118.0, speed_kts=360.0, track_deg=0.0, dt_sec=10.0)
        self.assertGreater(lat, 34.0)
        self.assertAlmostEqual(lon, -118.0, places=4)
        dist = haversine_nm(34.0, -118.0, lat, lon)
        self.assertAlmostEqual(dist, 1.0, places=2)

    def test_parse_aircraft_entry(self):
        client = ADSBClient(force_mock=True)
        raw_ac = {
            "hex": "a1234b",
            "flight": "UAL123  ",
            "lat": 34.1,
            "lon": -118.3,
            "alt_baro": 25000,
            "track": 270.0,
            "gs": 450.0,
            "t": "B738",
            "r": "N123UA",
            "squawk": "3412"
        }
        ac = client._parse_aircraft_entry(raw_ac, 34.0522, -118.2437)
        self.assertIsNotNone(ac)
        self.assertEqual(ac.hex, "A1234B")
        self.assertEqual(ac.flight, "UAL123")
        self.assertEqual(ac.alt_baro, 25000)
        self.assertAlmostEqual(ac.alt_meters, 25000 * 0.3048, places=2)
        self.assertEqual(ac.speed, 450.0)
        self.assertEqual(ac.track, 270.0)
        self.assertEqual(ac.type_code, "B738")
        self.assertEqual(ac.reg, "N123UA")
        self.assertEqual(ac.squawk, "3412")
        self.assertGreater(ac.distance_nm, 0.0)

    def test_mock_aircraft_generation_and_sorting(self):
        client = ADSBClient(force_mock=True)
        aircraft_list = client.fetch_closest_aircraft(34.0522, -118.2437, radius_nm=100.0, limit=50)

        self.assertEqual(len(aircraft_list), 50)
        # Check ascending sort by distance
        for i in range(len(aircraft_list) - 1):
            self.assertLessEqual(aircraft_list[i].distance_nm, aircraft_list[i + 1].distance_nm)

    def test_fetch_single_aircraft_mock(self):
        client = ADSBClient(force_mock=True)
        aircraft_list = client.fetch_closest_aircraft(34.0522, -118.2437, radius_nm=100.0, limit=10)
        first_hex = aircraft_list[0].hex
        
        # Poll the single aircraft
        fresh = client.fetch_single_aircraft(first_hex, 34.0522, -118.2437)
        self.assertIsNotNone(fresh)
        self.assertEqual(fresh.hex, first_hex)
        self.assertGreater(fresh.speed, 0.0)

    def test_orion_bridge_end_to_end(self):
        port = 5990
        server = OrionServer(host="127.0.0.1", port=port, video_enabled=False)
        server_thread = threading.Thread(target=lambda: asyncio.run(server.run()), daemon=True)
        server_thread.start()
        time.sleep(0.4)

        bridge = OrionBridge(host="127.0.0.1", port=port)
        self.assertTrue(bridge.connect())
        self.assertTrue(bridge.is_connected)

        # Send test aircraft coordinates
        success = bridge.send_aircraft_position(
            lat=34.12345,
            lon=-118.54321,
            alt_meters=3500.0,
            heading_deg=180.0
        )
        self.assertTrue(success)
        time.sleep(0.3)

        self.assertTrue(server.state.initialized)
        self.assertAlmostEqual(server.state.gps_lat, 34.12345, places=4)
        self.assertAlmostEqual(server.state.gps_lon, -118.54321, places=4)
        self.assertAlmostEqual(server.state.gps_alt, 3500.0, places=1)
        self.assertAlmostEqual(server.state.aircraft_heading, 180.0, places=1)
        self.assertAlmostEqual(server.state.target_tilt, 20.0, places=1)

        bridge.close()

    def test_tui_rendering_bounds(self):
        """Verify TUI rendering does not crash across various terminal dimensions (including 80x24)."""
        with patch("curses.color_pair", return_value=0):
            client = ADSBClient(force_mock=True)
            bridge = OrionBridge(host="127.0.0.1", port=8745, tilt_deg=20.0)
            tui = ADSBTrackerTUI(client, bridge, 34.05, -118.25)
            tui.aircraft_list = client.fetch_closest_aircraft(34.05, -118.25)
            tui.attached_aircraft = tui.aircraft_list[0]
            for i in range(10):
                tui.flight_trail.append((time.time() - i, 34.05, -118.25, 25000))

            for h in [24, 20, 15, 10, 5, 40]:
                for w in [80, 76, 60, 40, 120]:
                    stdscr = MagicMock()
                    stdscr.getmaxyx.return_value = (h, w)
                    # Verify no uncaught curses exceptions occur
                    tui._draw_attached_view(stdscr, h, w)
                    tui._draw_list_view(stdscr, h, w)


if __name__ == "__main__":
    unittest.main()
