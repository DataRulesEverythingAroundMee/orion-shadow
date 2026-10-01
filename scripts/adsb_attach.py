#!/usr/bin/env python3
"""
Orion Shadow - ADS-B Aircraft Attach & Telemetry Bridge
======================================================
Connects to open, keyless ADS-B endpoints (such as adsb.lol or opendata.adsb.fi,
or local dump1090/readsb feeds), retrieves and calculates the closest 50 aircraft
relative to a specified lat/lon geocoordinate, provides an interactive TUI to browse
and select an aircraft, and continuously streams the aircraft's lat/lon, altitude,
and heading coordinates to the Orion server so the gimbal/platform believes it is flying attached to it.

Author: Orion Shadow Project
"""

import os
import sys
import time
import math
import socket
import struct
import json
import argparse
import threading
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, field
import urllib.request
import urllib.error

# Ensure orion_shadow module is importable
SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    from orion_shadow.core.protocol import OrionPacket, OrionPktType
except ImportError:
    # Standalone fallback protocol definitions if run independently
    class OrionPktType:
        INITIALIZE = 0x00
        CMD = 0x01
        POSITIONS = 0x0A
        GPS_DATA = 0xD1
        EXT_HEADING_DATA = 0xD2

    @dataclass
    class OrionPacket:
        packet_id: int
        data: bytes

        def encode(self) -> bytes:
            payload = struct.pack(">BBBB", 0xD0, 0x0D, self.packet_id, len(self.data)) + self.data
            a, b = 1, 1
            for byte in payload:
                a = (a + byte) % 251
                b = (b + a) % 251
            checksum = (b << 8) | a
            return payload + struct.pack(">H", checksum)


# --- Geographic & Kinematic Calculations ---

EARTH_RADIUS_NM = 3440.065  # Nautical Miles
FEET_TO_METERS = 0.3048

def haversine_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate Great Circle distance between two points in Nautical Miles."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1 - a)))
    return EARTH_RADIUS_NM * c

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate initial bearing (degrees 0-360) from point 1 to point 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    initial_bearing = math.atan2(y, x)
    return (math.degrees(initial_bearing) + 360.0) % 360.0

def degrees_to_cardinal(deg: float) -> str:
    """Convert bearing degrees to 16-wind cardinal direction."""
    directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = round(deg / (360.0 / len(directions))) % len(directions)
    return directions[idx]

def extrapolate_position(lat: float, lon: float, speed_kts: float, track_deg: float, dt_sec: float) -> Tuple[float, float]:
    """Dead reckoning: extrapolate lat/lon forward in time given speed (kts) and track."""
    if speed_kts <= 0 or dt_sec <= 0:
        return lat, lon
    dist_nm = speed_kts * (dt_sec / 3600.0)
    delta = dist_nm / EARTH_RADIUS_NM
    theta = math.radians(track_deg)
    phi1 = math.radians(lat)
    lambda1 = math.radians(lon)

    phi2 = math.asin(math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta))
    lambda2 = lambda1 + math.atan2(
        math.sin(theta) * math.sin(delta) * math.cos(phi1),
        math.cos(delta) - math.sin(phi1) * math.sin(phi2)
    )
    return math.degrees(phi2), (math.degrees(lambda2) + 540.0) % 360.0 - 180.0


# --- Data Structures ---

@dataclass
class Aircraft:
    hex: str
    flight: str
    lat: float
    lon: float
    alt_baro: float  # feet
    track: float     # degrees (0-360)
    speed: float     # knots
    type_code: str = "N/A"
    reg: str = "N/A"
    squawk: str = "N/A"
    distance_nm: float = 0.0
    bearing_deg: float = 0.0
    last_update_time: float = field(default_factory=time.time)

    @property
    def alt_meters(self) -> float:
        return self.alt_baro * FEET_TO_METERS

    @property
    def bearing_cardinal(self) -> str:
        return degrees_to_cardinal(self.bearing_deg)

    def current_extrapolated_pos(self) -> Tuple[float, float]:
        """Compute current extrapolated position based on time elapsed since last fix."""
        dt = max(0.0, time.time() - self.last_update_time)
        # Cap extrapolation at 30 seconds to prevent runaway drift
        dt = min(dt, 30.0)
        return extrapolate_position(self.lat, self.lon, self.speed, self.track, dt)


# --- ADS-B API Client ---

class ADSBClient:
    """Fetches aircraft data from ADS-B endpoints or generates realistic mock data."""

    DEFAULT_ENDPOINTS = [
        "https://api.adsb.lol/v2/point/{lat}/{lon}/{radius}",
        "https://opendata.adsb.fi/api/v2/lat/{lat}/lon/{lon}/dist/{radius}",
    ]

    def __init__(self, endpoint: Optional[str] = None, force_mock: bool = False, api_key: Optional[str] = None):
        self.endpoint_raw = endpoint or "https://api.adsb.lol/v2/point/{lat}/{lon}/{radius}"
        self.force_mock = force_mock
        self.last_source_label = "Uninitialized"
        self._mock_aircraft_db: List[Dict[str, Any]] = []

    def _parse_aircraft_entry(self, ac: Dict[str, Any], ref_lat: float, ref_lon: float) -> Optional[Aircraft]:
        """Parse raw ADS-B dict into an Aircraft object."""
        # Find latitude and longitude
        lat = ac.get("lat")
        lon = ac.get("lon")
        if lat is None or lon is None:
            return None
        try:
            lat = float(lat)
            lon = float(lon)
        except (ValueError, TypeError):
            return None

        # Ignore invalid coordinates
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return None

        # Hex code (ICAO)
        hex_code = str(ac.get("hex", ac.get("icao", "UNKNOWN"))).strip().upper()

        # Flight / Callsign
        flight = str(ac.get("flight", ac.get("callsign", ac.get("r", hex_code)))).strip().upper()
        if not flight:
            flight = hex_code

        # Altitude
        raw_alt = ac.get("alt_baro", ac.get("alt_geom", ac.get("altitude", 0)))
        if raw_alt == "ground" or raw_alt is None:
            alt_ft = 0.0
        else:
            try:
                alt_ft = float(raw_alt)
            except (ValueError, TypeError):
                alt_ft = 0.0

        # Speed (knots)
        raw_spd = ac.get("gs", ac.get("speed", 0.0))
        try:
            speed = max(0.0, float(raw_spd) if raw_spd is not None else 0.0)
        except (ValueError, TypeError):
            speed = 0.0

        # Track (degrees)
        raw_track = ac.get("track", ac.get("heading", ac.get("true_heading", ac.get("mag_heading", 0.0))))
        try:
            track = (float(raw_track) if raw_track is not None else 0.0) % 360.0
        except (ValueError, TypeError):
            track = 0.0

        type_code = str(ac.get("t", ac.get("type", ac.get("desc", "N/A")))).strip()
        reg = str(ac.get("r", ac.get("reg", "N/A"))).strip()
        squawk = str(ac.get("squawk", "N/A")).strip()

        dist_nm = haversine_nm(ref_lat, ref_lon, lat, lon)
        bearing = calculate_bearing(ref_lat, ref_lon, lat, lon)

        return Aircraft(
            hex=hex_code,
            flight=flight,
            lat=lat,
            lon=lon,
            alt_baro=alt_ft,
            track=track,
            speed=speed,
            type_code=type_code if type_code else "N/A",
            reg=reg if reg else "N/A",
            squawk=squawk if squawk else "N/A",
            distance_nm=dist_nm,
            bearing_deg=bearing,
            last_update_time=time.time()
        )

    def _build_url(self, template: str, lat: float, lon: float, radius: float) -> str:
        """Interpolate placeholders in URL template."""
        url = template
        url = url.replace("{lat}", f"{lat:.4f}")
        url = url.replace("{lon}", f"{lon:.4f}")
        url = url.replace("{radius}", str(int(radius)))
        url = url.replace("{dist}", str(int(radius)))
        return url

    def _query_http(self, url: str) -> Optional[Dict[str, Any]]:
        """Perform HTTP GET request and return JSON object."""
        req = urllib.request.Request(url)
        req.add_header("User-Agent", "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36")
        req.add_header("Accept", "application/json")

        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode("utf-8", errors="replace"))
                    return data
        except Exception:
            return None
        return None

    def fetch_closest_aircraft(self, ref_lat: float, ref_lon: float, radius_nm: float = 100.0, limit: int = 50) -> List[Aircraft]:
        """
        Fetch aircraft within radius of (ref_lat, ref_lon), calculate distance,
        and return the closest `limit` (default 50) aircraft sorted ascending by distance.
        """
        if self.force_mock:
            self.last_source_label = "MOCK SIMULATOR (Offline Mode)"
            return self._get_mock_aircraft(ref_lat, ref_lon, limit=limit, radius_nm=radius_nm)

        # Build candidates list of URLs to try
        urls_to_try = []
        if self.endpoint_raw:
            if "{lat}" in self.endpoint_raw:
                urls_to_try.append(("Primary Endpoint", self._build_url(self.endpoint_raw, ref_lat, ref_lon, radius_nm)))
            elif self.endpoint_raw.endswith(".json"):
                urls_to_try.append(("Custom JSON feed", self.endpoint_raw))
            else:
                ep = self.endpoint_raw.rstrip("/")
                urls_to_try.append(("Custom Endpoint", f"{ep}/v2/point/{ref_lat:.4f}/{ref_lon:.4f}/{int(radius_nm)}"))

        # Fallback 100% open, keyless community endpoints
        fallbacks = [
            ("ADSB.lol", f"https://api.adsb.lol/v2/point/{ref_lat:.4f}/{ref_lon:.4f}/{int(radius_nm)}"),
            ("ADSB.fi", f"https://opendata.adsb.fi/api/v2/lat/{ref_lat:.4f}/lon/{ref_lon:.4f}/dist/{int(radius_nm)}"),
        ]
        for label, fb_url in fallbacks:
            if not any(fb_url == u[1] for u in urls_to_try):
                urls_to_try.append((label, fb_url))

        data = None
        used_label = ""
        for label, url in urls_to_try:
            data = self._query_http(url)
            if data is not None:
                used_label = label
                break

        if data is None:
            # If all live endpoints fail (offline, sandbox, DNS error, rate limit), fall back to realistic mock generator
            self.last_source_label = "MOCK SIMULATOR (Live Network Unavailable)"
            return self._get_mock_aircraft(ref_lat, ref_lon, limit=limit, radius_nm=radius_nm)

        self.last_source_label = used_label

        # Extract aircraft list from standard formats
        raw_list = []
        if isinstance(data, dict):
            if "ac" in data and isinstance(data["ac"], list):
                raw_list = data["ac"]
            elif "aircraft" in data and isinstance(data["aircraft"], list):
                raw_list = data["aircraft"]
        elif isinstance(data, list):
            raw_list = data

        parsed_aircraft: List[Aircraft] = []
        for item in raw_list:
            if isinstance(item, dict):
                ac_obj = self._parse_aircraft_entry(item, ref_lat, ref_lon)
                if ac_obj is not None:
                    parsed_aircraft.append(ac_obj)

        # Sort strictly by distance ascending and slice to top `limit` (50)
        parsed_aircraft.sort(key=lambda a: a.distance_nm)
        return parsed_aircraft[:limit]

    def fetch_single_aircraft(self, hex_code: str, fallback_lat: float, fallback_lon: float) -> Optional[Aircraft]:
        """Fetch fresh position for a specific tracked aircraft."""
        if self.force_mock or "MOCK" in self.last_source_label:
            # Update position in mock db
            for item in self._mock_aircraft_db:
                if item.get("hex", "").upper() == hex_code.upper():
                    # Move mock aircraft along track
                    dt = time.time() - item.get("last_time", time.time())
                    item["last_time"] = time.time()
                    speed = item.get("gs", item.get("speed", 0.0))
                    track = item.get("track", 0.0)
                    new_lat, new_lon = extrapolate_position(item["lat"], item["lon"], speed, track, dt)
                    item["lat"], item["lon"] = new_lat, new_lon
                    return self._parse_aircraft_entry(item, fallback_lat, fallback_lon)
            return None

        # Attempt to query hex endpoint on supported open, keyless APIs
        hex_urls = [
            f"https://api.adsb.lol/v2/hex/{hex_code.lower()}",
            f"https://opendata.adsb.fi/api/v2/hex/{hex_code.lower()}",
        ]
        for url in hex_urls:
            data = self._query_http(url)
            if data and isinstance(data, dict):
                entries = data.get("ac") or data.get("aircraft") or []
                for entry in entries:
                    if str(entry.get("hex", "")).strip().upper() == hex_code.upper():
                        return self._parse_aircraft_entry(entry, fallback_lat, fallback_lon)

        return None

    def _get_mock_aircraft(self, ref_lat: float, ref_lon: float, limit: int = 50, radius_nm: float = 80.0) -> List[Aircraft]:
        """Generate or update 50 realistic simulated aircraft flying in the area."""
        now = time.time()
        if not self._mock_aircraft_db or len(self._mock_aircraft_db) < limit:
            self._mock_aircraft_db = []
            airlines = ["AAL", "DAL", "UAL", "SWA", "BAW", "AFR", "DLH", "FDX", "UPS", "SKW", "ASA", "JBU"]
            types = ["B738", "A320", "B772", "B789", "A321", "A359", "C172", "PC12", "GLF6", "E75L"]

            for i in range(limit):
                # Spread aircraft across distances and angles
                angle_deg = (i * 37.5 + (i * 11) % 45) % 360.0
                dist = 2.0 + (i / float(limit)) * (radius_nm * 0.95)
                # Compute position at bearing and distance
                alat, alon = extrapolate_position(ref_lat, ref_lon, dist * 3600.0, angle_deg, 1.0)

                airline = airlines[i % len(airlines)]
                flight_no = 100 + (i * 37) % 8900
                flight = f"{airline}{flight_no}"
                hex_id = f"A{i+1:02X}{(i*73)%256:02X}"
                speed = 140.0 + (i * 23) % 340
                track = (angle_deg + 45.0 + (i * 33)) % 360.0
                alt = 3000.0 + ((i * 1370) % 35000)
                ac_type = types[i % len(types)]
                squawk = f"{(1200 + i * 37) % 7700:04d}"

                self._mock_aircraft_db.append({
                    "hex": hex_id,
                    "flight": flight,
                    "lat": alat,
                    "lon": alon,
                    "alt_baro": alt,
                    "track": track,
                    "gs": speed,
                    "t": ac_type,
                    "r": f"N{i+100}A",
                    "squawk": squawk,
                    "last_time": now
                })
        else:
            # Advance all mock aircraft positions realistically
            for item in self._mock_aircraft_db:
                dt = now - item.get("last_time", now)
                item["last_time"] = now
                if dt > 0:
                    nlat, nlon = extrapolate_position(item["lat"], item["lon"], item["gs"], item["track"], dt)
                    item["lat"] = nlat
                    item["lon"] = nlon

        results = []
        for item in self._mock_aircraft_db:
            ac_obj = self._parse_aircraft_entry(item, ref_lat, ref_lon)
            if ac_obj:
                results.append(ac_obj)

        results.sort(key=lambda a: a.distance_nm)
        return results[:limit]


# --- Orion Server Bridge ---

class OrionBridge:
    """Maintains a UDP link to the Orion server and streams GPS, heading, and gimbal/camera packets."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8745, tilt_deg: float = -45.0):
        self.host = host
        self.port = port
        self.tilt_deg = tilt_deg
        self.sock: Optional[socket.socket] = None
        self.is_connected = False
        self.packets_sent = 0
        self.last_sent_time = 0.0
        self.last_error = ""
        self._lock = threading.Lock()
        self._running = True
        self._drain_thread = threading.Thread(target=self._drain_incoming_loop, daemon=True)
        self._drain_thread.start()

    def connect(self) -> bool:
        """Establish or verify UDP connection to Orion server and configure camera tilt."""
        with self._lock:
            if self.is_connected and self.sock:
                return True
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setblocking(False)
                self.sock = s
                self.is_connected = True
                self.last_error = ""

                # Send INITIALIZE packet so the server brings subsystems online
                init_pkt = OrionPacket(OrionPktType.INITIALIZE, b"").encode()
                # Send CMD packet to set camera tilt (default: -45 degrees)
                cmd_payload = struct.pack(">ff", 0.0, float(self.tilt_deg))
                cmd_pkt = OrionPacket(OrionPktType.CMD, cmd_payload).encode()
                self.sock.sendto(init_pkt + cmd_pkt, (self.host, self.port))
                return True
            except Exception as e:
                self.is_connected = False
                self.sock = None
                self.last_error = str(e)
                return False

    def send_camera_tilt(self, tilt_deg: Optional[float] = None, pan_deg: float = 0.0) -> bool:
        """Send CMD packet (0x01) to set camera pan/tilt."""
        if tilt_deg is not None:
            self.tilt_deg = tilt_deg
        if not self.is_connected:
            if not self.connect():
                return False

        with self._lock:
            try:
                if not self.sock:
                    return False
                cmd_payload = struct.pack(">ff", float(pan_deg), float(self.tilt_deg))
                cmd_pkt = OrionPacket(OrionPktType.CMD, cmd_payload).encode()
                self.sock.sendto(cmd_pkt, (self.host, self.port))
                return True
            except Exception as e:
                self.last_error = str(e)
                return False

    def close(self):
        """Close UDP connection."""
        self._running = False
        with self._lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            self.is_connected = False

    def _drain_incoming_loop(self):
        """Continuously discard incoming telemetry echo to prevent UDP buffer congestion."""
        while self._running:
            time.sleep(0.05)
            if not self.is_connected or not self.sock:
                continue
            with self._lock:
                try:
                    if self.sock:
                        # Non-blocking read
                        _ = self.sock.recv(4096)
                except (BlockingIOError, InterruptedError):
                    pass
                except Exception as e:
                    self.is_connected = False
                    self.last_error = f"Socket error: {e}"
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None

    def send_aircraft_position(self, lat: float, lon: float, alt_meters: float, heading_deg: float) -> bool:
        """
        Send GPS_DATA and EXT_HEADING_DATA packets to Orion server via UDP.
        GPS_DATA payload: 3 floats (lat, lon, alt_meters) in Big-Endian.
        EXT_HEADING_DATA payload: 3 floats (heading, roll=0.0, pitch=0.0) in Big-Endian.
        """
        if not self.is_connected:
            if not self.connect():
                return False

        with self._lock:
            try:
                if not self.sock:
                    return False

                # Pack GPS_DATA (packet_id 0xD1)
                gps_payload = struct.pack(">fff", float(lat), float(lon), float(alt_meters))
                gps_packet = OrionPacket(OrionPktType.GPS_DATA, gps_payload).encode()

                # Pack EXT_HEADING_DATA (packet_id 0xD2)
                heading_payload = struct.pack(">fff", float(heading_deg), 0.0, 0.0)
                heading_packet = OrionPacket(OrionPktType.EXT_HEADING_DATA, heading_payload).encode()

                self.sock.sendto(gps_packet + heading_packet, (self.host, self.port))
                self.packets_sent += 1
                self.last_sent_time = time.time()
                return True
            except Exception as e:
                self.is_connected = False
                self.last_error = str(e)
                try:
                    if self.sock:
                        self.sock.close()
                except Exception:
                    pass
                self.sock = None
                return False


# --- Interactive Terminal User Interface (TUI) ---

class ADSBTrackerTUI:
    """Curses-based interactive TUI for browsing aircraft and flying attached."""

    MODE_LIST = "LIST"
    MODE_ATTACHED = "ATTACHED"

    def __init__(self, client: ADSBClient, bridge: OrionBridge, center_lat: float, center_lon: float,
                 radius_nm: float = 100.0, limit: int = 50, update_rate_hz: float = 2.0):
        self.client = client
        self.bridge = bridge
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.radius_nm = radius_nm
        self.limit = limit
        self.update_interval = 1.0 / max(0.1, update_rate_hz)

        self.mode = self.MODE_LIST
        self.aircraft_list: List[Aircraft] = []
        self.selected_index = 0
        self.scroll_offset = 0
        self.attached_aircraft: Optional[Aircraft] = None

        self.last_fetch_time = 0.0
        self.last_send_time = 0.0
        self.status_message = "Ready. Press Enter on an aircraft to attach."
        self.running = True

        # Flight history breadcrumbs (timestamp, lat, lon, alt_ft)
        self.flight_trail: List[Tuple[float, float, float, float]] = []

    def _safe_addstr(self, stdscr, y: int, x: int, text: str, attr: int = 0):
        """Safely write string to curses window without overflowing boundaries or raising ERR."""
        import curses
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or y >= max_y or x < 0 or x >= max_x:
            return
        avail = max_x - x
        if y == max_y - 1 and len(text) >= avail:
            trimmed = text[:avail - 1]
        else:
            trimmed = text[:avail]
        if not trimmed:
            return
        try:
            if attr:
                stdscr.addstr(y, x, trimmed, attr)
            else:
                stdscr.addstr(y, x, trimmed)
        except curses.error:
            pass

    def run(self):
        """Entry point that initializes curses or falls back cleanly."""
        if not sys.stdin.isatty():
            print("[!] Standard input is not a terminal. Please use --headless mode.")
            return

        import curses
        curses.wrapper(self._curses_main)

    def _curses_main(self, stdscr):
        import curses

        # Setup curses environment
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)  # 100ms key polling
        curses.start_color()
        curses.use_default_colors()

        # Initialize color pairs
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)    # Highlight Header / Banner
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)    # Selected row
        curses.init_pair(3, curses.COLOR_GREEN, -1)                  # Success / Connected
        curses.init_pair(4, curses.COLOR_YELLOW, -1)                 # Accent / Telemetry
        curses.init_pair(5, curses.COLOR_RED, -1)                    # Error / Disconnected
        curses.init_pair(6, curses.COLOR_CYAN, -1)                   # Table headers / Labels
        curses.init_pair(7, curses.COLOR_MAGENTA, -1)                # Special / Alt

        # Initial fetch
        self._refresh_aircraft_list()

        # Connect to Orion bridge early and configure camera tilt
        self.bridge.connect()
        self.bridge.send_camera_tilt(self.bridge.tilt_deg)

        while self.running:
            now = time.time()

            # Handle periodic background tasks
            if self.mode == self.MODE_LIST:
                # Periodic list refresh every 8 seconds
                if now - self.last_fetch_time >= 8.0:
                    self._refresh_aircraft_list()
            elif self.mode == self.MODE_ATTACHED and self.attached_aircraft:
                # Periodic position stream to Orion Server
                if now - self.last_send_time >= self.update_interval:
                    self._tick_attached_flight()

                # Refresh aircraft data from API every 4 seconds
                if now - self.last_fetch_time >= 4.0:
                    self._poll_attached_aircraft()

            # Render display
            stdscr.erase()
            max_y, max_x = stdscr.getmaxyx()

            if max_y < 12 or max_x < 50:
                msg = f"Terminal too small ({max_x}x{max_y}). Min: 80x24"
                self._safe_addstr(stdscr, max_y // 2, max(0, (max_x - len(msg)) // 2), msg, curses.color_pair(5))
                stdscr.refresh()
                try:
                    ch = stdscr.getch()
                    if ch in (ord('q'), ord('Q')):
                        self.running = False
                except Exception:
                    pass
                time.sleep(0.1)
                continue

            if self.mode == self.MODE_LIST:
                self._draw_list_view(stdscr, max_y, max_x)
            else:
                self._draw_attached_view(stdscr, max_y, max_x)

            stdscr.refresh()

            # Handle user input
            try:
                ch = stdscr.getch()
            except Exception:
                ch = -1

            if ch != -1:
                self._handle_input(ch, stdscr)

    def _refresh_aircraft_list(self):
        """Fetch closest aircraft from ADS-B endpoint."""
        self.aircraft_list = self.client.fetch_closest_aircraft(
            self.center_lat, self.center_lon, self.radius_nm, self.limit
        )
        self.last_fetch_time = time.time()
        if self.selected_index >= len(self.aircraft_list):
            self.selected_index = max(0, len(self.aircraft_list) - 1)

    def _poll_attached_aircraft(self):
        """Poll API for position update of attached aircraft."""
        if not self.attached_aircraft:
            return
        fresh = self.client.fetch_single_aircraft(
            self.attached_aircraft.hex, self.center_lat, self.center_lon
        )
        if fresh:
            self.attached_aircraft = fresh
        self.last_fetch_time = time.time()

    def _tick_attached_flight(self):
        """Compute current extrapolated position and transmit to Orion Server."""
        if not self.attached_aircraft:
            return

        cur_lat, cur_lon = self.attached_aircraft.current_extrapolated_pos()
        alt_m = self.attached_aircraft.alt_meters
        track = self.attached_aircraft.track

        # Send to Orion
        success = self.bridge.send_aircraft_position(cur_lat, cur_lon, alt_m, track)
        self.last_send_time = time.time()

        # Update flight trail breadcrumbs
        self.flight_trail.append((self.last_send_time, cur_lat, cur_lon, self.attached_aircraft.alt_baro))
        if len(self.flight_trail) > 10:
            self.flight_trail.pop(0)

        if success:
            self.status_message = f"[+] Sent fix to Orion: Lat {cur_lat:+.5f}, Lon {cur_lon:+.5f}, Alt {alt_m:.1f}m"
        else:
            self.status_message = f"[!] Failed to send to Orion ({self.bridge.last_error})"

    def _handle_input(self, ch: int, stdscr):
        import curses
        if ch in (ord('q'), ord('Q')):
            self.running = False

        elif self.mode == self.MODE_LIST:
            if ch in (curses.KEY_UP, ord('k')):
                if self.selected_index > 0:
                    self.selected_index -= 1
            elif ch in (curses.KEY_DOWN, ord('j')):
                if self.selected_index < len(self.aircraft_list) - 1:
                    self.selected_index += 1
            elif ch == curses.KEY_PPAGE:
                self.selected_index = max(0, self.selected_index - 10)
            elif ch == curses.KEY_NPAGE:
                self.selected_index = min(max(0, len(self.aircraft_list) - 1), self.selected_index + 10)
            elif ch == curses.KEY_HOME:
                self.selected_index = 0
            elif ch == curses.KEY_END:
                self.selected_index = max(0, len(self.aircraft_list) - 1)
            elif ch in (ord('r'), ord('R')):
                self.status_message = "Refreshing aircraft list..."
                self._refresh_aircraft_list()
                self.status_message = f"Refreshed. Found {len(self.aircraft_list)} aircraft."
            elif ch in (ord('m'), ord('M')):
                self.client.force_mock = not self.client.force_mock
                self._refresh_aircraft_list()
                self.status_message = f"Toggled data source. Mock: {self.client.force_mock}"
            elif ch in (10, 13, curses.KEY_ENTER, ord(' ')):
                if self.aircraft_list and 0 <= self.selected_index < len(self.aircraft_list):
                    self.attached_aircraft = self.aircraft_list[self.selected_index]
                    self.mode = self.MODE_ATTACHED
                    self.flight_trail = []
                    self.status_message = f"Attached to {self.attached_aircraft.flight} ({self.attached_aircraft.hex}). Streaming telemetry..."
                    self._tick_attached_flight()
            elif ch in (ord('c'), ord('C')):
                self._prompt_new_coordinates(stdscr)

        elif self.mode == self.MODE_ATTACHED:
            if ch in (27, ord('b'), ord('B'), curses.KEY_BACKSPACE, 127):
                # Detach and return to list
                self.mode = self.MODE_LIST
                self.attached_aircraft = None
                self.status_message = "Detached from aircraft. Returned to selector."
                self._refresh_aircraft_list()
            elif ch in (ord('r'), ord('R'), ord(' ')):
                # Force immediate re-poll and send
                self._poll_attached_aircraft()
                self._tick_attached_flight()

    def _prompt_new_coordinates(self, stdscr):
        """Prompt user to change reference lat/lon interactively."""
        import curses
        curses.echo()
        curses.curs_set(1)
        max_y, max_x = stdscr.getmaxyx()
        win_w = min(60, max_x - 4)
        win_h = 5
        win_y = max(0, max_y // 2 - 2)
        win_x = max(2, (max_x - 60) // 2)
        if win_w < 20 or max_y < 6:
            curses.noecho()
            curses.curs_set(0)
            return

        prompt_win = curses.newwin(win_h, win_w, win_y, win_x)
        prompt_win.box()
        self._safe_addstr(prompt_win, 1, 2, "Enter new Latitude Longitude (e.g. 34.05 -118.25):", curses.color_pair(4))
        prompt_win.refresh()

        try:
            val = prompt_win.getstr(2, 2, 40).decode("utf-8").strip()
            parts = val.replace(",", " ").split()
            if len(parts) >= 2:
                nlat, nlon = float(parts[0]), float(parts[1])
                self.center_lat, self.center_lon = nlat, nlon
                self.status_message = f"Updated reference center to ({nlat:.4f}, {nlon:.4f})"
                self._refresh_aircraft_list()
        except Exception as e:
            self.status_message = f"Invalid coordinates: {e}"

        curses.noecho()
        curses.curs_set(0)

    def _draw_list_view(self, stdscr, max_y: int, max_x: int):
        import curses

        # Header Title
        title = " ORION SHADOW - ADS-B AIRCRAFT ATTACH & TRACKER "
        self._safe_addstr(stdscr, 0, 0, title.ljust(max_x - 1)[:max_x - 1], curses.color_pair(1) | curses.A_BOLD)

        # Subtitle / Configuration status line
        orion_color = curses.color_pair(3) if self.bridge.is_connected else curses.color_pair(5)
        orion_status = "CONNECTED" if self.bridge.is_connected else "DISCONNECTED"
        ref_info = f"CENTER: ({self.center_lat:+.4f}, {self.center_lon:+.4f}) | RADIUS: {self.radius_nm:.0f}NM | SOURCE: {self.client.last_source_label}"
        self._safe_addstr(stdscr, 1, 1, ref_info[:max_x - 2], curses.color_pair(4))

        bridge_prefix = f"ORION SERVER: {self.bridge.host}:{self.bridge.port} ["
        self._safe_addstr(stdscr, 2, 1, bridge_prefix)
        status_x = 1 + len(bridge_prefix)
        self._safe_addstr(stdscr, 2, status_x, orion_status, orion_color | curses.A_BOLD)
        suffix = f"] | TRACKED: {len(self.aircraft_list)} closest aircraft | MODE: SELECTOR"
        self._safe_addstr(stdscr, 2, status_x + len(orion_status), suffix)

        # Table Column Headers
        col_header = (
            f" {'#':>2} | {'CALLSIGN':<8} | {'HEX':<6} | {'TYPE':<4} | "
            f"{'DIST(NM)':>8} | {'BEARING':<8} | {'ALT(FT)':>8} | "
            f"{'SPD(KT)':>7} | {'TRACK':>5} | {'LATITUDE':>9} | {'LONGITUDE':>10} | {'SQWK':<4}"
        )
        self._safe_addstr(stdscr, 4, 0, col_header.ljust(max_x - 1)[:max_x - 1], curses.color_pair(6) | curses.A_BOLD)

        # Aircraft Rows
        visible_rows = max(0, max_y - 8)
        if visible_rows > 0:
            if self.selected_index < self.scroll_offset:
                self.scroll_offset = self.selected_index
            elif self.selected_index >= self.scroll_offset + visible_rows:
                self.scroll_offset = self.selected_index - visible_rows + 1

            for i in range(visible_rows):
                row_idx = self.scroll_offset + i
                screen_y = 5 + i
                if row_idx >= len(self.aircraft_list):
                    break

                ac = self.aircraft_list[row_idx]
                alt_str = "GND" if ac.alt_baro == 0 else f"{int(ac.alt_baro):,}"
                bearing_str = f"{int(ac.bearing_deg):03d}° {ac.bearing_cardinal}"

                row_text = (
                    f" {row_idx+1:>2} | {ac.flight:<8} | {ac.hex:<6} | {ac.type_code:<4} | "
                    f"{ac.distance_nm:>7.1f}m | {bearing_str:<8} | {alt_str:>8} | "
                    f"{int(ac.speed):>7} | {int(ac.track):>4}° | {ac.lat:>+9.4f} | {ac.lon:>+10.4f} | {ac.squawk:<4}"
                )

                if row_idx == self.selected_index:
                    self._safe_addstr(stdscr, screen_y, 0, row_text.ljust(max_x - 1)[:max_x - 1], curses.color_pair(2) | curses.A_BOLD)
                else:
                    self._safe_addstr(stdscr, screen_y, 0, row_text[:max_x - 1])

        # Status Bar
        if max_y > 2:
            self._safe_addstr(stdscr, max_y - 2, 1, f"STATUS: {self.status_message}"[:max_x - 2], curses.color_pair(4))

        # Footer Keybindings
        if max_y > 1:
            footer = "[↑/↓/j/k] Navigate  [ENTER] Attach & Fly Attached  [R] Refresh  [C] Change Coords  [M] Toggle Mock  [Q] Quit"
            self._safe_addstr(stdscr, max_y - 1, 0, footer.ljust(max_x - 1)[:max_x - 1], curses.color_pair(1))

    def _draw_attached_view(self, stdscr, max_y: int, max_x: int):
        import curses
        if not self.attached_aircraft:
            return

        ac = self.attached_aircraft
        cur_lat, cur_lon = ac.current_extrapolated_pos()
        alt_m = ac.alt_meters

        # Header Title
        title = f" ✈ ORION SHADOW - ATTACHED & FLYING WITH {ac.flight} ({ac.hex}) "
        self._safe_addstr(stdscr, 0, 0, title.ljust(max_x - 1)[:max_x - 1], curses.color_pair(1) | curses.A_BOLD)

        # Connection Banner
        orion_color = curses.color_pair(3) if self.bridge.is_connected else curses.color_pair(5)
        orion_status = "STREAMING ACTIVE" if self.bridge.is_connected else "RECONNECTING"
        self._safe_addstr(stdscr, 1, 2, "ORION SERVER LINK: ", curses.A_BOLD)
        status_text = f"{self.bridge.host}:{self.bridge.port} [{orion_status}]"
        self._safe_addstr(stdscr, 1, 21, status_text, orion_color | curses.A_BOLD)
        rate_hz = 1.0 / self.update_interval if self.update_interval > 0 else 0.0
        stats_text = f"  |  Packets Transmitted: {self.bridge.packets_sent} (Rate: {rate_hz:.1f}Hz)"
        self._safe_addstr(stdscr, 1, 21 + len(status_text), stats_text)

        # Box 1: Aircraft Telemetry (76 columns wide to fit safely in 80-column terminals)
        box_y = 3
        speed_str = f"{int(ac.speed)} kts ({ac.speed*1.852:.0f} km/h)"
        alt_disp = f"{int(ac.alt_baro):,} ft ({alt_m:.0f} m)"
        track_disp = f"{int(ac.track):03d}° [{degrees_to_cardinal(ac.track):<3}]"

        b1_content = [
            f"Callsign / Flight : {ac.flight:<12}       Registration : {ac.reg:<14}",
            f"ICAO Hex Address  : {ac.hex:<12}       Aircraft Type: {ac.type_code:<14}",
            f"Transponder Squawk: {ac.squawk:<12}       Ground Speed : {speed_str:<18}",
            f"Heading / Track   : {track_disp:<12}       Baro Altitude: {alt_disp:<18}",
        ]
        b1_lines = (
            ["╔" + "═" * 27 + " TARGET FLIGHT DATA " + "═" * 27 + "╗"]
            + [f"║ {c.ljust(72)} ║" for c in b1_content]
            + ["╚" + "═" * 74 + "╝"]
        )
        for offset, bline in enumerate(b1_lines):
            if box_y + offset < max_y - 2:
                self._safe_addstr(stdscr, box_y + offset, 2, bline, curses.color_pair(6))

        # Box 2: Orion Ingestion Coordinates
        box2_y = box_y + len(b1_lines)
        b2_content = [
            f"Injected Latitude : {cur_lat:>+12.6f}°   (GPS_DATA pkt 0xD1)",
            f"Injected Longitude: {cur_lon:>+12.6f}°   (GPS_DATA pkt 0xD1)",
            f"Injected Altitude : {alt_m:>12.1f}m    (GPS_DATA pkt 0xD1)",
            f"Injected Heading  : {ac.track:>12.1f}°   (EXT_HEADING_DATA pkt 0xD2)",
            f"Camera Tilt       : {self.bridge.tilt_deg:>12.1f}°   (CMD pkt 0x01)",
        ]
        b2_lines = (
            ["╔" + "═" * 24 + " ORION SIMULATOR INGESTION " + "═" * 23 + "╗"]
            + [f"║ {c.ljust(72)} ║" for c in b2_content]
            + ["╚" + "═" * 74 + "╝"]
        )
        for offset, bline in enumerate(b2_lines):
            if box2_y + offset < max_y - 2:
                self._safe_addstr(stdscr, box2_y + offset, 2, bline, curses.color_pair(4))

        # Flight Breadcrumbs / History (bounded so it never exceeds max_y - 2)
        trail_y = box2_y + len(b2_lines) + 1
        if trail_y < max_y - 2:
            self._safe_addstr(stdscr, trail_y, 2, "RECENT TELEMETRY FIXES (Dead Reckoning & Live Updates):", curses.A_BOLD)
            max_crumbs = max(0, (max_y - 2) - (trail_y + 1))
            crumbs_to_show = list(reversed(self.flight_trail))[:min(5, max_crumbs)]
            for idx, (ts, tlat, tlon, talt) in enumerate(crumbs_to_show):
                ago = max(0.0, time.time() - ts)
                line = f"  [{idx+1}] {ago:.1f}s ago -> Lat: {tlat:+.5f}°, Lon: {tlon:+.5f}°, Alt: {int(talt):,}ft"
                self._safe_addstr(stdscr, trail_y + 1 + idx, 4, line, curses.color_pair(7))

        # Status and Controls
        if max_y > 2:
            self._safe_addstr(stdscr, max_y - 2, 1, f"STATUS: {self.status_message}"[:max_x - 2], curses.color_pair(4))
        if max_y > 1:
            footer = "[ESC / B] Detach & Pick Another Aircraft  [SPACE] Force Resend  [Q] Quit"
            self._safe_addstr(stdscr, max_y - 1, 0, footer.ljust(max_x - 1)[:max_x - 1], curses.color_pair(1))


# --- Headless / CLI Mode ---

def run_headless(args: argparse.Namespace):
    """Run in non-interactive CLI mode for automated tests and headless environments."""
    client = ADSBClient(endpoint=args.endpoint, force_mock=args.mock)
    bridge = OrionBridge(host=args.orion_host, port=args.orion_port, tilt_deg=args.tilt)

    print(f"[*] Querying ADS-B endpoint: {client.endpoint_raw}")
    print(f"[*] Reference Center: ({args.lat:.4f}, {args.lon:.4f}) | Radius: {args.radius:.0f} NM | Limit: {args.limit}")

    aircraft_list = client.fetch_closest_aircraft(args.lat, args.lon, args.radius, args.limit)
    print(f"[+] Retrieved {len(aircraft_list)} closest aircraft (Source: {client.last_source_label}):\n")

    col_fmt = "{:>2} | {:<8} | {:<6} | {:<4} | {:>8} | {:<8} | {:>8} | {:>7} | {:>5} | {:>9} | {:>10} | {:<4}"
    print(col_fmt.format("#", "CALLSIGN", "HEX", "TYPE", "DIST(NM)", "BEARING", "ALT(FT)", "SPD(KT)", "TRACK", "LAT", "LON", "SQWK"))
    print("-" * 95)
    for idx, ac in enumerate(aircraft_list):
        alt_str = "GND" if ac.alt_baro == 0 else f"{int(ac.alt_baro):,}"
        brg_str = f"{int(ac.bearing_deg):03d}° {ac.bearing_cardinal}"
        print(col_fmt.format(
            idx + 1, ac.flight, ac.hex, ac.type_code,
            f"{ac.distance_nm:.1f}nm", brg_str, alt_str,
            int(ac.speed), f"{int(ac.track)}°",
            f"{ac.lat:+.4f}", f"{ac.lon:+.4f}", ac.squawk
        ))
    print("-" * 95)

    if not args.select:
        print("\n[i] Run with --select <callsign|hex|index|closest> to attach to an aircraft and stream coordinates.")
        return

    # Selection resolution
    selected_ac: Optional[Aircraft] = None
    sel = args.select.strip()
    if sel.lower() == "closest" or sel == "1":
        if aircraft_list:
            selected_ac = aircraft_list[0]
    elif sel.isdigit():
        idx = int(sel) - 1
        if 0 <= idx < len(aircraft_list):
            selected_ac = aircraft_list[idx]
    else:
        for ac in aircraft_list:
            if ac.flight.upper() == sel.upper() or ac.hex.upper() == sel.upper():
                selected_ac = ac
                break

    if not selected_ac:
        print(f"[!] Selected aircraft '{args.select}' not found in closest {len(aircraft_list)} list.")
        return

    print(f"\n[+] Selected Target: {selected_ac.flight} ({selected_ac.hex}) - {selected_ac.type_code}")
    print(f"[*] Connecting to Orion server at {args.orion_host}:{args.orion_port}...")
    print(f"[*] Setting camera tilt to {args.tilt:.1f}°...")

    bridge.connect()
    bridge.send_camera_tilt(args.tilt)
    interval = 1.0 / max(0.1, args.rate)
    count = 0
    max_count = args.count if args.count > 0 else float("inf")

    try:
        while count < max_count:
            cur_lat, cur_lon = selected_ac.current_extrapolated_pos()
            alt_m = selected_ac.alt_meters
            track = selected_ac.track

            success = bridge.send_aircraft_position(cur_lat, cur_lon, alt_m, track)
            count += 1

            status = "SENT" if success else f"FAIL ({bridge.last_error})"
            print(f"[{count:04d}] [{status}] Target: {selected_ac.flight} -> Lat: {cur_lat:+.5f}°, Lon: {cur_lon:+.5f}°, Alt: {alt_m:.1f}m, Track: {track:.1f}°")

            time.sleep(interval)

            # Poll for updates every 4 seconds
            if count % int(max(1, 4.0 / interval)) == 0:
                fresh = client.fetch_single_aircraft(selected_ac.hex, args.lat, args.lon)
                if fresh:
                    selected_ac = fresh
    except KeyboardInterrupt:
        print("\n[*] Stopping telemetry streaming.")
    finally:
        bridge.close()


# --- Main CLI Entrypoint ---

def main():
    parser = argparse.ArgumentParser(
        description="ADS-B Closest Aircraft Selector & Orion Simulator Telemetry Bridge",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--lat", type=float, default=34.0522, help="Reference Latitude (default: Los Angeles 34.0522)")
    parser.add_argument("--lon", type=float, default=-118.2437, help="Reference Longitude (default: Los Angeles -118.2437)")
    parser.add_argument("--radius", type=float, default=100.0, help="Search radius in Nautical Miles (default: 100 NM)")
    parser.add_argument("--limit", type=int, default=50, help="Number of closest aircraft to list (default: 50)")
    parser.add_argument("--endpoint", type=str, default=None,
                        help="ADS-B endpoint URL or template (default: 'https://api.adsb.lol/v2/point/{lat}/{lon}/{radius}', with automatic fallbacks)")
    parser.add_argument("--orion-host", type=str, default="127.0.0.1", help="Orion server IP or hostname")
    parser.add_argument("--orion-port", type=int, default=8745, help="Orion server UDP port (default: 8745)")
    parser.add_argument("--rate", type=float, default=2.0, help="Coordinate transmission rate in Hz")
    parser.add_argument("--tilt", type=float, default=-45.0, help="Camera/gimbal tilt angle in degrees (default: -45.0, limits: -80° to +28°)")
    parser.add_argument("--mock", action="store_true", help="Force mock ADS-B traffic generator for offline testing")
    parser.add_argument("--headless", action="store_true", help="Run in non-interactive CLI mode without curses TUI")
    parser.add_argument("--select", type=str, default=None, help="In headless mode, attach to aircraft by callsign, hex, index (1..50), or 'closest'")
    parser.add_argument("--count", type=int, default=0, help="In headless mode with --select, stop after sending N packets (0 = infinite)")

    args = parser.parse_args()

    # If headless is explicitly requested or terminal is not a tty, run headless
    if args.headless or not sys.stdin.isatty():
        run_headless(args)
    else:
        client = ADSBClient(endpoint=args.endpoint, force_mock=args.mock)
        bridge = OrionBridge(host=args.orion_host, port=args.orion_port, tilt_deg=args.tilt)
        tui = ADSBTrackerTUI(
            client=client,
            bridge=bridge,
            center_lat=args.lat,
            center_lon=args.lon,
            radius_nm=args.radius,
            limit=args.limit,
            update_rate_hz=args.rate
        )
        try:
            tui.run()
        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()


if __name__ == "__main__":
    main()
