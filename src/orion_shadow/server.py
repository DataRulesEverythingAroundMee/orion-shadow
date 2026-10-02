import asyncio
import argparse
import logging
import struct
import math
from typing import List, Optional, Set, Tuple
from orion_shadow import setup_logging
from orion_shadow.core.engine import ProtocolEngine
from orion_shadow.core.protocol import OrionPacket, OrionPktType, UDP_OUT_PORT, UDP_IN_PORT, TCP_PORT
from orion_shadow.core.state import GimbalState
from orion_shadow.engine.terrain import TerrainEngine
from orion_shadow.engine.video_server import VideoServer

logger = logging.getLogger(__name__)

class OrionUDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, server: 'OrionServer'):
        self.server = server
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport
        self.server.transport = transport

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        self.server.handle_datagram(data, addr)

    def error_received(self, exc: Exception):
        logger.error("[UDP] Socket error received: %s", exc)

    def connection_lost(self, exc: Optional[Exception]):
        if exc:
            logger.error("[UDP] Socket connection lost: %s", exc)


class OrionServer:
    def __init__(self, host: str = '0.0.0.0', port: int = UDP_OUT_PORT, 
                 udp_in_port: int = UDP_IN_PORT, tcp_port: Optional[int] = TCP_PORT,
                 dt: float = 0.1, dted_path: Optional[str] = None, 
                 tile_url: Optional[str] = None, video_port: int = 5004,
                 multicast_group: str = '239.255.0.1', video_enabled: bool = True,
                 log_level: str = 'warning', video_fps: int = 10,
                 video_width: int = 1280, video_height: int = 720,
                 max_tile_zoom: int = 17, tile_zoom: Optional[int] = None,
                 prefetch: bool = True, prefetch_distance: float = 3000.0,
                 lat: float = 0.0, lon: float = 0.0, alt: float = 0.0,
                 pan: float = 0.0, tilt: Optional[float] = None, heading: float = 0.0,
                 speed: float = 0.0, tile_cache_dir: Optional[str] = "cache/tiles",
                 gpu_encoding: Optional[bool] = None, drape_subsample: int = 6,
                 distance_lod: bool = True):
        self.host = host
        self.port = port
        self.udp_port = port
        self.udp_in_port = udp_in_port
        self.tcp_port = tcp_port
        self.dt = dt
        self.initial_lat = lat
        self.initial_lon = lon
        self.initial_alt = alt
        self.initial_pan = pan
        self.initial_tilt = tilt
        self.initial_heading = heading
        self.initial_speed = speed
        self.terrain = TerrainEngine(dted_path)
        self.state = GimbalState(
            dt, 
            self.terrain,
            lat=lat,
            lon=lon,
            alt=alt,
            pan=pan,
            tilt=tilt,
            heading=heading,
            speed=speed
        )
        self.engine = ProtocolEngine()
        self.clients: Set[Tuple[str, int]] = set()  # UDP clients
        self.tcp_clients: Set[asyncio.StreamWriter] = set()  # TCP clients
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.tcp_server: Optional[asyncio.AbstractServer] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._tasks: List[asyncio.Task] = []
        self._running: bool = False

        # Logger setup
        setup_logging(log_level)
        self.logger = logging.getLogger("orion_shadow.server")
        
        # Multicast Video Server
        self.video_server: Optional[VideoServer] = None
        if video_enabled:
            self.video_server = VideoServer(
                self.state, 
                tile_url_template=tile_url, 
                terrain_engine=self.terrain,
                multicast_group=multicast_group,
                port=video_port,
                host=host,
                fps=video_fps,
                width=video_width,
                height=video_height,
                max_tile_zoom=max_tile_zoom,
                tile_zoom=tile_zoom,
                prefetch_enabled=prefetch,
                prefetch_distance=prefetch_distance,
                tile_cache_dir=tile_cache_dir,
                gpu_encoding=gpu_encoding,
                drape_subsample=drape_subsample,
                distance_lod=distance_lod
            )


    def _format_packet_details(self, packet: OrionPacket) -> str:
        pkt_name = self.engine.packet_id_map.get(packet.packet_id, f"PKT_0x{packet.packet_id:02X}")
        details = []
        if packet.packet_id == OrionPktType.CMD:
            if len(packet.data) == 8:
                pan, tilt = struct.unpack(">ff", packet.data[:8])
                details.append(f"pan={pan:.2f}, tilt={tilt:.2f}")
            elif len(packet.data) >= 4:
                pan_raw, tilt_raw = struct.unpack_from(">hh", packet.data, 0)
                if len(packet.data) >= 5:
                    mode = packet.data[4]
                    if mode in (0x10, 0x11, 0x30):
                        details.append(f"mode=0x{mode:02X} (RATE), pan_rate={math.degrees(pan_raw/1000.0):.2f} deg/s, tilt_rate={math.degrees(tilt_raw/1000.0):.2f} deg/s")
                    else:
                        details.append(f"mode=0x{mode:02X}, pan={math.degrees(pan_raw/1000.0):.2f}, tilt={math.degrees(tilt_raw/1000.0):.2f}")
                else:
                    details.append(f"pan={math.degrees(pan_raw/1000.0):.2f}, tilt={math.degrees(tilt_raw/1000.0):.2f}")
        elif packet.packet_id == OrionPktType.CAMERAS:
            if len(packet.data) <= 4:
                details.append("request camera settings")
            else:
                details.append(f"camera settings payload ({len(packet.data)} bytes)")
        elif packet.packet_id == OrionPktType.KTNC_SETTINGS:
            if len(packet.data) <= 1:
                details.append("request KTnC settings")
            else:
                details.append(f"KTnC settings payload ({len(packet.data)} bytes)")
        elif packet.packet_id == OrionPktType.INITIALIZE:
            details.append("initialize / discovery request")
        elif packet.packet_id == OrionPktType.RESET:
            details.append("reset request")
        elif packet.packet_id == OrionPktType.STARTUP_CMD:
            details.append("startup command")
        elif packet.packet_id == OrionPktType.CAMERA_SWITCH:
            if len(packet.data) >= 1:
                details.append(f"switch to camera_index={packet.data[0]}")
        elif packet.packet_id == OrionPktType.CAMERA_CMD or packet.packet_id == OrionPktType.CAMERA_STATE:
            if len(packet.data) == 0:
                details.append("request camera state")
            elif len(packet.data) >= 2 and len(packet.data) < 8:
                zoom_raw = struct.unpack_from(">h", packet.data, 0)[0]
                if len(packet.data) >= 4:
                    focus_raw = struct.unpack_from(">h", packet.data, 2)[0]
                    details.append(f"zoom={zoom_raw / 100.0:.2f}x, focus={focus_raw / 10000.0:.2f}")
                else:
                    details.append(f"zoom={zoom_raw / 100.0:.2f}x")
            elif len(packet.data) >= 8:
                zoom, focus = struct.unpack(">ff", packet.data[:8])
                details.append(f"zoom={zoom:.2f}x, focus={focus:.2f}")
        elif packet.packet_id == OrionPktType.NETWORK_VIDEO:
            if len(packet.data) == 0:
                details.append("request network video settings")
            else:
                details.append(f"network video settings ({len(packet.data)} bytes)")
        elif packet.packet_id == OrionPktType.LASER_STATES:
            if len(packet.data) == 0:
                details.append("request laser states")
        elif packet.packet_id == OrionPktType.VIDEO_OPTIONS:
            if len(packet.data) == 0:
                details.append("request video options")
        elif packet.packet_id == OrionPktType.TRACK_OPTIONS:
            if len(packet.data) == 0:
                details.append("request track options")
        elif packet.packet_id == OrionPktType.LASER_CMD:
            if len(packet.data) >= 4:
                power = struct.unpack(">f", packet.data[:4])[0]
                details.append(f"laser_power={power:.2f}")
        elif packet.packet_id == OrionPktType.GPS_DATA:
            if len(packet.data) >= 28:
                raw_lat, raw_lon, raw_alt = struct.unpack_from(">iii", packet.data, 4)
                vn, ve, vd = struct.unpack_from(">iii", packet.data, 16)
                lat, lon, alt = raw_lat * 1e-7, raw_lon * 1e-7, raw_alt / 10000.0
                spd_kts = math.hypot(vn / 1000.0, ve / 1000.0) * 1.94384
                details.append(f"lat={lat:.5f}, lon={lon:.5f}, alt={alt:.1f}m, speed={spd_kts:.1f}kts")
            elif len(packet.data) >= 16:
                f_lat, f_lon, f_alt, f_spd = struct.unpack_from(">ffff", packet.data, 0)
                if -90.0 <= f_lat <= 90.0 and -180.0 <= f_lon <= 180.0 and (abs(f_lat) > 0.001 or abs(f_lon) > 0.001):
                    details.append(f"lat={f_lat:.5f}, lon={f_lon:.5f}, alt={f_alt:.1f}m, speed={f_spd:.1f}kts")
                else:
                    raw_lat, raw_lon, raw_alt = struct.unpack_from(">iii", packet.data, 4)
                    lat, lon, alt = raw_lat * 1e-7, raw_lon * 1e-7, raw_alt / 10000.0
                    details.append(f"lat={lat:.5f}, lon={lon:.5f}, alt={alt:.1f}m")
            elif len(packet.data) >= 12:
                lat, lon, alt = struct.unpack(">fff", packet.data[:12])
                details.append(f"lat={lat:.5f}, lon={lon:.5f}, alt={alt:.1f}m")
        elif packet.packet_id == OrionPktType.EXT_HEADING_DATA:
            if len(packet.data) >= 8 and len(packet.data) < 12:
                raw_hdg = struct.unpack_from(">h", packet.data, 0)[0]
                raw_pitch = struct.unpack_from(">h", packet.data, 6)[0]
                heading = math.degrees(raw_hdg / 10430.06004058) % 360.0
                pitch = math.degrees(raw_pitch / 10430.06004058)
                details.append(f"heading={heading:.1f}, pitch={pitch:.1f}")
            elif len(packet.data) >= 12:
                heading, roll, pitch = struct.unpack(">fff", packet.data[:12])
                details.append(f"heading={heading:.1f}, roll={roll:.1f}, pitch={pitch:.1f}")
        elif packet.packet_id == OrionPktType.GEOLOCATE_TELEMETRY_CORE:
            details.append("geolocate telemetry core")
        elif packet.packet_id == OrionPktType.GEOPOINT_CMD:
            if len(packet.data) >= 18:
                raw_lat, raw_lon, raw_alt = struct.unpack_from(">iii", packet.data, 0)
                details.append(f"target_lat={raw_lat*1e-7:.5f}, target_lon={raw_lon*1e-7:.5f}, target_alt={raw_alt/10000.0:.1f}m, mode=ORION_MODE_GEOPOINT (0x60)")
            elif len(packet.data) >= 12:
                lat, lon, alt = struct.unpack(">fff", packet.data[:12])
                details.append(f"target_lat={lat:.5f}, target_lon={lon:.5f}, target_alt={alt:.1f}m, mode=ORION_MODE_GEOPOINT (0x60)")
            else:
                details.append("mode=ORION_MODE_GEOPOINT (0x60)")

        detail_str = f" ({', '.join(details)})" if details else ""
        return f"{pkt_name} [0x{packet.packet_id:02X}, len={len(packet.data)}]{detail_str}"

    def handle_datagram(self, data: bytes, addr: Tuple[str, int]):
        if addr not in self.clients:
            self.logger.info("[UDP] Registered new client connection from %s:%d", addr[0], addr[1])
            self.clients.add(addr)

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("[UDP] Received datagram of %d bytes from %s:%d", len(data), addr[0], addr[1])
        
        offset = 0
        while offset + 6 <= len(data):
            if data[offset] == 0xD0 and data[offset + 1] == 0x0D:
                p_id = data[offset + 2]
                length = data[offset + 3]
                pkt_len = 4 + length + 2
                if offset + pkt_len <= len(data):
                    full_packet = data[offset:offset + pkt_len]
                    try:
                        packet = self.engine.parse(full_packet)
                        if self.logger.isEnabledFor(logging.INFO):
                            self.logger.info("[UDP] Command/Request from %s: %s", addr, self._format_packet_details(packet))
                        response_pkt = None
                        if packet:
                            response_pkt = self.state.update_from_command(packet)
                        
                        if response_pkt:
                            echo_data = response_pkt
                        elif len(packet.data) > 0 or packet.packet_id == OrionPktType.INITIALIZE:
                            echo_data = full_packet
                        else:
                            echo_data = None
                        
                        # Echo behavior
                        if echo_data and self.transport and not self.transport.is_closing():
                            self.transport.sendto(echo_data, addr)
                            if self.logger.isEnabledFor(logging.DEBUG):
                                self.logger.debug("[UDP] Sent echo/response (%d bytes) to %s", len(echo_data), addr)
                            # Also respond on UDP_IN_PORT (8746) if different, for clients expecting discovery reply on UDP_IN_PORT
                            if self.udp_in_port and addr[1] != self.udp_in_port:
                                try:
                                    self.transport.sendto(echo_data, (addr[0], self.udp_in_port))
                                except Exception as exc:
                                    self.logger.warning("[UDP] Failed to send secondary response to port %d: %s", self.udp_in_port, exc)
                    except Exception as e:
                        self.logger.error("[UDP] Error processing packet from %s: %s", addr, e)
                    offset += pkt_len
                else:
                    if self.logger.isEnabledFor(logging.DEBUG):
                        self.logger.debug("[UDP] Fragmented datagram from %s (offset=%d, needed=%d, got=%d)", addr, offset, pkt_len, len(data))
                    break
            else:
                offset += 1

    async def handle_tcp_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        addr = writer.get_extra_info('peername')
        self.logger.info("[TCP] New client connection accepted from %s", addr)
        self.tcp_clients.add(writer)
        buffer = bytearray()
        try:
            while self._running:
                chunk = await reader.read(1024)
                if not chunk:
                    break
                buffer.extend(chunk)
                if self.logger.isEnabledFor(logging.DEBUG):
                    self.logger.debug("[TCP] Received %d bytes from %s (buffer size: %d)", len(chunk), addr, len(buffer))
                while len(buffer) >= 6:
                    if buffer[0] == 0xD0 and buffer[1] == 0x0D:
                        p_id = buffer[2]
                        length = buffer[3]
                        pkt_len = 4 + length + 2
                        if len(buffer) >= pkt_len:
                            full_packet = bytes(buffer[:pkt_len])
                            del buffer[:pkt_len]
                            try:
                                packet = self.engine.parse(full_packet)
                                if self.logger.isEnabledFor(logging.INFO):
                                    self.logger.info("[TCP] Command/Request from %s: %s", addr, self._format_packet_details(packet))
                                response_pkt = None
                                if packet:
                                    response_pkt = self.state.update_from_command(packet)
                                if response_pkt:
                                    echo_data = response_pkt
                                elif len(packet.data) > 0 or packet.packet_id == OrionPktType.INITIALIZE:
                                    echo_data = full_packet
                                else:
                                    echo_data = None
                            except Exception as e:
                                self.logger.error("[TCP] Error processing packet from %s: %s", addr, e)
                                continue

                            if writer.is_closing():
                                break
                            if echo_data:
                                writer.write(echo_data)
                                await writer.drain()
                                if self.logger.isEnabledFor(logging.DEBUG):
                                    self.logger.debug("[TCP] Sent %d bytes to %s", len(echo_data), addr)
                        else:
                            break
                    else:
                        del buffer[0]
        except (asyncio.CancelledError, ConnectionError, BrokenPipeError, OSError):
            pass
        except Exception as e:
            self.logger.error("[TCP] Client %s error: %s", addr, e)
        finally:
            self.logger.info("[TCP] Closing connection from %s", addr)
            self.tcp_clients.discard(writer)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """TCP handler delegate."""
        await self.handle_tcp_client(reader, writer)

    async def simulation_loop(self):
        """The 'Heartbeat' of the simulator: physics and telemetry."""
        while self._running:
            await asyncio.sleep(self.dt)
            if not self._running:
                break
            
            # 1. Advance Physics
            self.state.step()
            
            # 2. Broadcast Telemetry to all clients
            slow_generators = [
                self.state.get_laser_state_packet,
                self.state.get_diagnostics_packet,
                self.state.get_video_options_packet,
                self.state.get_tracking_options_packet,
                self.state.get_cameras_packet,
                self.state.get_faults_packet,
            ]
            self._telemetry_tick = getattr(self, "_telemetry_tick", 0) + 1
            slow_packet = slow_generators[self._telemetry_tick % len(slow_generators)]()
            packets = [
                self.state.get_geolocate_telemetry_core_packet(),
                self.state.get_telemetry_packet(),
                self.state.get_camera_state_packet(),
                slow_packet,
            ]
                        
            # Broadcast to UDP clients
            transport = self.transport
            if self.clients and transport and not transport.is_closing():
                for addr in list(self.clients):
                    if not self._running or not self.transport or transport.is_closing():
                        break
                    try:
                        for p in packets:
                            transport.sendto(p, addr)
                    except Exception as e:
                        if not self._running:
                            break
                        self.logger.error("[UDP] Error sending telemetry to %s: %s", addr, e)
                        if addr in self.clients:
                            self.clients.remove(addr)

            # Broadcast to TCP clients
            if self.tcp_clients:
                for writer in list(self.tcp_clients):
                    if not self._running:
                        break
                    if writer.is_closing():
                        self.tcp_clients.discard(writer)
                        continue
                    try:
                        for p in packets:
                            writer.write(p)
                        await writer.drain()
                    except Exception as e:
                        self.logger.warning("[TCP] Error sending telemetry to client, discarding: %s", e)
                        self.tcp_clients.discard(writer)

    def close(self):
        self._running = False
        if self.tcp_server:
            try:
                self.tcp_server.close()
            except Exception:
                pass
            self.tcp_server = None
        if self.transport:
            try:
                self.transport.close()
            except Exception:
                pass
            self.transport = None
        for writer in list(self.tcp_clients):
            try:
                writer.close()
            except Exception:
                pass
        self.tcp_clients.clear()
        if self.loop and self.loop.is_running():
            for t in list(self._tasks):
                self.loop.call_soon_threadsafe(t.cancel)

    async def run(self):
        self._running = True
        self.loop = asyncio.get_running_loop()
        try:
            transport, protocol = await self.loop.create_datagram_endpoint(
                lambda: OrionUDPProtocol(self),
                local_addr=(self.host, self.udp_port),
                reuse_port=True
            )
        except (TypeError, OSError):
            transport, protocol = await self.loop.create_datagram_endpoint(
                lambda: OrionUDPProtocol(self),
                local_addr=(self.host, self.udp_port)
            )
        self.transport = transport
        self.logger.info("[+] Orion UDP Server active on %s:%d (responses to %s)", self.host, self.udp_port, self.udp_in_port)

        if self.tcp_port is not None:
            try:
                self.tcp_server = await asyncio.start_server(
                    self.handle_tcp_client,
                    self.host,
                    self.tcp_port,
                    reuse_address=True
                )
                self.logger.info("[+] Orion TCP Server active on %s:%d", self.host, self.tcp_port)
            except Exception as e:
                self.logger.warning("[!] Warning: Could not bind TCP server to %s:%d: %s", self.host, self.tcp_port, e)

        self.logger.info("[+] Physics Rate: %.1fHz | Telemetry Rate: %.1fHz", 1/self.dt, 1/self.dt)
        
        self._tasks = [asyncio.create_task(self.simulation_loop())]

        if self.tcp_server:
            self._tasks.append(asyncio.create_task(self.tcp_server.serve_forever()))
        
        if self.video_server:
            if self.video_server.is_3d_terrain_active:
                self.logger.info("[+] 3D Draped Terrain Visualization active (DTED + Map Tiles)")
            elif self.video_server.visualizer:
                self.logger.info("[*] Flat 2D Map Visualization active (tile-url only)")
            self.logger.info("[+] Video Stream (Multicast) active on udp://@%s:%d", self.video_server.multicast_group, self.video_server.port)
            self._tasks.append(asyncio.create_task(self.video_server.start()))
        
        try:
            await asyncio.gather(*self._tasks)
        except (asyncio.CancelledError, KeyboardInterrupt):
            pass
        finally:
            self._running = False
            for t in self._tasks:
                if not t.done():
                    t.cancel()
            await asyncio.gather(*self._tasks, return_exceptions=True)
            if self.tcp_server:
                self.tcp_server.close()
                await self.tcp_server.wait_closed()
                self.tcp_server = None
            for writer in list(self.tcp_clients):
                try:
                    writer.close()
                except Exception:
                    pass
            self.tcp_clients.clear()
            if self.transport:
                self.transport.close()
                self.transport = None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OrionShadow SIL/HIL Gimbal Simulator")
    parser.add_argument("--host", default="0.0.0.0", help="Interface to bind to (default: 0.0.0.0)")
    parser.add_argument("--port", "--udp-port", type=int, default=UDP_OUT_PORT, dest="port", help=f"UDP port for commands/discovery (default: {UDP_OUT_PORT})")
    parser.add_argument("--udp-in-port", type=int, default=UDP_IN_PORT, help=f"UDP port for discovery responses (default: {UDP_IN_PORT})")
    parser.add_argument("--tcp-port", type=int, default=TCP_PORT, help=f"TCP port for persistent communication (default: {TCP_PORT})")
    parser.add_argument("--dt", type=float, default=0.1, help="Physics/telemetry update interval in seconds (default: 0.1)")
    parser.add_argument("--dted-path", type=str, default=None, help="Path to DTED folder or file for terrain simulation")
    parser.add_argument("--tile-url", type=str, default=None, help="XYZ tile URL template (e.g. 'https://{z}/{x}/{y}.png')")
    parser.add_argument("--multicast-group", type=str, default="239.255.0.1", help="Multicast IP address for video stream (default: 239.255.0.1)")
    parser.add_argument("--video-port", type=int, default=5004, help="Multicast UDP port for video stream (default: 5004)")
    parser.add_argument("--fps", "--video-fps", type=int, default=10, dest="fps", help="Video stream framerate in FPS (default: 10)")
    parser.add_argument("--video-width", type=int, default=1280, help="Video stream width in pixels (default: 1280, e.g. 1280 for 720p HD)")
    parser.add_argument("--video-height", type=int, default=720, help="Video stream height in pixels (default: 720, e.g. 720 for 720p HD)")
    parser.add_argument("--tile-zoom", "--zoom", type=int, default=None, dest="tile_zoom", help="Fixed XYZ tile zoom level (e.g. 14..19, default: adaptive)")
    parser.add_argument("--max-tile-zoom", type=int, default=17, help="Maximum tile zoom level for adaptive resolution (default: 17)")
    parser.add_argument("--no-prefetch", action="store_false", dest="prefetch", help="Disable lookahead tile prefetching ahead of aircraft")
    parser.add_argument("--prefetch-distance", type=float, default=10000.0, help="Lookahead distance in meters for prefetching tiles ahead of aircraft (default: 10000.0m)")
    parser.add_argument("--no-video", action="store_true", help="Disable multicast video streaming")
    parser.add_argument("--lat", "--latitude", type=float, default=0.0, dest="lat", help="Initial camera/aircraft latitude in degrees (default: 0.0)")
    parser.add_argument("--lon", "--longitude", type=float, default=0.0, dest="lon", help="Initial camera/aircraft longitude in degrees (default: 0.0)")
    parser.add_argument("--alt", "--altitude", type=float, default=1000.0, dest="alt", help="Initial camera/aircraft altitude in meters MSL (default: 1000.0)")
    parser.add_argument("--pan", type=float, default=0.0, help="Initial gimbal pan in degrees (default: 0.0)")
    parser.add_argument("--tilt", type=float, default=None, help="Initial gimbal tilt in degrees (default: -20.0 if alt/lat/lon set, else 0.0)")
    parser.add_argument("--heading", type=float, default=0.0, help="Initial aircraft heading in degrees (default: 0.0)")
    parser.add_argument("--speed", type=float, default=0.0, help="Initial aircraft speed in knots (default: 0.0)")
    parser.add_argument("--logger", "--log-level", default="warning", dest="log_level",
                        choices=["debug", "info", "warning", "error", "critical"],
                        type=str.lower,
                        help="Logging level for orion-shadow (e.g. info, debug, warning, error. Default: warning)")
    parser.add_argument("--tile-cache-dir", type=str, default="cache/tiles", help="Local directory path to cache downloaded map tiles (default: cache/tiles)")
    parser.add_argument("--gpu-encoding", dest="gpu_encoding", action="store_true", default=None, help="Enable hardware GPU video encoding (NVIDIA NVENC)")
    parser.add_argument("--no-gpu-encoding", dest="gpu_encoding", action="store_false", help="Disable hardware GPU video encoding, force CPU libx264")
    parser.add_argument("--drape-subsample", type=int, default=6, help="Screen ray marching subsampling factor for 3D terrain (default: 6)")
    parser.add_argument("--distance-lod", dest="distance_lod", action="store_true", default=True, help="Enable distance-dependent tile Level of Detail (LOD, lower zoom for distant tiles)")
    parser.add_argument("--no-distance-lod", dest="distance_lod", action="store_false", help="Disable distance-dependent tile Level of Detail (force uniform zoom)")
    args = parser.parse_args()

    server = OrionServer(
        host=args.host, 
        port=args.port, 
        udp_in_port=args.udp_in_port,
        tcp_port=args.tcp_port,
        dt=args.dt, 
        dted_path=args.dted_path,
        tile_url=args.tile_url,
        video_port=args.video_port,
        multicast_group=args.multicast_group,
        video_enabled=not args.no_video,
        log_level=args.log_level,
        video_fps=args.fps,
        video_width=args.video_width,
        video_height=args.video_height,
        max_tile_zoom=args.max_tile_zoom,
        tile_zoom=args.tile_zoom,
        prefetch=args.prefetch,
        prefetch_distance=args.prefetch_distance,
        lat=args.lat,
        lon=args.lon,
        alt=args.alt,
        pan=args.pan,
        tilt=args.tilt,
        heading=args.heading,
        speed=args.speed,
        tile_cache_dir=args.tile_cache_dir,
        gpu_encoding=args.gpu_encoding,
        drape_subsample=args.drape_subsample,
        distance_lod=args.distance_lod
    )

    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        server.logger.info("Simulator shut down.")
