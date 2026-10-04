# OrionShadow Technical Reference Card

This document provides a concise reference card for engineers integrating with the **OrionShadow** simulator or debugging communication with Trillium Orion gimbal systems. For comprehensive topic guides, refer to the **[Documentation Hub](README.md)**.

---

## 🧭 Topic Guides Directory

- 📐 **[Architecture & Concurrency](architecture.md)** — Event loops, async workers, telemetry broadcast pipeline.
- 📡 **[Protocol & Network Specification](protocol_network.md)** — Big-endian framing, Fletcher-251 checksum, complete packet catalog.
- ⚙️ **[Physics & State Machine](physics_state_machine.md)** — HD40-XV hardware specs, modes (RATE/POS/GEOPOINT), Euler dynamics.
- 🏔️ **[Terrain & Geospatial Engine](terrain_geospatial.md)** — DTED parsing, WGS84 geodesy, ray-casting target geolocation.
- 🎥 **[Synthetic Video & Rendering](video_rendering.md)** — 2D tiles, 3D relief draping, HUD overlays, FFmpeg/NVENC multicast.
- 📷 **[Camera, Payloads & Tracking](camera_payloads_tracking.md)** — 1x–112x zoom, KTnC camera protocol, video tracking, laser, faults.
- ✈️ **[ADS-B Live Flight Bridge](adsb_integration.md)** — Real flight tracking, interactive TUI, injecting live aircraft telemetry.
- 💻 **[CLI & Configuration Reference](cli_configuration.md)** — Full command-line option reference, scripts/run.sh, performance tuning.
- 🔌 **[Integration Guide](integration_guide.md)** — C++/Python SDK examples, Wireshark dissector, VLC/ffplay video feeds.
- 🧪 **[Testing & Development](testing_development.md)** — Custom test runner, test structure, mock data generators.

---

## ⚡ Quick Reference: Ports & Network

| Identifier | Port | Protocol | Usage | Reference |
| :--- | :--- | :--- | :--- | :--- |
| `UDP_OUT_PORT` | `8745` | UDP | Inbound command reception & discovery broadcast listening | `OrionComm.h` |
| `UDP_IN_PORT` | `8746` | UDP | Destination port for discovery responses sent to SDK clients | `OrionComm.h` |
| `TCP_PORT` | `8747` | TCP | Persistent bidirectional command and high-rate telemetry session | `OrionComm.h` |
| Video Stream | `5004` | UDP Multicast | H.264 / MPEG-TS multicast video feed (`239.255.0.1:5004`) | `VideoServer` |

---

## 📦 Packet Framing & Checksum

### Binary Layout

| Byte Offset | Field | Value / Type | Notes |
| :--- | :--- | :--- | :--- |
| `0` | `Sync0` | `0xD0` (208) | Standard Orion synchronization byte 0 |
| `1` | `Sync1` | `0x0D` (13) | Standard Orion synchronization byte 1 |
| `2` | `Packet ID` | `uint8` (0–255) | Packet identifier (see Packet Catalog) |
| `3` | `Length` | `uint8` (0–140) | Length of the subsequent payload data |
| `4 .. L+3` | `Payload` | Bytes ($L$) | Big-endian payload data |
| `L+4` | `Fletch0` | `uint8` | Checksum MSB |
| `L+5` | `Fletch1` | `uint8` | Checksum LSB |

### Fletcher-16 Checksum (Modulo-251 Prime)

> [!IMPORTANT]
> The OrionPublic protocol uses a modified 16-bit Fletcher's checksum with **modulo 251** (a prime number), NOT the standard modulo 255. Packets with invalid checksums are dropped immediately.

```python
def compute_fletcher251(payload_including_header: bytes) -> int:
    a, b = 1, 1
    for byte in payload_including_header:
        a = (a + byte) % 251
        b = (b + a) % 251
    return (b << 8) | a
```

---

## 📋 Common Packet IDs

| ID | Name | Direction | Payload Length | Summary |
| :--- | :--- | :--- | :--- | :--- |
| `0x00` | `ORION_PKT_INITIALIZE` | In / Out | 0 bytes | Discovery handshake & client registration |
| `0x01` | `ORION_PKT_CMD` | In | 5–8 bytes | Pan/tilt rate, angle, or mode command |
| `0x02` | `ORION_PKT_VERSION` | Out | 8 bytes | Firmware and protocol version |
| `0x03` | `ORION_PKT_CAMERAS` | In / Out | Variable | Camera optical specs and capabilities |
| `0x04` | `ORION_PKT_CAMERA_CMD` | In | 4 bytes | Zoom and focus control |
| `0x05` | `ORION_PKT_CAMERA_STATE` | Out | 4 bytes | Active camera zoom and focus state |
| `0x07` | `ORION_PKT_KTNC_SETTINGS` | In / Out | 16 bytes | KTnC sensor exposure, sharpness, contrast |
| `0x08` | `ORION_PKT_VIDEO_TRACK_CMD`| In | 10 bytes | Video track mode, target coordinates |
| `0x09` | `ORION_PKT_VIDEO_TRACK_STATE`| Out | 14 bytes | Tracking status, confidence, bounding box |
| `0x0A` | `ORION_PKT_POSITIONS` | Out | 16 bytes | Pan, tilt, roll encoder positions |
| `0x0B` | `ORION_PKT_DIAGNOSTICS` | Out | 18 bytes | Voltages (5V/12V/24V), temps, currents, error bits |
| `0x0C` | `ORION_PKT_GEO_DATA` | Out | 24 bytes | Target lat/lon/alt, slant range, footprint corners |
| `0x10` | `ORION_PKT_LASER_CMD` | In | 2 bytes | Laser rangefinder/pointer enable and power |
| `0x11` | `ORION_PKT_LASER_STATE` | Out | 6 bytes | Laser firing status and power telemetry |
| `0xD1` | `ORION_PKT_GPS_DATA` | In / Out | 28 bytes | Aircraft latitude, longitude, altitude, groundspeed |
| `0xD2` | `ORION_PKT_EXT_HEADING_DATA`| In / Out | 8 bytes | Aircraft true heading and status |

For exhaustive byte offsets and field encoding, see **[Protocol & Network Specification](protocol_network.md)**.

---

## 🛠️ Common Integration Checklist

- [ ] Connect via TCP (`8747`) for stable command/telemetry streaming, or UDP (`8745`) for connectionless datagrams.
- [ ] Send `ORION_PKT_INITIALIZE` (`0x00`) to register client address and request version.
- [ ] Ensure all multi-byte values use **Big-Endian** (network byte order).
- [ ] Implement the Modulo-251 Fletcher checksum on every transmitted packet.
- [ ] Subscribe to the video stream at `udp://@239.255.0.1:5004` (using VLC or ffplay).
