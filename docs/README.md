# Orion-Shadow Documentation Hub

Welcome to the **Orion-Shadow** documentation hub. Orion-Shadow is a high-fidelity Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) digital twin simulator for Trillium Engineering Orion gimbal and camera systems. It specifically models the **Trillium HD40-XV** electro-optical gimbal payload and implements the **OrionPublic Protocol v1.4.0** (targeting **Orion SDK 3.1.9**).

This documentation suite serves as a comprehensive, modular "one-stop shop" covering every aspect of the software: network protocols, state machines, dynamics, terrain simulation, synthetic video streaming, payload controls, live ADS-B flight tracking, configuration, and SDK integration.

---

## 🗺️ Documentation Map

The documentation is organized into focused, cross-referenced topic guides:

| Document | Description |
| :--- | :--- |
| **[Architecture & Concurrency](architecture.md)** | Internal software structure, `asyncio` concurrency, execution loops, and dataflow pipelines. |
| **[Protocol & Network Specification](protocol_network.md)** | Port assignments (`8745`, `8746`, `8747`), packet framing, Fletcher-251 checksum, and full packet dictionary. |
| **[Physics & State Machine](physics_state_machine.md)** | Trillium HD40-XV mechanical model, operating modes (`RATE`, `POSITION`, `GEOPOINT`), and Euler dynamics. |
| **[Terrain & Geospatial Engine](terrain_geospatial.md)** | DTED parsing (`.dt0`–`.dt1`), WGS84 geodetic transformations, bilinear interpolation, and ray-casting geolocation. |
| **[Synthetic Video & Rendering](video_rendering.md)** | 2D tile visualizer, 3D draped terrain renderer, HUD symbology, FFmpeg multicast, and GPU/NVENC acceleration. |
| **[Camera, Payloads & Tracking](camera_payloads_tracking.md)** | 1x–112x zoom & HFOV math, KTnC camera protocol emulation, video tracking engine, laser subsystem, and fault injection. |
| **[ADS-B Live Flight Bridge](adsb_integration.md)** | Attaching the gimbal to live real-world flights using open ADS-B feeds and an interactive TUI. |
| **[CLI & Configuration Reference](cli_configuration.md)** | Full command-line option reference, configuration parameters, runner scripts, and performance tuning. |
| **[Integration Guide](integration_guide.md)** | Pairing with the Orion SDK (C++ and Python), Wireshark Lua dissector setup, VLC/ffplay video reception, and troubleshooting. |
| **[Testing & Development](testing_development.md)** | Test runner architecture (`scripts/pytest`), unit/integration test suite, mock DTED generation, and contributing guide. |
| **[Technical Reference Card](technical_reference.md)** | Quick-reference cheat sheet for packet structures, port numbers, and common developer workflows. |

---

## 🏗️ High-Level System Overview

The following diagram illustrates how Orion-Shadow bridges external controllers, software applications, video players, and live aircraft feeds:

```mermaid
flowchart TD
    subgraph Clients["Downstream Clients & Operators"]
        SDK["Orion SDK Client (C++ / Python)"]
        WS["Wireshark Packet Dissector"]
        VLC["Video Stream Player (VLC / FFplay / GStreamer)"]
    end

    subgraph OrionShadow["Orion-Shadow Digital Twin Server"]
        Net["Dual Network Server\nUDP :8745 (Cmd/Discovery) | UDP :8746 (Resp)\nTCP :8747 (Persistent Stream)"]
        Proto["Protocol Engine\n(Fletcher-251 Checksum, Big-Endian Framing)"]
        State["Gimbal State Machine\n(Trillium HD40-XV Model)"]
        Physics["Physics Engine\n(Dynamics, Inertia, Damping)"]
        Terrain["Terrain Engine\n(DTED Elevation & Ray Casting)"]
        Video["Multicast Video Server\n(2D/3D Draped Tiles, HUD, NVENC/FFmpeg)"]
        Faults["Fault Engine\n(Overcurrent, Comms, Sensor Dropouts)"]
    end

    subgraph External["External Services & Data"]
        ADSB["ADS-B Feed (adsb.lol / dump1090)"]
        Tiles["Map Tile Server (XYZ / Satellite)"]
        DTED["DTED Elevation Files (.dt0 / .dt1)"]
    end

    SDK <-->|UDP 8745 / TCP 8747| Net
    Net <--> Proto
    Proto <--> State
    State <--> Physics
    State <--> Terrain
    State <--> Faults
    State --> Video
    Video -->|UDP Multicast :5004| VLC
    Net -.->|Packet Mirroring| WS
    ADSB -.->|scripts/adsb_attach.py| Net
    Tiles -.-> Video
    DTED -.-> Terrain
```

---

## ⚡ Quick-Start Summary

### 1. Installation
Clone the repository and install dependencies in editable mode:
```bash
cd /home/user/dev/orion-shadow
pip install -e .
```

### 2. Launch the Simulator
Run the simulator on standard ports with simulated coordinates:
```bash
python3 -m orion_shadow.server --lat 39.6 --lon -116.3 --alt 2500 --heading 90
```

Or execute the pre-configured runner script:
```bash
./scripts/run.sh
```

### 3. Connect Video Stream
Open VLC or ffplay to watch the real-time synthesized camera view and HUD:
```bash
ffplay -fflags nobuffer -flags low_delay udp://239.255.0.1:5004
```

### 4. Connect Your Orion SDK Application
Point your Orion SDK application to `localhost:8747` (TCP) or `localhost:8745` (UDP) to start sending gimbal commands and receiving live telemetry.
