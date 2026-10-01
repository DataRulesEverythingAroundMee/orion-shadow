# Orion Gimbal & Camera Simulator

A high-fidelity Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) simulator for Trillium Engineering Orion gimbaled camera systems. This project is designed to allow software developers to test control algorithms, telemetry processing, and camera management systems without requiring physical gimbal hardware.

## 🚀 Overview

The **OrionShadow** replicates the communication behavior of an Orion gimbal by implementing the `OrionPublic` protocol (targeting Orion SDK 3.1.9). It acts as a UDP server that accepts commands from the Orion SDK and responds with realistic telemetry data, including gimbal positions, camera states, and sensor information.

### Key Features

• **Protocol Compliance**: Full implementation of the OrionPublic protocol (targeting Orion SDK 3.1.9 / Protocol 1.4.0), including big-endian packet framing and modified 16-bit Fletcher checksums.
• **SIL (Software-in-the-Loop)**: Simulate the complete logic of the gimbal's internal state machine, including motor modes, fault states, and command echoing.
• **HIL (Hardware-in-the-Loop)**: Provides a stable network interface for testing physical controllers or embedded systems in a controlled environment.
• **Physics-Driven Dynamics**: Moves via simulated inertia, acceleration, and velocity clamping rather than instant position jumps.
• **Mock Telemetry**: Generates realistic sensor streams (GPS, IMU, Pan/Tilt positions) to stress-test downstream software.
• **Command Echoing**: Implements the gimbal's native behavior of echoing configuration packets to all connected clients.
• **Camera Modeling**: Simulates camera zoom, focus, readiness, and video/tracking configuration.
• **Fault Simulation**: Injects realistic hardware faults (e.g., sensor timeouts, motor overcurrent) to test control loop resilience.
• **Lifecycle & Diagnostics**: Supports full gimbal lifecycle management (Reset, Startup) and provides real-time diagnostic telemetry.

---

## 🛠 Architecture

The simulator is built as a modular Python application composed of several key layers:

### 1. Protocol Layer (`orion_shadow.core.protocol`)
This layer handles the low-level packet construction and parsing. 
• **Framing**: Implements the `0xD0 0x0D` sync header.
• **Checksum**: Calculates and verifies the modified 16-bit Fletcher's checksum.
• **Endianness**: Manages big-endian byte order for all multi-byte fields.

### 2. Physics Engine (`orion_shadow.engine.physics`)
Simulates the physical movement of the gimbal axes.
• **Integration**: Uses a $dt$-driven update loop to integrate acceleration into velocity and position.
• **Dynamics**: Models inertia, velocity limits, and acceleration limits to provide a realistic response to commands.
• **Damping**: Simulates mechanical friction to prevent infinite oscillation.

### 3. Terrain Engine (`orion_shadow.engine.terrain`)
Provides terrain-awareness for altitude-critical simulations.
• **Optional Activation**: Enabled only when a DTED (Digital Terrain Elevation Data) directory is provided via the CLI.
• **Elevation Lookup**: Uses DTED data to provide realistic ground elevation at the current GPS coordinates.
• **Altitude Integration**: Automatically updates the gimbal's reported altitude based on terrain elevation when active.

### 4. State Machine (`orion_shadow.core.state`)
Simulates the internal logic of the Orion Crown board.
• **Modes**: Manages transitions between `DISABLED`, `FAULT`, `STABILIZING`, and `TRACKING`.
• **Commands**: Interprets `ORION_PKT_CMD` to update target pan/tilt angles.
• **Telemetry**: Tracks the "current" physical position of the gimbal, which is updated based on physics integration.

### 5. Communication Layer (`orion_shadow.server`)
An asynchronous dual UDP and TCP server adhering to the standard ports defined in Orion SDK (`OrionComm.h`):
• **`UDP_OUT_PORT` (8745)**: Listens for incoming discovery broadcasts and datagram commands.
• **`UDP_IN_PORT` (8746)**: Destination port for discovery responses sent back to SDK clients.
• **`TCP_PORT` (8747)**: Persistent TCP server for command streams and bidirectional telemetry.
• **Concurrency & Multiplexing**: Broadcasts periodic telemetry updates across all connected UDP and TCP clients.

### 6. Telemetry Engine (`orion_shadow.telemetry`)
A periodic task that generates and pushes state packets.
• **Dynamic Data**: Provides varying GPS, IMU, and gimbal position data.
• **Customization**: Allows users to define "nominal" vs "faulty" telemetry profiles.

---

## 📦 Packet Specification

The simulator adheres to the `OrionPublicProtocol` version `1.4.0` (targeting `orion-sdk` version `3.1.9`).

### General Packet Format

| Byte         | Name    | Value                                                  |
|--------------|---------|--------------------------------------------------------|
| 0            | Sync0   | `0xD0`                                                   |
| 1            | Sync1   | `0x0D`                                                   |
| 2            | ID      | Packet identifier (0-255)                              |
| 3            | Length  | Data payload length (0-140 bytes)                     |
| 4...L+3     | Data    | Payload data                                           |
| L+4         | Fletch0 | Checksum (MSB)                                         |
| L+5         | Fletch1 | Checksum (LSB)                                         |

---

## 🚀 Quick-Start Guide

### 1. Prerequisites
• Python 3.9+
• `pip` or `uv`

### 2. Installation
Clone the repository and install the package in editable mode:

```bash
git clone https://github.com/DataRulesEverythingAroundMee/orion-shadow.git
cd orion-shadow
pip install -e .
```

### 3. Running the Simulator
Start the simulator. It binds to the standard Orion SDK ports (`8745` UDP, `8747` TCP). You can adjust the physics timestep (`--dt`) to increase or decrease simulation fidelity.

To enable terrain-aware simulation, provide the path to your DTED data using the `--dted-path` flag.

```bash
# Standard mode (10Hz) on default ports (UDP 8745, TCP 8747)
python -m orion_shadow.server --host 0.0.0.0

# High-fidelity mode (100Hz)
python -m orion_shadow.server --host 0.0.0.0 --dt 0.01

# Terrain-aware mode
python -m orion_shadow.server --host 0.0.0.0 --dted-path /path/to/dted/folder
```

**Command Line Options:**
• `--host`: Interface to bind to (default: `0.0.0.0`).
• `--port` / `--udp-port`: UDP port for discovery and commands (default: `8745`).
• `--udp-in-port`: UDP port for discovery responses (default: `8746`).
• `--tcp-port`: TCP port for persistent communication (default: `8747`).
• `--dt`: Physics/Telemetry update interval in seconds (default: `0.1`).
• `--dted-path`: Path to DTED folder for terrain simulation (optional).

### 4. Integrating Your Software
To use the simulator with your existing Orion SDK software, simply connect to the simulator's IP on the standard ports.

**TCP Connection (SDK default persistent connection):**
```python
gimbal_client.connect(ip="localhost", port=8747)
```

**UDP Connection (Discovery / Datagram):**
```python
gimbal_client.connect(ip="localhost", port=8745)
```

---

## 🧪 Testing
The simulator includes a comprehensive test suite to ensure protocol compliance, physics accuracy, and state machine reliability.

**Run all tests:**
```bash
pytest
```

**Run specific protocol tests:**
```bash
pytest tests/test_simulator.py
```

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
