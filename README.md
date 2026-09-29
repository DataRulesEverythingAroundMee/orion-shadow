
# Orion Gimbal & Camera Simulator

A high-fidelity Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) simulator for Trillium Engineering Orion gimbaled camera systems. This project is designed to allow software developers to test control algorithms, telemetry processing, and camera management systems without requiring physical gimbal hardware.

## 🚀 Overview

The **OrionShadow** replicates the communication behavior of an Orion gimbal by implementing the `OrionPublic` protocol. It acts as a TCP/IP server that accepts commands from the Orion SDK and responds with realistic telemetry data, including gimbal positions, camera states, and sensor information.

### Key Features

• **Protocol Compliance**: Full implementation of the OrionPublic protocol, including big-endian packet framing and modified 16-bit Fletcher checksums.
• **SIL (Software-in-the-Loop)**: Simulate the complete logic of the gimbal's internal state machine, including motor modes, fault states, and command echoing.
• **HIL (Hardware-in-the-Loop)**: Provides a stable network interface for testing physical controllers or embedded systems in a controlled environment.
• **Physics-Driven Dynamics**: Moves via simulated inertia, acceleration, and velocity clamping rather than instant position jumps.
• **Mock Telemetry**: Generates realistic sensor streams (GPS, IMU, Pan/Tilt positions) to stress-test downstream software.
• **Command Echoing**: Implements the gimbal's native behavior of echoing configuration packets to all connected clients.

---

## 🛠 Architecture

The simulator is built as a modular Python application composed of several key layers:

### 1. Protocol Layer (`orion_shadow.protocol`)
This layer handles the low-level packet construction and parsing. 
|- **Framing**: Implements the `0xD0 0x0D` sync header.
|- **Checksum**: Calculates and verifies the modified 16-bit Fletcher's checksum (modulo 251).
|- **Endianness**: Manages big-endian byte order for all multi-byte fields.

### 2. Physics Engine (`orion_shadow.physics`)
Simulates the physical movement of the gimbal axes.
|- **Integration**: Uses a $dt$-driven update loop to integrate acceleration into velocity and position.
|- **Dynamics**: Models inertia, velocity limits, and acceleration limits to provide a realistic response to commands.
|- **Damping**: Simulates mechanical friction to prevent infinite oscillation.

### 3. State Machine (`orion_shadow.state`)
Simulates the internal logic of the Orion Crown board.
|- **Modes**: Manages transitions between `DISABLED`, `FAULT`, `STABILIZING`, and `TRACKING`.
|- **Commands**: Interprets `ORION_PKT_CMD` to update target pan/tilt angles.
|- **Telemetry**: Tracks the "current" physical position of the gimbal, which is updated based on physics integration.

### 4. Communication Layer (`orion_shadow.server`)
An asynchronous TCP/IP server that maintains connections with multiple SDK clients.
|- **Concurrency**: Uses `asyncio` to handle multiple clients simultaneously.
|- **Multiplexing**: Broadcasts telemetry updates to all connected clients to mimic real hardware behavior.

### 5. Telemetry Engine (`orion_shadow.telemetry`)
A periodic task that generates and pushes state packets.
|- **Dynamic Data**: Provides varying GPS, IMU, and gimbal position data.
|- **Customization**: Allows users to define "nominal" vs "faulty" telemetry profiles.

---

## 📦 Packet Specification

The simulator adheres to the `OrionPublicProtocol` version `1.3.0.a`.

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
git clone https://github.com/DataRulesEverythingAroundMee/orion-shadow.git\ncd orion-shadow\npip install -e .
```

### 3. Running the Simulator
Start the simulator as a TCP server. You can adjust the physics timestep (`--dt`) to increase or decrease simulation fidelity.

```bash
# Standard mode (10Hz)
python -m orion_shadow.server --host 0.0.0.0 --port 5000

# High-fidelity mode (100Hz)
python -m orion_shadow.server --host 0.0.0.0 --port 5000 --dt 0.01
```

**Command Line Options:**
• `--host`: Interface to bind to (default: `0.0.0.0`).
• `--port`: Port to listen on (default: `5000`).
• `--dt`: Physics/Telemetry update interval in seconds (default: `0.1`).

### 4. Integrating Your Software
To use the simulator with your existing Orion SDK software, simply change your connection settings to point to the simulator's IP.

**Before (Physical Hardware):**
`gimbal_client.connect(ip="192.168.1.100", port=5000)`

**After (Simulator):**
`gimbal_client.connect(ip="localhost", port=5000)`

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
