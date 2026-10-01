# OrionShadow Documentation

This document provides technical details for engineers integrating with the OrionShadow simulator or developing software for the original Orion gimbal systems.

## 🛠 System Overview

OrionShadow is a digital twin of the Trillium Engineering Orion gimbal. It provides a high-fidelity simulation environment for Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) testing.

### Key Simulation Parameters
- **Protocol**: `OrionPublic` (v1.3.0.a)
- **Communication**: UDP (simulated Ethernet/Serial)
- **Physics Integration**: $dt$-based (default 0.1s)
- **Endianness**: Big-endian (Network Byte Order)

---

## 📡 Protocol Implementation Details

### Packet Framing
Every packet sent by the simulator or received by your software MUST follow the OrionPublic framing standard:

| Byte         | Name    | Value                                                  |
|--------------|---------|--------------------------------------------------------|
| 0            | Sync0   | `0xD0`                                                   |
| 1            | Sync1   | `0x0D`                                                   |
| 2            | ID      | Packet identifier (0-255)                              |
| 3            | Length  | Data payload length (0-140 bytes)                     |
| 4...L+3     | Data    | Payload data                                           |
| L+4         | Fletch0 | Checksum MSB                                         |
| L+5         | Fletch1 | Checksum LSB                                         |

### Checksum Calculation
The Orion protocol uses a modified 16-bit Fletcher's checksum. It is crucial to implement this exactly as specified, or the gimbal (and the simulator) will reject the packets.

**Algorithm:**
1. Initialize two accumulators, `a` and `b`, to `1`.
2. For every byte in the payload (including Sync and ID):
    - `a = (a + byte) % 251`
    - `b = (b + a) % 251`
3. The resulting checksum is a 16-bit value: `(b << 8) | a`.

*Note: The use of modulo 251 (a prime number) instead of the standard 255 is a critical requirement for Orion hardware compatibility.*

---

## 📈 Physics & Dynamics Model

To allow for testing control loops, OrionShadow does not perform instant position updates. Instead, it simulates the inertia and motor response of the gimbal axes.

### Motion Model
The simulator uses a basic Euler integration approach for each axis (Pan/Tilt):

1.  **Targeting**: An `ORION_PKT_CMD` packet sets the `target_position`.
2.  **Acceleration**: The engine calculates required acceleration to reach the target, clamped by `max_acc`.
3.  **Velocity**: $v_{new} = v_{old} + (a \cdot dt)$, clamped by `max_vel`.
4.  **Position**: $p_{new} = p_{old} + (v \cdot dt)$.
5.  **Damping**: A damping coefficient is applied to velocity at each step to simulate mechanical friction and prevent oscillation.

### Tuning for SIL/HIL
- **For SIL (Software-in-the-Loop)**: Use a larger $dt$ (e.g., 0.1s) for faster-than-real-time simulation of logic.
- **For HIL (Hardware-in-the-Loop)**: Use a smaller $dt$ (e.g., 0.01s or 0.001s) to match the high-frequency requirements of real-time embedded controllers.

---

## 🔍 Troubleshooting & Debugging

### Common Issues

| Symptom | Likely Cause | Action |
|---------|--------------|-------|
| **No response from simulator** | Incorrect Sync bytes or Port mismatch | Check that your client is using `0xD0 0x0D` and connecting to the correct port. |
| **Packets rejected/dropped** | Checksum error | Re-verify your Fletcher-251 implementation. |
| **Erratic gimbal movement** | Low telemetry frequency | Increase the telemetry rate or decrease the simulation $dt$. |
| **Commands ignored** | Incorrect Packet ID | Ensure you are using `0x01` for commands and `0x00` for initialization. |

### Debugging Tools
- **Wireshark**: Use the `Orion-Wireshark.lua` plugin (found in the original Orion SDK) to inspect the UDP stream.
- **Logging**: Enable verbose logging in your SDK client to monitor the raw byte sequences being transmitted.

---

## 📝 Integration Checklist

- [ ] Ensure the connection is established via UDP.
- [ ] Send an `ORION_PKT_INITIALIZE` (0x00) packet immediately after connection.
- [ ] Verify that the client is using **Big-endian** byte order.
- [ ] Confirm the Fletcher-251 checksum is applied to every outgoing packet.
- [ ] Match the telemetry frequency of your control loop to the simulator's update rate.
