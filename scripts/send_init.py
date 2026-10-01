#!/usr/bin/env python3
import socket
import sys
import os
import argparse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from orion_shadow.core.protocol import OrionPacket, OrionPktType

def send_initialize(host='127.0.0.1', port=8745):
    packet = OrionPacket(OrionPktType.INITIALIZE, b"")
    data = packet.encode()
    
    print(f"[*] Sending INITIALIZE packet via UDP to {host}:{port}...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(5)
            print(f"[*] Sending INITIALIZE packet: {data.hex().upper()}")
            sock.sendto(data, (host, port))
            print("[+] Packet sent successfully.")
            
            # Wait for echo/response
            try:
                response, _ = sock.recvfrom(1024)
                if response:
                    print(f"[+] Received response ({len(response)} bytes): {response.hex().upper()}")
                    if data in response:
                        print("[+] INITIALIZE command successfully acknowledged by Orion server.")
                else:
                    print("[-] No response received.")
            except socket.timeout:
                print("[-] Timeout waiting for server response.")
                
    except ConnectionRefusedError:
        print(f"[!] Connection refused: Orion server is not running on {host}:{port}.")
        print("[!] Start the simulator using: ./scripts/run.sh")
    except Exception as e:
        print(f"[!] Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send INITIALIZE command to Orion simulator")
    parser.add_argument("--host", default="127.0.0.1", help="Orion server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8745, help="Orion server port (default: 8745)")
    args = parser.parse_args()
    
    send_initialize(host=args.host, port=args.port)
