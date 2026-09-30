import socket
import struct

def send_initialize(host='127.0.0.1', port=5000):
    # OrionPublic Packet: 
    # Sync0 (0xD0), Sync1 (0x0D), PacketID (0x01 for INITIALIZE), Length (0x00)
    # Total header length is 4 bytes.
    
    packet_id = 0x01
    length = 0x00
    header = struct.pack(">BBBB", 0xD0, 0x0D, packet_id, length)
    
    print(f"[*] Connecting to {host}:{port}...")
    try:
        with socket.create_connection((host, port), timeout=5) as sock:
            print(f"[*] Sending INITIALIZE packet: {header.hex().upper()}")
            sock.sendall(header)
            print("[+] Packet sent successfully.")
            
            # Wait for echo/response
            response = sock.recv(1024)
            if response:
                print(f"[+] Received response: {response.hex().upper()}")
            else:
                print("[-] No response received.")
                
    except Exception as e:
        print(f"[!] Error: {e}")

if __name__ == "__main__":
    send_initialize()
