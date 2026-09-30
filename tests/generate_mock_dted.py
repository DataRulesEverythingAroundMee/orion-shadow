import struct
import numpy as np
import os

def create_mock_dted(path, rows, cols, lat_min, lat_max, lon_min, lon_max, elevation_func):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header_size = 128
    magic = b'DTED'
    
    with open(path, 'wb') as f:
        # Header
        f.write(magic)
        f.write(struct.pack(">II", rows, cols))
        f.write(struct.pack(">dd", lat_min, lat_max))
        f.write(struct.pack(">dd", lon_min, lon_max))
        f.write(b'\x00' * (header_size - len(magic) - 8 - 16 - 16))
        
        # Data (uint16)
        for r in range(rows):
            for c in range(cols):
                lat = lat_min + (r / (rows - 1)) * (lat_max - lat_min)
                lon = lon_min + (c / (cols - 1)) * (lon_max - lon_min)
                alt = elevation_func(lat, lon)
                f.write(struct.pack(">H", int(round(alt))))

if __name__ == "__main__":
    # Flat terrain at 100m
    create_mock_dted(
        "tests/terrain_data/flat.dt0", 
        10, 10, 
        34.0, 35.0, -119.0, -118.0, 
        lambda lat, lon: 100.0
    )
    
    # Sloped terrain from 0 to 1000m
    create_mock_dted(
        "tests/terrain_data/slope.dt1", 
        10, 10, 
        34.0, 35.0, -119.0, -118.0, 
        lambda lat, lon: (lat - 34.0) * 1000.0
    )
    print("[*] Mock DTED files regenerated.")
