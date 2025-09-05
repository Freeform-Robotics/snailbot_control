# Send control_data_t message over COM7
import serial
import struct

def crc16(data: bytes, poly=0x1021, start=0xFFFF):
    crc = start
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# Message fields
header = 0x03
vel_x = 0.5         # example value
vel_rot = 0.1       # example value

# Pack header, vel_x, vel_rot (little-endian)
payload = struct.pack('<Bdd', header, vel_x, vel_rot)

# Compute CRC16 over first 17 bytes (header + vel_x + vel_rot)
checksum = crc16(payload)

# Pack checksum (little-endian)
msg = payload + struct.pack('<H', checksum)

# Send over COM7
with serial.Serial('COM7', 115200, timeout=1) as ser:
    ser.write(msg)
    print("Message sent:", msg.hex())