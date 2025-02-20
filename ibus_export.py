import serial
import struct
import time

# Set up UART
uart = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)

def calculate_checksum(packet):
    checksum = 0xFFFF
    for byte in packet:
        checksum -= byte
    return checksum & 0xFF  # Return lower byte of checksum

def send_ibus_packet(channels):
    # iBus packet format: [0x20, data (30 bytes), checksum (1 byte)]
    packet = bytearray([0x20])  # Start byte
    
    # Add channel values (16-bit, little-endian)
    for channel in channels:
        packet.extend(struct.pack('<H', channel))
    
    # Calculate and add checksum
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    
    # Send packet
    uart.write(packet)

# Example usage
try:
    while True:
        # Example channel values (1000-2000 for RC systems)
        channels = [1500, 1500, 1000, 2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        
        # Send iBus packet
        send_ibus_packet(channels)
        
        # Wait before sending the next packet
        time.sleep(0.02)  # 50 Hz (standard iBus frequency)

except KeyboardInterrupt:
    print("Exiting...")