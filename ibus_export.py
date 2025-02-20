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
    
    # If there are fewer than 15 channels, pad with 1500 (neutral value)
    while len(packet) < 31:  # 1 start byte + 30 data bytes
        packet.extend(struct.pack('<H', 1500))  # Neutral value for unused channels
    
    # Calculate and add checksum
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    
    # Send packet
    uart.write(packet)

# Example usage
try:
    while True:
        # Define channel values (1000-2000 for RC systems)
        # Example: 10 channels (throttle, steering, aux1, aux2, etc.)
        channels = [
            1500,  # Channel 1: Throttle (neutral)
            1500,  # Channel 2: Steering (neutral)
            1000,  # Channel 3: Aux 1 (e.g., arm/disarm)
            2000,  # Channel 4: Aux 2 (e.g., mode switch)
            1500,  # Channel 5: Unused (neutral)
            1500,  # Channel 6: Unused (neutral)
            1500,  # Channel 7: Unused (neutral)
            1500,  # Channel 8: Unused (neutral)
            1500,  # Channel 9: Unused (neutral)
            1500,  # Channel 10: Unused (neutral)
        ]
        
        # Send iBus packet
        send_ibus_packet(channels)
        
        # Wait before sending the next packet
        time.sleep(0.02)  # 50 Hz (standard iBus frequency)

except KeyboardInterrupt:
    print("Exiting...")