import serial
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

# Function to calculate checksum
def calculate_checksum(buffer):
    checksum = 0xffff - 0x20
    for i in range(29):
        checksum -= buffer[i]
    return checksum

# Example function to send data
def send_data(channels):
    buffer = bytearray(31)
    buffer[0] = 0x20  # Start byte
    buffer[1] = channels[0] & 0xFF
    buffer[2] = (channels[0] >> 8) & 0xFF
    buffer[3] = channels[1] & 0xFF
    buffer[4] = (channels[1] >> 8) & 0xFF
    buffer[5] = channels[2] & 0xFF
    buffer[6] = (channels[2] >> 8) & 0xFF
    buffer[7] = channels[3] & 0xFF
    buffer[8] = (channels[3] >> 8) & 0xFF
    buffer[9] = channels[4] & 0xFF
    buffer[10] = (channels[4] >> 8) & 0xFF
    buffer[11] = channels[5] & 0xFF
    buffer[12] = (channels[5] >> 8) & 0xFF

    # Calculate checksum
    checksum = calculate_checksum(buffer)
    buffer[29] = checksum & 0xFF
    buffer[30] = (checksum >> 8) & 0xFF

    # Send buffer to the receiver
    ser.write(buffer)

# Example channel data to send (replace with actual data)
channels = [1000, 1500, 2000, 1000, 1500, 2000]

while True:
    send_data(channels)
    time.sleep(0.1)  # Adjust the delay as needed
