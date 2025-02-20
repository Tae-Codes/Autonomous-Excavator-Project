import serial
import time
 
buffer = bytearray(31)


ser = serial.Serial('/dev/ttyS0', 115200, timeout=0)

while True:
    if ser.in_waiting > 0:
        c = ser.read(1)
        if c and c[0] == 0x20:
            if ser.in_waiting >= len(buffer):
                ser.readinto(buffer)
                checksum = 0xffff - 0x20
                for i in range(29):
                    checksum -= buffer[i]
                if checksum == (buffer[30] << 8) | buffer[29]:
                    buffer [0] = 0x40
                    ch1 = buffer[2] * 255 + buffer[1]
                    ch2 = buffer[4] * 255 + buffer[3]
                    ch3 = buffer[6] * 255 + buffer[5]
                    ch4 = buffer[8] * 255 + buffer[7]
                    ch5 = buffer[10] * 255 + buffer[9]
                    ch6 = buffer[12] * 255 + buffer[11]
                    ch7 = buffer[14] * 255 + buffer[13]
                    ch8 = buffer[16] * 255 + buffer[15]
                    ch9 = buffer[18] * 255 + buffer[17]
                    ch10 = buffer[20] * 255 + buffer[19]                 
                
                    print(f'ch 1- {ch1} 2- {ch2} 3- {ch3} 4- {ch4} 5- {ch5} 6- {ch6} 7- {ch7} 8- {ch8} 9- {ch9} 10- {ch10}')