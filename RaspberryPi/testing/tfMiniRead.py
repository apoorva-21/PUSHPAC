#reading raw data from the serial port from the tfmini 1.5.3 LiDAR using the CP2102 ttl-usb converter

import serial
import time

ser = serial.Serial('/dev/ttyUSB0',115200,timeout = 1)
ser.write(bytes(0x42))
ser.write(bytes(0x57))
ser.write(bytes(0x02))
ser.write(bytes(0x00))
ser.write(bytes(0x00))
ser.write(bytes(0x00))
ser.write(bytes(0x01))
ser.write(bytes(0x06))

while(True):
    while(ser.in_waiting >= 9):
        if(('Y' == ser.read()) and ('Y' == ser.read())):

            dist_L = ser.read()
            dist_H = ser.read()
            totalDist = (ord(dist_H) * 256) + (ord(dist_L))
            for i in range (0,5):
                ser.read()
        print totalDist
