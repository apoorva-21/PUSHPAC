import cv2
import numpy as np
import socket
import sys
import pickle
import struct
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

#cap=cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (208, 160)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (208,160))

time.sleep(0.1)

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('192.168.43.100',8089))
print 'Connected!'
for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port = True):
	image = frame.array
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	data = pickle.dumps(image)
	print len(data)
#	cv2.imshow('frame',image)
#	k = cv2.waitKey(1) & 0xFF
#	if k == ord('q'):
#		break
	clientsocket.sendall(struct.pack("L", len(data))+data) #unsigned long for raspberrypi's architecture is of 4 bytes
	print 'Sent!'
	rawCapture.truncate(0)
