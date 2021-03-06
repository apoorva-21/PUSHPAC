import socket
import sys
import cv2
import pickle
import numpy as np
import struct

HOST='192.168.43.100'
PORT=8089

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print 'Socket created'

s.bind((HOST,PORT))
print 'Socket bind complete'
s.listen(1)
print 'Socket now listening'

conn,addr=s.accept()

data = ""
payload_size = struct.calcsize("i")  #int has struct size of 4 bytes. unsigned long here has a size of 8 bytes, so fails reading from RPi DGs properly!
while True:
    while len(data) < payload_size:
        data += conn.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("i", packed_msg_size)[0]
    while len(data) < msg_size:
        data += conn.recv(4096)
        print 'receiving', len(data), msg_size
    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame=pickle.loads(frame_data)
    # print frames
    frame = cv2.flip(frame,0)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break

cv2.destroyAllWindows()
exit()
