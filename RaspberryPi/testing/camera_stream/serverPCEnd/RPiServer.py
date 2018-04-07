import socket
import sys
import cv2
import pickle
import numpy as np
import struct

HOST='192.168.43.100'
PORT=8089

font = cv2.FONT_HERSHEY_SIMPLEX

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print 'Socket created'

s.bind((HOST,PORT))
print 'Socket bind complete'
s.listen(1)
print 'Socket now listening'

conn,addr=s.accept()
data = ""
payload_size = struct.calcsize("i") 
while True:
    while len(data) < payload_size:
        data += conn.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("i", packed_msg_size)[0]
    while len(data) < msg_size:
        data += conn.recv(4096)
     #   print 'receiving', len(data), msg_size
    frame_data = data[:msg_size]
    data = data[msg_size:]
    allData = pickle.loads(frame_data)
    frame = allData[0]
    extraData = allData[1]
    # print frames
    frame = cv2.flip(frame,0)
    #put the readings on screen
    cv2.putText(frame, 'LAT:{}'.format(extraData['lat']), (0,20), font, 0.5, (0,255,0),1)
    cv2.putText(frame, 'LON:{}'.format(extraData['long']), (0,34), font, 0.5, (0,255,0),1)
    cv2.putText(frame, 'MAG_X:{}'.format(extraData['mx']), (0,48), font, 0.5, (0,255,0),1)
    cv2.putText(frame, 'MAG_Y:{}'.format(extraData['my']), (0,62), font, 0.5, (0,255,0),1)
    cv2.putText(frame, 'MAG_Z:{}'.format(extraData['mz']), (0,76), font, 0.5, (0,255,0),1)    
    if extraData['an'] == 1:
        cv2.putText(frame,'MAG ANOMALY!!', (100,120), font, 1, (0,0,0),2)    
    # print extraData
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break

cv2.destroyAllWindows()
exit()
