import socket
import sys
import cv2
import pickle
import numpy as np
import struct
#Code to receive the incoming UDP stream, to display real time video at the Ground Station,
#and to write out magnetic data to a local file

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

numIters = 0
open('magnetoPlot.txt' , 'w')

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
    #frame = allData[0]
    extraData = allData[0]
    # print frames
    #frame = cv2.flip(frame,0)
    #cv2.putText(frame, 'LAT:{}'.format(extraData['lat']), (0,20), font, 0.5, (0,255,0),1)
    #cv2.putText(frame, 'LON:{}'.format(extraData['long']), (0,34), font, 0.5, (0,255,0),1) 48,62,76
    #cv2.putText(frame, 'MAG_X:{}'.format(extraData['mx']), (0,20), font, 0.5, (0,255,0),1)
    #cv2.putText(frame, 'MAG_Y:{}'.format(extraData['my']), (0,34), font, 0.5, (0,255,0),1)
    #cv2.putText(frame, 'MAG_Z:{}'.format(extraData['mz']), (0,48), font, 0.5, (0,255,0),1)    
    if extraData['an'] == 1:
        cv2.putText(frame,'MAG ANOMALY!!', (50,120), font, 0.8, (0,0,0),2)   
    # plot magneto Data:
    with open('magnetoPlot.txt','a') as f:
        modMag = np.sqrt(extraData['mx']**2 + extraData['my']**2 + extraData['mz']**2)
        s = str(numIters) + ',' + str(modMag) + '\n'
        f.write(s)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break
    numIters += 1
cv2.destroyAllWindows()
exit()
