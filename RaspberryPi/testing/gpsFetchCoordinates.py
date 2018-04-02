from pynmea import nmea
import serial, time, sys, threading#, dateutil, shutil

BAUD = 9600
GPS_PORT_NUM = '/dev/ttyUSB0'
ser = []
def init_serial():
	global ser
	ser = serial.Serial()
	ser.baudrate = BAUD
	ser.port = GPS_PORT_NUM
	ser.timeout = 1
	ser.open()
	ser.isOpen()
	#receive raw serial data:
	
def getCoordinates(rawSplit):

	location = dict()
	location['time'] = rawSplit[1]
	location['latitude'] = rawSplit[2]+rawSplit[3]
	location["longitude"] = rawSplit[4]+rawSplit[5]
	location["fixQuality"] = rawSplit[6]
	location["nSats"] = rawSplit[7]
	location["altitude"] = rawSplit[9]
	return location

init_serial()
while ser.isOpen():
	raw = ''
	raw = str(ser.readline())
	if len(raw) < 50:
		print "GPS not BOUND"
	else:
		rawSplit = raw.split(',')
		if rawSplit[0] == '$GPGGA':
			location= getCoordinates(rawSplit)
			print location
