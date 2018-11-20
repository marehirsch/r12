import serial
import time

port = serial.Serial("/dev/ttyACM0", baudrate=19200, timeout=3.0)

while True:
	port.write("Time: " + str(time.time()) + "\n")
    	rcv = port.readline()
    	print rcv
