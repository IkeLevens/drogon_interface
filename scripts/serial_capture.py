import serial
import rospy
import os
import sys
import std_msgs.msg
import time

ser = serial.Serial(
	port='/dev/ttyACM0',\
	baudrate=1200,\
	parity=serial.PARITY_NONE,\
	stopbits=serial.STOPBITS_ONE,\
	bytesize=serial.EIGHTBITS,\
		timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
subject = sys.argv[1]
if not os.path.exists(subject):
	os.makedirs(subject)
line = []
running = False;
clearing = False;
trial = 0;
trj = "foo"
f = open(subject + "/" + subject + str(trial) + ".txt", 'w')

def webcallback(data):
	global trj
	if not data.data == "/home/pracsys/trajectories/clear.trj":
		trj = data.data

def callback(data):
	global clearing
	global running
	global trial
	global f
	global trj
	time.sleep(.015)
	print trj
	if (clearing):
		if (running):
			clearing = False
			running = False
		else:
			clearing = True
			running = True
	else:
		if (running):
			clearing = True
			running = False
		else:
			clearing = False
			running = True
			trial+=1
			f.close()
			f = open(subject + "/" + subject + str(trial) + ".txt", 'w')
			f.write(trj + "\n")

rospy.init_node('SERIAL_CAPTURE', anonymous = True)
rospy.Subscriber("playback", std_msgs.msg.Bool, callback)
rospy.Subscriber("web", std_msgs.msg.String, webcallback)

while True:
	if (running and not clearing):
		for c in ser.read():
			line.append(c)
			if c == '\n' and not f.closed:
				f.write(''.join(line)[:-2] + "\n")
				line = []
				break
f.close()
ser.close()
