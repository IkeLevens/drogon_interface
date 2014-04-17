import serial
import rospy
import os
import sys
import std_msgs.msg
import time
import re

ser = serial.Serial(port='/dev/ttyACM0', baudrate=1200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
subject = sys.argv[1]
path = '/home/pracsys/hri/' + subject
if not os.path.exists(path):
	os.makedirs(path)
line = []
running = False
clearing = False
gotTRJ = False
trial = -1;
trj = "foo"
prefix = path + '/' + subject
f = open(prefix + "b" + str(0) + "t" + str(0) + ".txt", 'w')
f.close()
start = time.time()

def webcallback(data):
	global trj
	global gotTRJ
	if not data.data == "/home/pracsys/trajectories/clear.trj":
		trj = data.data
		gotTRJ = True

def callback(data):
	global clearing
	global running
	global trial
	global f
	global trj
	global start
	global gotTRJ
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
			f.close()
		else:
			trial+=1
			block = trial / 40
			inBlock = trial % 40
			f = open(prefix + "b" + str(block) + "t" + str(inBlock) + ".txt", 'w')
			while True:
				if gotTRJ:
					f.write(trj + "\n")
					gotTRJ = False
					break
				else:
					time.sleep(.005)
			start = time.time()
			clearing = False
			running = True

rospy.init_node('SERIAL_CAPTURE', anonymous = True)
rospy.Subscriber("playback", std_msgs.msg.Bool, callback)
rospy.Subscriber("web", std_msgs.msg.String, webcallback)

valid = re.compile('\d\.\d\d')

while True:
	if (running and not clearing):
		try:
			for c in ser.read():
				line.append(c)
				if c == '\n' and not f.closed:
					numberString = ''.join(line)
					number = valid.search(numberString)
					if (number):
						f.write(str(time.time() - start) + '\t')
						f.write(number.group(0))
						f.write('\n')
					line = []
					break
		except:
			continue
f.close()
ser.close()
