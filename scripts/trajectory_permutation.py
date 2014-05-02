import serial
import os
import sys
import time
import re
import rospy
import std_msgs.msg
import time
import random
import pygame

fileArray = ['/home/pracsys/trajectories/apr17/trj_short', '/home/pracsys/trajectories/apr17/trj_claw', '/home/pracsys/trajectories/apr17/trj_curve', '/home/pracsys/trajectories/apr17/trj_straight', '/home/pracsys/trajectories/apr17/trj_straight_down']
armArray = ['_right_', '_left_']
targetArray = [[3, 4, 5, 6, 7], [9, 10, 11, 12, 13]]
finalArray = []
running = True
pygame.init()

ser = serial.Serial(port='/dev/ttyACM0', baudrate=1200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)
subject = sys.argv[1]
path = '/home/pracsys/hri/' + subject
if not os.path.exists(path):
	os.makedirs(path)
line = []
prefix = path + '/' + subject
f = open(prefix + "b" + str(0) + "t" + str(0).zfill(2) + ".csv", 'w')
f.close()
start = time.time()
valid = re.compile('\d\.\d\d')

#This section of the code is from an answer written by John Millikin on http://stackoverflow.com/questions/1394956/how-to-do-hit-any-key-in-python
try:
	# Win32
    from msvcrt import getch
except ImportError:
    # UNIX
    def getch():
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
#End of John's code.

def callback(data):
	global running
	running = data.data

rospy.init_node('TRAJECTORY_PERMUTATION', anonymous = True)
pub = rospy.Publisher('web', std_msgs.msg.String, tcp_nodelay = True)
rospy.Subscriber("playback", std_msgs.msg.Bool, callback)
time.sleep(1)
clear = '/home/pracsys/trajectories/apr17/clear.trj'

for i in range(0,2):
	for j in range(0,5):
		for k in range(0,5):
			finalArray.append(fileArray[j] + armArray[i] + str(targetArray[i][k]) + '.ts')

pub.publish(data = clear)
while (running):
	time.sleep(0.05)

for i in range(0,3):
	random.shuffle(finalArray)
	for j in range(0,50):
		print str(50 * i + j) + ' trials run, press any key to begin next trial. (Do not press escape.)'
		a = getch()
		if ord(a) == 27:
			exit()
		pub.publish(data = finalArray[j])
		if finalArray[j][-10:-5] == 'right':
			pygame.mixer.music.load("/var/www/Baxter/buzz.wav")
			pygame.mixer.music.play()
		else:
			pygame.mixer.music.load("/var/www/Baxter/bell.wav")
			pygame.mixer.music.play()
		running = True
		f = open(prefix + "b" + str(i) + "t" + str(j).zfill(2) + ".csv", 'w')
		f.write(finalArray[j] + '\n')
		start = time.time()
		while (running):
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
		line = []
		pub.publish(data = clear)
		running = True
		while (running):
			time.sleep(0.05)
f.close()
ser.close()
