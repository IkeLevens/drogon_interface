import roslib
import rospy
roslib.load_manifest('baxter_interface')
import baxter_interface
import sys
import time

def main():
	rospy.init_node("gripper_control_python_script")
	rs = baxter_interface.RobotEnable()
	rs.enable()
	if (sys.argv[2] == 'left'):
		grip = baxter_interface.Gripper('left')
	else:
		grip = baxter_interface.Gripper('right')
	time.sleep(1)
	if (sys.argv[1] == 'close'):
		grip.close()
	else:
		grip.open()
	#rs.disable()

if __name__ == '__main__':
	main()
