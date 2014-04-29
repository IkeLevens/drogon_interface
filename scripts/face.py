import rospy
import sensor_msgs.msg

def callback(data):
	pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
	pub.publish(data)
	rospy.sleep(1)
	rospy.signal_shutdown("image captured")
	
rospy.init_node("CAMERA_FACE_DISPLAY")
rospy.Subscriber("/cameras/left_hand_camera/image", sensor_msgs.msg.Image, callback)
rospy.spin()
