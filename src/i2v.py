#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_image_to_video')
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

src_file = ""
publish_topic = "image_to_video/out"
publish_freq = 10

def main(args):
	global src_file, publish_freq, publish_topic

	# Update the global state vars with the user input:
	handle_args(args)

	rospy.init_node('ros_image_to_video', anonymous=True)
	image_pub = rospy.Publisher(publish_topic,Image)
	bridge = CvBridge()


	# Load the image

	im = cv2.imread(src_file)
	if im is None:
		print "Cannot open file " + src_file
		exit()

	im = bridge.cv2_to_imgmsg(im, "bgr8")

	try:
		r = rospy.Rate(publish_freq) # 10hz
		while not rospy.is_shutdown():
			image_pub.publish(im)
			r.sleep()

	except KeyboardInterrupt:
		print "User Terminated..."

def handle_args(args):
	global publish_topic, publish_freq, src_file

	# Strip all options:
	args = [arg for arg in args if not arg.startswith("-")]

	if len(args) >= 2:
		src_file = args[1]
		
		if len(args) >= 3:
			publish_topic = args[2]

			if len(args) >= 4:
				publish_freq = int(args[3])
	else:
		print("""ROS Image to Video

rosrun ros_image_to_video i2v.py IMAGE_FILE [IMAGE_OUT_TOPIC [FREQUENCY]]

IMAGE_FILE:
	The image file to publish. Required.

[IMAGE_OUT_TOPIC]:
	The image topic to publish output to. (Default: image_to_video/out)	

[FREQUENCY]:
	How many times a second to publish the image. (Default: 30)
""")
		exit()

if __name__ == '__main__':
		main(sys.argv)