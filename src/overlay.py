#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_overlay')
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_overlay.msg import *
from cv_bridge import CvBridge, CvBridgeError
import overlay_shapes as shp

data_topic = "image_overlay/data"
subscribe_topic = "image_overlay/in"
publish_topic = "image_overlay/out"
opt_verbose = False
opt_gui = False

window_name = "Overlay Preview"

image_pub = None
image_sub = None
bridge = None

shapes = {}

def main(args):
	global image_pub, image_sub, bridge

	# Update the global state vars with the user input:
	handle_args(args)

	rospy.init_node('ros_overlay', anonymous=True)

	image_pub = rospy.Publisher(publish_topic,Image)

	if opt_gui:
		cv2.namedWindow(window_name, 1)

	bridge = CvBridge()
	data_sub = rospy.Subscriber(data_topic,Shape,callback_shape)
	image_sub = rospy.Subscriber(subscribe_topic,Image,callback_render)

	callback_shape(shp.overlay_circle(0, (100, 200), 50, (255, 0, 128)))
	callback_shape(shp.overlay_circle(1, (300, 300), 70, (255, 255, 0)))

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "User Terminated..."

	if opt_gui:
		cv2.destroyAllWindows()


def handle_args(args):
	global data_topic, subscribe_topic, publish_topic, opt_verbose, opt_gui

	# Parse args:
	for arg in args:
		if arg.startswith("-"):
			if "v" in arg:
				opt_verbose = True
			if "d" in arg:
				opt_gui = True

	# Strip all options:
	args = [arg for arg in args if not arg.startswith("-")]

	if len(args) >= 2:
		data_topic = args[1]
		
		if len(args) >= 3:
			subscribe_topic = args[2]

			if len(args) >= 4:
				publish_topic = args[3]
	else:
		print("""ROS Overlay: Simple image overlays.
Usage: rosrun ros_overlay ros_overlay.py [OPTION] [DATA_TOPIC [IMAGE_IN_TOPIC [IMAGE_OUT_TOPIC]]].

[OPTION]:
	-v	Verbose. Prints state information.
	-d 	Display generated image.

[DATA_TOPIC]:
	The ROS topic to listen on for geometry information. (Default: image_overlay/data)

[IMAGE_IN_TOPIC]:
	The image topic to take as input. (Default: image_overlay/in)

[IMAGE_OUT_TOPIC]:
	The image topic to publish output to. (Default: image_overlay/out)	
""")
		exit()


def callback_render(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:                                                                                                                                                                                                              
		print e

	(rows,cols,channels) = cv_image.shape

	for sid, shape in shapes.iteritems():
		# Switch based on the type of shape:
		if shape.type == Shape.SHP_NONE:
			print "Shape " + str(sid) + " is of type SHP_NONE."
		elif shape.type == Shape.SHP_CIRCLE:
			cv2.circle(cv_image, (shape.poly[0].x,shape.poly[0].y), shape.param[0], (shape.color[0], shape.color[1], shape.color[2]), -1 if shape.fill else int(shape.thickness))
		else:
			print "Shape " + str(sid) + " is of unrecognized type " + str(shape.type) + "."

	if opt_gui:
		cv2.imshow(window_name, cv_image)
		cv2.waitKey(3)

	try:
		image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	except CvBridgeError, e:
		print e

def callback_shape(data):
	if data.delete:
		shapes.pop(data.id, None)
		if opt_verbose:
			print "Removed " + data.id
	else:
		shapes[data.id] = data
		if opt_verbose:
			print "Added/Updated " + data.id

if __name__ == '__main__':
		main(sys.argv)