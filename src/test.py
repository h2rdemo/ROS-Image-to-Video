#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_overlay')
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_overlay.msg import *
import overlay_shapes as shp

data_topic = "image_overlay/data"

def main(args):
	global image_pub, image_sub, bridge

	rospy.init_node('ros_overlay_test', anonymous=True)

	data_pub = rospy.Publisher(data_topic,Shape)

	print "Simple circle demo. Usage:\nid x y [r [cr cg cb]] # Draw or replace; OR\nid # Delete"

	_r = 30
	_cr = 255
	_cg = 000
	_cb = 128


	try:
		while not rospy.is_shutdown():
			linemsg = None
			inp = [int(ss) for ss in raw_input('->\t').split()]
			if len(inp) == 1:
				linemsg = shp.overlay_delete(inp[0])
			elif len(inp) >= 3:
				i = inp[0]
				x = inp[1]
				y = inp[2]
				if len(inp) >= 4:
					_r = inp[3]
					if len(inp) == 7:
						_cr = inp[4]
						_cg = inp[5]
						_cb = inp[6]

				print "\t".join(str(itm) for itm in [i, x, y, _r, _cr, _cg, _cb])

				linemsg = shp.overlay_circle(i, (x, y), _r, (_cb, _cg, _cr))

			if linemsg is not None:
				data_pub.publish(linemsg)

	except KeyboardInterrupt, EOFError:
		print "User Terminated..."

	if opt_gui:
		cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)