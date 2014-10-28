ros_overlay
===========

Usage
-----

```
rosrun ros_overlay overlay.py [OPTION] [DATA_TOPIC [IMAGE_IN_TOPIC [IMAGE_OUT_TOPIC]]].

[OPTION]:
	-v	Verbose. Prints state information.
	-d 	Display generated image.

[DATA_TOPIC]:
	The ROS topic to listen on for geometry information. (Default: image_overlay/data)

[IMAGE_IN_TOPIC]:
	The image topic to take as input. (Default: image_overlay/in)

[IMAGE_OUT_TOPIC]:
	The image topic to publish output to. (Default: image_overlay/out)	
```

Example:

```
rosrun ros_overlay overlay.py test /openni/rgb/image_color -d
```

Data
----

The data declaration is meant to be as general as possible, and so is not particularly elegant. Import `overlay_shapes.py` for some very necessary helper functions!

Right now, we only support circles.

