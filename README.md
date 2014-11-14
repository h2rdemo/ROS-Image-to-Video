ros_image_to_video
==================

Usage
-----

```
rosrun ros_image_to_video i2v.py IMAGE_FILE [IMAGE_OUT_TOPIC [FREQUENCY]]

IMAGE_FILE:
	The image file to publish. Required.

[IMAGE_OUT_TOPIC]:
	The image topic to publish output to. (Default: image_to_video/out)	

[FREQUENCY]:
	How many times a second to publish the image. (Default: 10)
```

Example:

```
rosrun ros_image_to_video i2v.py cit.jpg image_overlay/in 30
```

