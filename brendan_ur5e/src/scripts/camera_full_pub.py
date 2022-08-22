#!/usr/bin/env python3.9

import sys

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

import rospy
from brendan_ur5e.msg import image_data
import std_msgs

if ros_path in sys.path:
    sys.path.remove(ros_path)
    
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

#if device_product_line == 'L500':
#    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
#else:
#    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pf = pipeline.start(config)
print(f"device: {pf.get_device()}")
print(f"depth_sensor: {pf.get_device().first_depth_sensor()}")
print(f"depth_scale: {pf.get_device().first_depth_sensor().get_depth_scale()}")
print(f"streams: {pf.get_streams()}")


rospy.init_node('camera_full_pub', anonymous=True)
pub_rgb = rospy.Publisher('/camera/rgb/image_muddy', image_data, queue_size=1)
pub_depth = rospy.Publisher('/camera/depth/image_muddy', image_data, queue_size=1)
#bridge = CvBridge()

rate = rospy.Rate(30)
i = 0
try:
    while not rospy.is_shutdown():
    
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap_dim = depth_image.shape
        color_colormap_dim = color_image.shape
        
        color_img_msg = image_data(timestamp=color_frame.timestamp, height=color_colormap_dim[0], width=color_colormap_dim[1], length=color_colormap_dim[2], encoding="rgb8", data=color_image.flatten().tolist())
        #color_img_msg = image_data(data=color_image.flatten().tolist())
        
        depth_img_msg = image_data(timestamp=depth_frame.timestamp, height=depth_colormap_dim[0], width=depth_colormap_dim[1], length=1, encoding="mono8", data=depth_image.flatten().tolist())
        
        pub_rgb.publish(color_img_msg)
        pub_depth.publish(depth_img_msg)
        i = i + 1
        rate.sleep()
    
finally:

    # Stop streaming
    pipeline.stop()
