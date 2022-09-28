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


# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('841612070149') #right side of table (desk side)
config_1.enable_stream(rs.stream.depth)
config_1.enable_stream(rs.stream.color)
# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('843212070389') #left side of table (door side)
config_2.enable_stream(rs.stream.depth)
config_2.enable_stream(rs.stream.color)


# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)



rospy.init_node('camera_full_pub', anonymous=True)
pub1_rgb = rospy.Publisher('/camera1/rgb/image_muddy', image_data, queue_size=1)
pub1_depth = rospy.Publisher('/camera1/depth/image_muddy', image_data, queue_size=1)

pub2_rgb = rospy.Publisher('/camera2/rgb/image_muddy', image_data, queue_size=1)
pub2_depth = rospy.Publisher('/camera2/depth/image_muddy', image_data, queue_size=1)
#bridge = CvBridge()

rate = rospy.Rate(30)
i = 0
try:
    while not rospy.is_shutdown():
    
        # Camera 1
        print('publishing right camera')
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline_1.wait_for_frames()
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
        
        pub1_rgb.publish(color_img_msg)
        pub1_depth.publish(depth_img_msg)
        
        # Camera 2
        print('publishing left camera')
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline_2.wait_for_frames()
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
        
        pub2_rgb.publish(color_img_msg)
        pub2_depth.publish(depth_img_msg)
        
        rate.sleep()
    
finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()
