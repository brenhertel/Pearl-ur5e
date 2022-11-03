## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#needs python3.9

###############################################
##      Open CV and Numpy integration        ##
###############################################
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

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

#device.first_depth_sensor().set_option(rs.option.laser_power, 200)
#device.first_depth_sensor().set_option(rs.option.depth_units, 0.0001)

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

align_to = rs.stream.color
align = rs.align(align_to)

#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgra8, 30)

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

a = 0
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        
        print(frames.timestamp)
        aligned_frames = align.process(frames)
        print(aligned_frames.timestamp)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        mono_depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        print(color_frame.timestamp)
        a += 1

    #cv2.imwrite("depth_image.png", mono_depth_image)
    #cv2.imwrite("color_image.png", color_image)
finally:

    # Stop streaming
    pipeline.stop()