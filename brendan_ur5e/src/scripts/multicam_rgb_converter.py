#!/usr/bin/env python


import cv2
import numpy as np
import rospy
from brendan_ur5e.msg import image_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraConverter(object):
    
    def __init__(self):
        self.pub_rgb1 = rospy.Publisher('/camera1/rgb/image_raw', Image, queue_size=1)
        self.pub_rgb2 = rospy.Publisher('/camera2/rgb/image_raw', Image, queue_size=1)
        #self.pub_depth_res = rospy.Publisher('/camera/depth/image_resized', Image, queue_size=1)
        self.bridge = CvBridge()
        #self.height_crop_frac = (1 - 69./87.)/2
        #self.width_crop_frac = (1 - 42./58.)/2

    def camera_conversion_rgb1(self, msg):
    
        b = bytearray()
        b.extend(map(ord, msg.data))
        new_data = list(b)
        #print(new_data[0:3])
        self.pub_rgb1.publish(self.bridge.cv2_to_imgmsg(np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length)), msg.encoding))
        #image_arr = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        #print(image_arr)
        #cv2.imshow("frame", image_arr)
        #cv2.waitKey(10)
        #cv2.destroyAllWindows()

    def camera_conversion_rgb2(self, msg):
    
        b = bytearray()
        b.extend(map(ord, msg.data))
        new_data = list(b)
        #print(new_data[0:3])
        self.pub_rgb2.publish(self.bridge.cv2_to_imgmsg(np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length)), msg.encoding))
        #image_arr = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        #print(image_arr)
        #cv2.imshow("frame", image_arr)
        #cv2.waitKey(10)
        #cv2.destroyAllWindows()
    
        
        
    
if __name__ == '__main__':
    cc = CameraConverter()
    rospy.init_node('rgb_converter', anonymous=True)
    sub_rgb = rospy.Subscriber('/camera1/rgb/image_muddy', image_data, cc.camera_conversion_rgb1)
    sub_rgb = rospy.Subscriber('/camera2/rgb/image_muddy', image_data, cc.camera_conversion_rgb2)
    rospy.spin()
