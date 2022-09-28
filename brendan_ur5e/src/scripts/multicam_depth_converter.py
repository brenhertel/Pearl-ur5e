#!/usr/bin/env python


import cv2
import numpy as np
import rospy
from brendan_ur5e.msg import image_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraConverter(object):
    
    def __init__(self):
        self.pub_depth1 = rospy.Publisher('/camera1/depth/image_raw', Image, queue_size=1)
        self.pub_depth2 = rospy.Publisher('/camera2/depth/image_raw', Image, queue_size=1)
        #self.pub_depth_res = rospy.Publisher('/camera/depth/image_resized', Image, queue_size=1)
        self.bridge = CvBridge()
        #self.height_crop_frac = (1 - 69./87.)/2
        #self.width_crop_frac = (1 - 42./58.)/2

    
    def camera_conversion_depth1(self, msg):
    
        b = bytearray()
        b.extend(map(ord, msg.data))
        new_data = list(b)
        #print(new_data[0:3])
        img_data = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        self.pub_depth1.publish(self.bridge.cv2_to_imgmsg(img_data, msg.encoding))
        #image_arr = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        #print(image_arr)
        #cv2.imshow("frame", image_arr)
        #cv2.waitKey(10)
        #cv2.destroyAllWindows()
        #height_low_ind = int(self.height_crop_frac * msg.height)
        #height_hi_ind = int((1 - self.height_crop_frac) * msg.height)
        #width_low_ind = int(self.width_crop_frac * msg.height)
        #width_hi_ind = int((1 - self.width_crop_frac) * msg.height)
        #print(img_data.shape)
        #crop_img = img_data[width_low_ind:width_hi_ind, height_low_ind:height_hi_ind]
        #print(crop_img.shape)
        #resize_img = cv2.resize(crop_img, (msg.width, msg.height))
        #self.pub_depth_res.publish(self.bridge.cv2_to_imgmsg(resize_img, msg.encoding))
        
    def camera_conversion_depth2(self, msg):
    
        b = bytearray()
        b.extend(map(ord, msg.data))
        new_data = list(b)
        #print(new_data[0:3])
        img_data = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        self.pub_depth2.publish(self.bridge.cv2_to_imgmsg(img_data, msg.encoding))
        #image_arr = np.array(new_data, dtype = np.uint8).reshape((msg.height, msg.width, msg.length))
        #print(image_arr)
        #cv2.imshow("frame", image_arr)
        #cv2.waitKey(10)
        #cv2.destroyAllWindows()
        #height_low_ind = int(self.height_crop_frac * msg.height)
        #height_hi_ind = int((1 - self.height_crop_frac) * msg.height)
        #width_low_ind = int(self.width_crop_frac * msg.height)
        #width_hi_ind = int((1 - self.width_crop_frac) * msg.height)
        #print(img_data.shape)
        #crop_img = img_data[width_low_ind:width_hi_ind, height_low_ind:height_hi_ind]
        #print(crop_img.shape)
        #resize_img = cv2.resize(crop_img, (msg.width, msg.height))
        #self.pub_depth_res.publish(self.bridge.cv2_to_imgmsg(resize_img, msg.encoding))
        
        
    
if __name__ == '__main__':
    cc = CameraConverter()
    rospy.init_node('depth_converter', anonymous=True)
    sub_rgb = rospy.Subscriber('/camera1/depth/image_muddy', image_data, cc.camera_conversion_depth1)
    sub_rgb = rospy.Subscriber('/camera2/depth/image_muddy', image_data, cc.camera_conversion_depth2)
    rospy.spin()
