#!/usr/bin/env python

'''
import cv2


cam = cv2.VideoCapture(2)
while(1):
    #frame = cv2.imread('/home/bhertel/catkin_ws/src/brendan_ur5e/pictures/lasa_dataset_deformed.png')
    _, frame = cam.read()
    cv2.imshow('Display',frame)
    cv2.waitKey(100)
    cv2.destroyAllWindows()
cam.release()
'''

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_pub():
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=100)
    rospy.init_node('camera_rgb_pub', anonymous=True)
    bridge = CvBridge()
    cam = cv2.VideoCapture(2)
    rate = rospy.Rate(20)
    while cam.isOpened() and not rospy.is_shutdown():
        _, frame = cam.read()
        rospy.loginfo('Frame read, publishing...')
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        #cv2.imshow('frame',frame)
        #cv2.waitKey(1000)
        #cv2.destroyAllWindows()
        rate.sleep()
        #rospy.Rate(1.0).sleep()
    
    cam.release()
    
    
def camera_save():
    #pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=100)
    #rospy.init_node('camera_rgb_pub', anonymous=True)
    #bridge = CvBridge()
    cam = cv2.VideoCapture(2)
    #rate = rospy.Rate(10)
    i = 0
    while cam.isOpened() and i < 100:
        _, frame = cam.read()
        i = i + 1
    #rospy.loginfo('Frame read, publishing...')
    #pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    cv2.imshow('frame',frame)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()
    #rate.sleep()
    #rospy.Rate(1.0).sleep()
    cv2.imwrite("homography_pts.png", frame)
    
    cam.release()

def camera_load():
    img = cv2.imread("homography_pts.png")
    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_pub()

