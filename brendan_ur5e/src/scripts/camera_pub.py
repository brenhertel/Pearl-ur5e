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
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
    rospy.init_node('camera_rgb_pub', anonymous=True)
    bridge = CvBridge()
    cam = cv2.VideoCapture(2)
    rate = rospy.Rate(20)
    while cam.isOpened() and not rospy.is_shutdown():
        _, frame = cam.read()
        rospy.loginfo('Frame read, publishing...')
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        #cv2.imshow('frame',frame)
        #cv2.waitKey(100)
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
    cv2.imwrite("screwdriver.png", frame)
    
    cam.release()

def camera_load():
    img = cv2.imread("cup.png")
    #img = img[0:350, :]
    cv2.imshow("img", img)
    cv2.waitKey(0)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.destroyAllWindows()
    upper = np.array([180, 255, 255])
    lower = np.array([140, 50, 50])
    img = cv2.inRange(img, lower, upper)
    #cv2.imshow('img1',img)
    kernel = np.ones((5,5),np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    #cv2.imshow('img2',img)
    
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 100
    #params.maxArea = 15000
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = True
    params.minConvexity = 0.87
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_pub()

