import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import std_msgs


class Detector(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.show = False
        self.pub = rospy.Publisher('/object/position', PointStamped, queue_size=100)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()
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
        self.detector = cv2.SimpleBlobDetector_create(params)
        
        self.upper = np.array([180, 255, 255])
        self.lower = np.array([140, 40, 40])
        self.kernel = np.ones((5,5),np.uint8)

    def publish_data(self, pt):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = pt[0]
        self.msg.point.y = pt[1]
        self.msg.point.z = 1.0
        self.pub.publish(self.msg)

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        img = img[0:350, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = cv2.inRange(img, self.lower, self.upper)
        #cv2.imshow('img1',img)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, self.kernel)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, self.kernel)
        img = cv2.dilate(img, self.kernel, iterations=2)
        
        keypoints = self.detector.detect(img)
        if keypoints:
            rospy.loginfo("best keypoint: " + str(keypoints[0].pt)) 
            self.publish_data(keypoints[0].pt)
        
        if self.show:
            im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("Keypoints", im_with_keypoints)
            cv2.waitKey(100)
            cv2.destroyAllWindows()
    
def detector():

    rospy.init_node('Detector', anonymous=True)
    
    det = Detector()

    rospy.Subscriber('/camera/rgb/image_raw', Image, det.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    detector()
