import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import std_msgs


class Sifter(object):

    def __init__(self):
        self.ref_img = cv2.imread('/home/bhertel/Downloads/screwdriver_sift.jpg')
        rx, ry, _ = np.shape(self.ref_img)
        self.ref_img = cv2.resize(self.ref_img,(int(rx * 0.3), int(ry * 0.3)))
        self.bridge = CvBridge()
        self.ref_blur = cv2.GaussianBlur(self.ref_img,(5,5),cv2.BORDER_DEFAULT)
        self.num_matches = 15
        self.num_neighbors = 5
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.keypoints1, self.des1 = self.sift.detectAndCompute(self.ref_blur, None)
        self.bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
        self.show = False
        self.pub = rospy.Publisher('/object/position', PointStamped, queue_size=1)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()

    def publish_data(self, pt):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = pt[0]
        self.msg.point.y = pt[1]
        self.msg.point.z = 1.0
        self.pub.publish(self.msg)

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        blur2 = cv2.GaussianBlur(frame,(3,3),cv2.BORDER_DEFAULT)
        
        keypoints2, des2 = self.sift.detectAndCompute(blur2, None)
        
        matches1 = self.bf.match(self.des1,des2)
        matches1 = sorted(matches1, key= lambda match : match.distance)
        
        keypoint_dists = []
        for mat1 in matches1[:self.num_matches]:
            dists = []
            for mat2 in matches1[:self.num_matches]:
               dists.append(((keypoints2[mat1.trainIdx].pt[0] - keypoints2[mat2.trainIdx].pt[0])**2 + (keypoints2[mat1.trainIdx].pt[1] - keypoints2[mat2.trainIdx].pt[1])**2)**0.5)
            sort_dists = sorted(dists)
            dist = sum(sort_dists[:self.num_neighbors])
            keypoint_dists.append(dist)
        best_key = np.argmin(keypoint_dists)
        rospy.loginfo("best keypoint: " + str(keypoints2[matches1[best_key].trainIdx].pt)) 
        self.publish_data(keypoints2[matches1[best_key].trainIdx].pt)
        
        if self.show:
            matched_img1 = cv2.drawMatches(self.ref_blur, self.keypoints1, blur2, keypoints2, matches1[:self.num_matches], None)
            start_point = keypoints2[matches1[best_key].trainIdx].pt
            end_point = keypoints2[matches1[best_key].trainIdx].pt
            
            start_point = (int(start_point[0] - 20), int(start_point[1] - 20))
            end_point = (int(end_point[0] + 20), int(end_point[1] + 20))
        
            cv2.rectangle(blur2, start_point, end_point, (0, 0, 255), thickness= 3, lineType=cv2.LINE_8)  
            
            cv2.imshow("object", blur2)
            #cv2.imshow("matched", matched_img1)
            
            cv2.waitKey(100)
            cv2.destroyAllWindows()
    
def sifter():

    rospy.init_node('sifter', anonymous=True)
    
    sf = Sifter()

    rospy.Subscriber('/camera/rgb/image_raw', Image, sf.callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    sifter()
