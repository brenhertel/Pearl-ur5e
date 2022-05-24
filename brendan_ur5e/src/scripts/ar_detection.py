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
        self.show = True
        self.pub = rospy.Publisher('/object/position', PointStamped, queue_size=100)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()
        #This line can be changed depending on what dictionary is being used (ex. 5X5 -> 4X4)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
    def publish_data(self, pt):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = pt[0]
        self.msg.point.y = pt[1]
        self.msg.point.z = 1.0
        self.pub.publish(self.msg)
        
    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        resize = cv2.resize(frame, dsize=(600, 800))
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        
        if len(corners) > 0:
            ids = ids.flatten()
            
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                
                
      
                rospy.loginfo("AR center: (" + str(cX) + "," + str(cY) + ")")
                #these lines are causing an error
                #self.publish_data(cX.pt)
                self.publish_data([cX, cY])
                
        if self.show:
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            
            cv2.putText(frame, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            
            cv2.imshow("Frame", frame)
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
