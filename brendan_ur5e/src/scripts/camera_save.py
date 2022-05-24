import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped


class Recorder(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.show = False
        self.img_list = []

    def callback(self, data):
        rospy.loginfo('Adding Image')
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        self.img_list.append(img)
    
    def save_vid(self):
        rospy.loginfo('Saving Video')
        
        height, width, layers = self.img_list[0].shape
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        video = cv2.VideoWriter('ur5e_video.mp4', fourcc, 20, (width,height))
        for i in range(len(self.img_list)):
            video.write(self.img_list[i])
        cv2.destroyAllWindows()
        video.release()
    

class Sift_Recorder(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.show = False
        self.img_list = []
        self.pt_list = []
        self.x = 0
        self.y = 0
        self.alpha = 0.2

    def callback(self, data):
        #rospy.loginfo('Adding Image')
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        self.img_list.append(img)
        self.pt_list.append([self.x, self.y])

    def pt_callback(self, msg):
        #rospy.loginfo('Adding Point')
        self.x = self.alpha * msg.point.x + (1 - self.alpha) * self.x
        self.y = self.alpha * msg.point.y + (1 - self.alpha) * self.y
    
    def save_vid(self):
        rospy.loginfo('Saving Video')
        
        height, width, layers = self.img_list[0].shape
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        video = cv2.VideoWriter('screwdriver_video_2.mp4', fourcc, 20, (width,height))
        for i in range(len(self.img_list)):
            if self.pt_list[i][0] is not None:
                start_point = (int(self.pt_list[i][0] - 20), int(self.pt_list[i][1] - 20))
                end_point = (int(self.pt_list[i][0] + 20), int(self.pt_list[i][1] + 20))
                cv2.rectangle(self.img_list[i], start_point, end_point, (0, 0, 255), thickness= 3, lineType=cv2.LINE_8)  
            video.write(self.img_list[i])
        cv2.destroyAllWindows()
        video.release()
        
def detector():

    rospy.init_node('Recorder', anonymous=True)
    
    rec = Sift_Recorder()

    try:
        rospy.Subscriber('/object/position', PointStamped, rec.pt_callback)
    	print('Press [Enter] to start recording')
    	raw_input()
        rospy.Subscriber('/camera/rgb/image_raw', Image, rec.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        rec.save_vid()
    except:
        rec.save_vid()
    #except rospy.ROSInterruptException:
    #    print('Ros interrupt')
    #    rec.save_vid()
    #except rospy.ServiceException, e: 
    #    print('Ros service exception')
    #    rec.save_vid()
    #    print "Service call failed: %s"%e
    #except KeyboardInterrupt:
    #    print('keyboard interrupt')
    #    rec.save_vid()


if __name__ == '__main__':
    detector()
