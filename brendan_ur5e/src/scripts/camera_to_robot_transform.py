#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
import std_msgs


class Transformer(object):

    def __init__(self):
        self.H = np.loadtxt('homography.txt')
        self.pub = rospy.Publisher('/object/position_tf', PointStamped, queue_size=100)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()

    def publish_data(self, pt):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = pt[0]
        self.msg.point.y = pt[1]
        self.msg.point.z = pt[2]
        self.pub.publish(self.msg)

    def callback(self, msg):
        in_pt = np.array([msg.point.x, msg.point.y, msg.point.z]).reshape((3, 1))
        out_pt = np.matmul(self.H, in_pt)
        self.publish_data(out_pt) 
    
class Transformer_Local(object):

    def __init__(self):
        self.pub = rospy.Publisher('/object/position_local', PointStamped, queue_size=1)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()

    def publish_data(self, pt):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = pt[0]
        self.msg.point.y = pt[1]
        self.msg.point.z = pt[2]
        self.pub.publish(self.msg)

    def callback(self, msg):
        in_pt = np.array([msg.point.x - 320, 240 - msg.point.y, msg.point.z - 250])
        self.publish_data(in_pt) 
    
    
def transformer():

    rospy.init_node('position_transform', anonymous=True)
    
    tf = Transformer_Local()

    rospy.Subscriber('/object/position', PointStamped, tf.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    transformer()
