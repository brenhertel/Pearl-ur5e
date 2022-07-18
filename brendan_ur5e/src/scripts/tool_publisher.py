#!/usr/bin/env python

import numpy as np
import rospy
from tf.msg import tfMessage
import std_msgs
import moveit_commander
import copy
import sys

    
class Transformer(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.pub0 = rospy.Publisher('/tool0', tfMessage, queue_size=100)
        self.pub1 = rospy.Publisher('/tool1', tfMessage, queue_size=100)
        
    def get_tool0_position(self):
        tool = self.robot.get_link("tool0").pose()
        return tool.pose.position.x, tool.pose.position.y, tool.pose.position.z
        
    def get_tool0_orientation(self):
        tool = self.robot.get_link("tool0").pose()
        return tool.pose.orientation.x, tool.pose.orientation.y, tool.pose.orientation.z, tool.pose.orientation.w

    def get_tool1_position(self):
        left_pad = self.robot.get_link("left_inner_finger_pad").pose()
        right_pad = self.robot.get_link("right_inner_finger_pad").pose()
        return (left_pad.pose.position.x + right_pad.pose.position.x) / 2, (left_pad.pose.position.y + right_pad.pose.position.y) / 2, (left_pad.pose.position.z + right_pad.pose.position.z) / 2

    def publish_data0(self, msg):
        x, y, z = self.get_tool0_position()
        msg.transforms[0].transform.translation.x = x
        msg.transforms[0].transform.translation.y = y
        msg.transforms[0].transform.translation.z = z
        x, y, z, w = self.get_tool0_orientation()
        msg.transforms[0].transform.rotation.x = x
        msg.transforms[0].transform.rotation.y = y
        msg.transforms[0].transform.rotation.z = z
        msg.transforms[0].transform.rotation.w = w
        self.pub0.publish(msg)
        
    def publish_data1(self, msg):
        x, y, z = self.get_tool1_position()
        msg.transforms[0].transform.translation.x = x
        msg.transforms[0].transform.translation.y = y
        msg.transforms[0].transform.translation.z = z
        x, y, z, w = self.get_tool0_orientation()
        msg.transforms[0].transform.rotation.x = x
        msg.transforms[0].transform.rotation.y = y
        msg.transforms[0].transform.rotation.z = z
        msg.transforms[0].transform.rotation.w = w
        self.pub1.publish(msg)

    def callback(self, msg):
        if msg.transforms[0].child_frame_id == 'tool0_controller':
            self.publish_data0(copy.copy(msg))
            self.publish_data1(copy.copy(msg))
    
def transformer():

    rospy.init_node('tool_transform', anonymous=True)
    
    tf = Transformer()

    rospy.Subscriber('/tf', tfMessage, tf.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    transformer()
