#!/usr/bin/env python 

import roslib
roslib.load_manifest('brendan_ur5e')
import actionlib
import os
import rospy
from brendan_ur5e.action import gripper_status_action
import socket
import time
import binascii
from ast import literal_eval
import rospy
from std_msgs.msg import Int32
from brendan_ur5e.msg import gripper_pos
import std_msgs

class gripper_action(object):

    def __init__(self):
        self._as = actionlib.SimpleActionServer('gripper_status_handler', gripper_status_action, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
      
     ### Gripper Playback ###      
     def myhook(self):
    	self.s.close()
    	self.c.close()
    	self.s_self.close()

    def set_position(self, data):
        #bin_data = '{0:32b}'.format(data.data)
        #bin_data = format(data.data, "08b")
        rospy.loginfo('data recieved ' + str(data.data))
        self.c.send(chr(data.data))

    def listener(self):
        print('Connected!')
        self._as.publish_feedback(True)
        rospy.loginfo('listening to /gripper_sends/position')
        rospy.Subscriber('/gripper_sends/position', Int32, self.set_position, self.c)
        #rospy.init_node('gripper_pose_sub', anonymous=True)
        rospy.on_shutdown(myhook)
        rospy.spin()

    def gripper_playback(self):
        HOST = "172.16.32.67" # The UR IP address
        PORT = 30002 # UR secondary client
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, PORT))
        
        self.s_self = socket.socket()
        bound = 0
        while not bound:
        	try:
        	    self.s_self.bind(('', 32345))
        	    bound = 1
        	except:
        	    print('Could not bind to port, retrying')
        	    rospy.loginfo('Could not bind to port, retrying')
        	    rospy.sleep(0.1)
        	    bound = 0
    		
        self.s_self.listen(5)
    
        f = open ("/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/gripper_control/get_pose_from_comp.script", "rb")   #Robotiq Gripper
        ln = f.read(1024)
        while (ln):
          self.s.send(ln)
          ln = f.read(1024)
    
        self.c = None
        while not self.c:
            print('trying...')
            self.c, addr = self.s_self.accept()
        try:
            self.listener()
        except rospy.ROSInterruptException:
            self.s.close()
            self.c.close()
            self.s_self.close() 
      
    ### Gripper Record ###
    def talker(self):
        print('Connected!')
        pub = rospy.Publisher('/gripper_data/position', gripper_pos, queue_size=100)
        #rospy.init_node('gripper_pose_pub', anonymous=True)
        while not rospy.is_shutdown():
            self._as.publish_feedback(True)
            ret = self.c.recv(1024)
            if ret:
                bit_string = bin(int(binascii.hexlify(ret), 16))
       	        int_conv = int(literal_eval(bit_string))
                #print(int_conv)
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                msg = gripper_pos()
                msg.header = h
                msg.gripper_pos = int_conv
                pub.publish(msg)
    
    def gripper_record(self):
        HOST = "172.16.32.67" # The UR IP address
        PORT = 30002 # UR secondary client
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, PORT))
        self.s_self = socket.socket()
        self.s_self.bind(('', 12345))
        self.s_self.listen(5)
    
        f = open ("/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/gripper_control/send_pose_to_comp.script", "rb")   #Robotiq Gripper
        ln = f.read(1024)
        while (ln):
            self.s.send(ln)
            ln = f.read(1024)
    
        self.c = None
        while not self.c:
            print('trying...')
            self.c, addr = self.s_self.accept()
        try:
            self.talker()
        except rospy.ROSInterruptException:
            self.s.close()
            self.c.close()
            self.s_self.close() 
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
        if success:
            self._result.sequence = self._feedback.sequence
            self._as.set_succeeded(self._result)

#def handle_gripper_srv(req): 
#    print ('Type = ' + str(req.type))
#    if (req.type == 0): #record
#    	os.system('rosrun brendan_ur5e publish_gripper_position.py')
#    elif (req.type == 1): #playback
#    	os.system('rosrun brendan_ur5e listen_gripper_position.py')
#    else:
#    	print('Unknown Type')
#    	return -1
#    while not (rospy.get_param('gripper_status')):
#    	rospy.sleep(0.001)
#    	print(rospy.get_param('gripper_status'))
#    return 1

def gripper_handler_server(): 
    rospy.init_node('gripper_request_handler', anonymous=True) 
    #s = rospy.Service('gripper_data_request', gripper_data_request, handle_gripper_srv) 
    server = gripper_action()
    print('Ready to handle gripper requests')
    rospy.spin() 

if __name__ == "__main__": 
    gripper_handler_server()
