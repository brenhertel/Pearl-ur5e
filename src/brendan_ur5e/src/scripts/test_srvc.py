#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from brendan_ur5e.srv import shutdown_request

def shutdown_self(req):
	rospy.loginfo("Still Waiting")
	if req.a == 1:
		rospy.loginfo("recieved request to shutdown")
		rospy.signal_shutdown("Because I was told to")

def shutdown_self_server():
	rospy.init_node('shutdown_self_server')
	s = rospy.Service('shutdown_self', shutdown_request, shutdown_self)
	print("Waiting to shutdown")
	rospy.spin()

if __name__ == "__main__":
	shutdown_self_server()
