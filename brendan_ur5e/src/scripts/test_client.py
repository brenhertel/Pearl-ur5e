#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from brendan_ur5e.srv import shutdown_request

def shutdown_self_client(req):
	rospy.wait_for_service('shutdown_self')
	try:
		shutdown_self_call = rospy.ServiceProxy('shutdown_self', shutdown_request)
		shutdown_self_call(req)
		return
	except rospy.ServiceException, e:
		print("Service call failed: %s", e)

if __name__ == "__main__":
	print (sys.argv[1])
	shutdown_self_client(int(sys.argv[1]))
