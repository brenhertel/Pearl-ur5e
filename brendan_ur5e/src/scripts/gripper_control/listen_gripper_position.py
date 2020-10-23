#!/usr/bin/env python

#https://dof.robotiq.com/discussion/1649/control-gripper-via-ur-controller-client-interface
#got from link above
# Echo client program
import socket
import time
import binascii
from ast import literal_eval
import rospy
from std_msgs.msg import Int32

s = None
c = None
s_self = None

def myhook():
    s.close()
    c.close()
    s_self.close()


def set_position(data, c):
    #bin_data = '{0:32b}'.format(data.data)
    #bin_data = format(data.data, "08b")
    rospy.loginfo('data recieved ' + str(data.data))
    c.send(chr(data.data))

def listener(c):
    print('Connected!')
    rospy.loginfo('listening to /gripper_sends/position')
    rospy.Subscriber('/gripper_sends/position', Int32, set_position, c)
    rospy.init_node('gripper_pose_sub', anonymous=True)
    rospy.on_shutdown(myhook)
    
    rospy.spin()
    

if __name__ == '__main__':
    HOST = "172.16.32.67" # The UR IP address
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    #HOST_self = "127.0.0.1" # own IP address
    #PORT_self = 80 # connection port
    s_self = socket.socket()
    bound = 0
    while not bound:
    	try:
    	    s_self.bind(('', 32345))
    	    bound = 1
    	except:
    	    print('Could not bind to port, retrying')
    	    rospy.loginfo('Could not bind to port, retrying')
    	    rospy.sleep(0.1)
    	    bound = 0
    		
    s_self.listen(5)
    
    f = open ("/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/gripper_control/get_pose_from_comp.script", "rb")   #Robotiq Gripper
    #f = open ("setzero.script", "rb")  #Robotiq FT sensor
    
    ln = f.read(1024)
    while (ln):
        s.send(ln)
        ln = f.read(1024)
    
    c = None
        
    while not c:
        print('trying...')
        c, addr = s_self.accept()
    try:
        listener(c)
    except rospy.ROSInterruptException:
        s.close()
        c.close()
        s_self.close()
    


