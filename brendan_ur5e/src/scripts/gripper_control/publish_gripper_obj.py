#!/usr/bin/env python

#https://dof.robotiq.com/discussion/1649/control-gripper-via-ur-controller-client-interface
#got from link above
# Echo client program
import socket
import time
import binascii
from ast import literal_eval
import rospy
from brendan_ur5e.msg import gripper_pos
import std_msgs
import os

def talker(c):
    print('Connected!')
    pub = rospy.Publisher('/gripper_data/obj', gripper_pos, queue_size=100)
    rospy.init_node('gripper_obj_pub', anonymous=True)
    while not rospy.is_shutdown():
        ret = c.recv(1024)
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
    

if __name__ == '__main__':
    HOST = "172.16.32.67" # The UR IP address
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    #HOST_self = "127.0.0.1" # own IP address
    #PORT_self = 80 # connection port
    s_self = socket.socket()
    s_self.bind(('', 12345))
    s_self.listen(5)
    
    f = open ("/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/gripper_control/send_obj_to_comp.script", "rb")   #Robotiq Gripper
    #f = open ("setzero.script", "rb")  #Robotiq FT sensor
    
    ln = f.read(1024)
    while (ln):
        s.send(ln)
        ln = f.read(1024)
    
    c = None
    os.system("echo Hello from OS1")
    while not c:
        print('trying...')
        c, addr = s_self.accept()
    try:
        talker(c)
    except:
        s.close()
        c.close()
        os.system("echo -e 'def test(): \n freedrive_mode() \n end_freedrive_mode() \nend \n' | telnet 172.16.32.67 30002")
        pass
    


