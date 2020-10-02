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

def talker(c):
    pub = rospy.Publisher('/gripper_data/speed', Int32, queue_size=100)
    rospy.init_node('gripper_pose_pub', anonymous=True)
    while not rospy.is_shutdown():
        ret = c.recv(1024)
        bit_string = bin(int(binascii.hexlify(ret), 16))
        int_conv = int(literal_eval(bit_string))
        #print(int_conv)
        pub.publish(int_conv)
    

if __name__ == '__main__':
    HOST = "172.16.32.67" # The UR IP address
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    #HOST_self = "127.0.0.1" # own IP address
    #PORT_self = 80 # connection port
    s_self = socket.socket()
    s_self.bind(('', 22345))
    s_self.listen(5)
    
    f = open ("/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/gripper_control/send_pose_to_comp.script", "rb")   #Robotiq Gripper
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
        talker(c)
    except rospy.ROSInterruptException:
        s.close()
        c.close()
        pass
    


