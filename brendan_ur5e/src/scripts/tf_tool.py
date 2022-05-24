import numpy as np
import rospy
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
#using https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation for reference
class Transformer(object):

    def __init__(self):
        self.pub = rospy.Publisher('/tool1', tfMessage, queue_size=10)
        self.x_trans = 0.185 #m

    def callback(self, msg):
        if msg.transforms[0].child_frame_id == 'tool0_controller':
            orientation_q = msg.transforms[0].transform.rotation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            print([roll, pitch, yaw])
            
            x = msg.transforms[0].transform.translation.x
            y = msg.transforms[0].transform.translation.y
            z = msg.transforms[0].transform.translation.z
            
            qi = -msg.transforms[0].transform.rotation.x
            qj = msg.transforms[0].transform.rotation.y
            qk = msg.transforms[0].transform.rotation.z
            qr = msg.transforms[0].transform.rotation.w
            
            #print(euler_from_quaternion(qi, qj, qk, qr))
            
            a = 1 - (2*(qj**2 + qk**2))
            b = 2*(qi*qj - qk*qr)
            c = 2*(qi*qk + qj*qr)
            d = 2*(qi*qj + qk*qr)
            e = 1 - (2*(qi**2 + qk**2))
            f = 2*(qj*qk - qi*qr)
            g = 2*(qi*qk - qj*qr)
            h = 2*(qj*qk + qi*qr)
            i = 1 - (2*(qi**2 + qj**2))
            
            xp = x - h * self.x_trans
            yp = y - b * self.x_trans
            zp = z + e * self.x_trans
            
            new_msg = msg
            new_msg.transforms[0].transform.translation.x = xp
            new_msg.transforms[0].transform.translation.y = yp
            new_msg.transforms[0].transform.translation.z = zp
            
            self.pub.publish(new_msg)
        
def transformer():

    rospy.init_node('tool_transformer', anonymous=True)
    
    trans = Transformer()
    
    rospy.Subscriber('/tf', tfMessage, trans.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    transformer()
    
#def main():
#    qx = 0.7071
#    qy = 0.7071
#    qz = 0.
#    qw = 0.
#    
#    x = 0.
#    y = 0.
#    z = 0.
#    
#    x_trans = 1.
#    
#    a = 1 - (2*(qy**2 + qz**2))
#    d =2*(qx*qy + qz*qw)
#    g =2*(qx*qz + qy*qw)
#    
#    xp = x + a * x_trans
#    yp = y + d * x_trans
#    zp = z + z * x_trans
#    
#    print([xp, yp, zp])
#
#if __name__ == '__main__':
#    main()
