import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
import std_msgs


class Localizer(object):

    def __init__(self):
        self.alpha = 0.25
        self.pub = rospy.Publisher('/object/position_local', PointStamped, queue_size=1)
        self.h = std_msgs.msg.Header()
        self.msg = PointStamped()
        self.local_x = None
        self.local_y = None

    def publish_data(self, x, y):
        self.h.stamp = rospy.Time.now()
        self.msg.header = self.h
        self.msg.point.x = x
        self.msg.point.y = y
        self.msg.point.z = 1.0
        self.pub.publish(self.msg)

    def callback(self, msg):
        self.local_x = ((1 - self.alpha) * msg.point.x) + (self.alpha * self.local_x) if self.local_x is not None else msg.point.x
        self.local_y = ((1 - self.alpha) * msg.point.y) + (self.alpha * self.local_y) if self.local_y is not None else msg.point.y
        self.publish_data(self.local_x, self.local_y)
    
def localizer():

    rospy.init_node('position_localizer', anonymous=True)
    
    loc = Localizer()

    rospy.Subscriber('/object/position_tf', PointStamped, loc.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    localizer()
