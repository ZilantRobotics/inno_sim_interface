#!/usr/bin/env python

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry

attitude_pub = rospy.Publisher('/sim/attitude', QuaternionStamped, queue_size = 1)

def odom_callback(data):
    q = QuaternionStamped()
    q.header = data.header
    q.quaternion = data.pose.pose.orientation
    attitude_pub.publish(q)


def innosim_relay():
    rospy.init_node('innosim_relay', anonymous=True)
    rospy.Subscriber('/mavros/global_position/local', Odometry, odom_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        innosim_relay()
    except rospy.ROSInterruptException:
        pass
