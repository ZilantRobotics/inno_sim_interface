#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix, Joy
from geometry_msgs.msg import QuaternionStamped
import time

# Define publish rate limit (Hz)
PUBLISH_RATE_HZ = 10
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ

class DynamicsTo3DRelay:
    def __init__(self):
        rospy.init_node('dynamics_to_3d_relay', anonymous=True)

        # ROS topic subscribers
        rospy.Subscriber('/uav/gps_point', NavSatFix, self.callback_gps_point)
        rospy.Subscriber('/uav/attitude', QuaternionStamped, self.callback_attitude)
        rospy.Subscriber('/uav/actuators', Joy, self.callback_actuators)
        
        # ROS topic publishers
        self.publisher_gps_position = rospy.Publisher('/sim/gps_position', NavSatFix, queue_size=10)
        self.publisher_attitude = rospy.Publisher('/sim/attitude', QuaternionStamped, queue_size=10)
        self.publisher_actuators = rospy.Publisher('/sim/actuators', Joy, queue_size=10)

        # Time variables for controlling publish rate
        self.last_publish_time_gps = time.time()
        self.last_publish_time_attitude = time.time()
        self.last_publish_time_actuators = time.time()

    def callback_gps_point(self, data):
        if time.time() - self.last_publish_time_gps >= PUBLISH_PERIOD_SEC:
            self.publisher_gps_position.publish(data)
            self.last_publish_time_gps = time.time()

    def callback_attitude(self, data):
        if time.time() - self.last_publish_time_attitude >= PUBLISH_PERIOD_SEC:
            self.publisher_attitude.publish(data)
            self.last_publish_time_attitude = time.time()

    def callback_actuators(self, data):
        if time.time() - self.last_publish_time_actuators >= PUBLISH_PERIOD_SEC:
            self.publisher_actuators.publish(data)
            self.last_publish_time_actuators = time.time()

def main():
    try:
        relay = DynamicsTo3DRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
