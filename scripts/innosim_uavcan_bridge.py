#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from drone_communicators.msg import Fix
from pyquaternion import Quaternion

FRD_FLU = Quaternion(w=0, x=1, y=0, z=0)
NED_TO_ENU = Quaternion(w=0, x=0.70711, y=0.70711, z=0)

class InnoSimBridge:
    def __init__(self):
        self.attitude_pub = rospy.Publisher('/sim/attitude', QuaternionStamped, queue_size = 1)
        self.joy_pub = rospy.Publisher('/sim/actuators', Joy, queue_size = 1)
        self.gps_pub = rospy.Publisher('/sim/gps_position', NavSatFix, queue_size = 1)

        rospy.Subscriber('/uav/gps_position', Fix, self.gps_callback)
        rospy.Subscriber('/uav/attitude', QuaternionStamped, self.attitude_callback)
        rospy.Subscriber('/uav/actuators', Joy, self.actuators_callback)

        self.gps_msg = NavSatFix()

    def gps_callback(self, in_msg):
        self.gps_msg.header.stamp = in_msg.header.stamp
        self.gps_msg.latitude = in_msg.latitude_deg_1e8 * 1e-08
        self.gps_msg.longitude = in_msg.longitude_deg_1e8 * 1e-08
        self.gps_msg.altitude = in_msg.height_msl_mm * 1e-03 + 6.5
        self.gps_pub.publish(self.gps_msg)

    def attitude_callback(self, msg):
        # We receive altitude in PX4 notation, but we need to publish in ROS notation
        # So, at first we convert NED to baselink
        # After that we convert baselink to ENU
        q = Quaternion(w=msg.quaternion.w,
                    x=msg.quaternion.x,
                    y=msg.quaternion.y,
                    z=msg.quaternion.z)

        q = NED_TO_ENU * q * FRD_FLU

        msg.quaternion.w = q.w
        msg.quaternion.x = q.x
        msg.quaternion.y = q.y
        msg.quaternion.z = q.z

        self.attitude_pub.publish(msg)

    def actuators_callback(self, data):
        # PX4 UAVCAN VTOL channels:
        # 1 FR, ccw (0 - 8192)
        # 2 RL, ccw
        # 3 FL, cw
        # 4 RR, cw
        # 5 aileron (1000 - 2000)
        # 6 elevator (0 -> 4096 -> 8192)
        # 7 rudder
        # 8 pusher (0 -> 8192)
        FR_pwm = data.axes[0]
        RL_pwm = data.axes[1]
        FL_pwm = data.axes[2]
        RR_pwm = data.axes[3]
        pusher_pwm = data.axes[4]
        aileron1_pwm = data.axes[5]
        aileron2_pwm = data.axes[6]
        elevator_pwm = data.axes[7]

        joy = Joy()
        # Sim:
        # FR, cw, rate, rpm (Front right motor speed)
        # RL, cw, rate, rpm (Rear left motor speed)
        # FL, ccw, rate, rpm (Front left motor speed)
        # RR, ccw, rate, rpm (Rear right motor speed)
        # aileron left, cw, deg
        # aileron right, cw, deg
        # elevator, cw, deg
        # rudder, cw, deg
        # thrust, pusher, rate, rpm


if __name__ == '__main__':
    try:
        rospy.init_node('innosim_relay', anonymous=True)
        inno_sim_bridge = InnoSimBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

