#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from uavcan_msgs.msg import Fix
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
        self.joy = Joy()
        self.joy.axes = [0] * 9

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

    def actuators_callback(self, actuators):
        try:
            if len(actuators.axes) < 4:
                rospy.logerr_throttle(5, "InnosimUavcanBridge: wrong actuators msg len(axes) < 4")
                return
            elif len(actuators.axes) != 8:
                rospy.logwarn_throttle(5, "InnosimUavcanBridge: wrong airframe. Let it be MC.")
        except:
            rospy.logerr_throttle(5, "InnosimUavcanBridge: wrong actuators msg. There is no axes")
            return

        MIXER_MIN_MAX = [
            (0, 1), # 0. FR is normalized into [0, +1], where 0 means turn off
            (0, 1), # 1. RL is normalized into [0, +1], where 0 means turn off
            (0, 1), # 2. FL motor is normalized into [0, +1], where 0 means turn off
            (0, 1), # 3. RR motor is normalized into [0, +1], where 0 means turn off
            (0, 1), # 4. Aileron is normalized into [0, +1], where 0.5 is a middle position
            (-1, 1),# 5. Elevator is normalized into [-1, +1], where 0 is a middle position
            (-1, 1),# 6. Rudder is normalized into [-1, +1], where 0 is a middle position
            (0, 1), # 7. Pusher is normalized into [0, +1], where 0 means turn off
        ]

        # 1. Clamp input data and convert it to the px4 control group format (from -1 to +1)
        mixer = [0] * min(len(MIXER_MIN_MAX), len(actuators.axes))
        for idx in range(len(mixer)):
            mixer[idx] = max(MIXER_MIN_MAX[idx][0], min(actuators.axes[idx], MIXER_MIN_MAX[idx][1]))
        if len(mixer) == 8:
            mixer[4] = (mixer[4] - 0.5) * 2

        # 2. Convert them to the sim format
        SIM_MAX_VALUES = [
            15500,  # FR, cw, rate, rpm (Front right motor speed)
            15500,  # RL, cw, rate, rpm (Rear left motor speed)
            15500,  # FL, ccw, rate, rpm (Front left motor speed)
            15500,  # RR, ccw, rate, rpm (Rear right motor speed)
            70,     # aileron left, cw, deg
            70,     # aileron right, cw, deg
            70,     # elevator, cw, deg
            70,     # rudder, cw, deg
            15500   # thrust, pusher, rate, rpm
        ]

        for idx in range(min(4, len(mixer))):
            self.joy.axes[idx] = mixer[idx] * SIM_MAX_VALUES[idx]
        if len(mixer) == 8:
            self.joy.axes[4] = mixer[4] * SIM_MAX_VALUES[4]
            self.joy.axes[5] = -mixer[4] * SIM_MAX_VALUES[5]
            self.joy.axes[6] = mixer[5] * SIM_MAX_VALUES[6]
            self.joy.axes[7] = mixer[6] * SIM_MAX_VALUES[7]
            self.joy.axes[8] = mixer[7] * SIM_MAX_VALUES[8]

        self.joy_pub.publish(self.joy)


if __name__ == '__main__':
    try:
        rospy.init_node('innosim_relay', anonymous=True)
        inno_sim_bridge = InnoSimBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

