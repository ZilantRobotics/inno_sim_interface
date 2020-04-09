#!/usr/bin/env python

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCOut
from sensor_msgs.msg import Joy

attitude_pub = rospy.Publisher('/sim/attitude', QuaternionStamped, queue_size = 1)
joy_pub = rospy.Publisher('/sim/actuators', Joy, queue_size = 1)

def odom_callback(data):
    q = QuaternionStamped()
    q.header = data.header
    q.quaternion = data.pose.pose.orientation
    attitude_pub.publish(q)

def rc_callback(data):
    # PX4 SITL VTOL channels:
    # 1 FR, ccw (900 - 2000)
    # 2 RL, ccw
    # 3 FL, cw
    # 4 RR, cw (1000 - 2000)
    # 5 pusher (1000 - 2000)
    # 6 aileron (1000 - 2000)
    # 7 aileron
    # 8 elevator
    FR_pwm = data.channels[0]
    RL_pwm = data.channels[1]
    FL_pwm = data.channels[2]
    RR_pwm = data.channels[3]
    pusher_pwm = data.channels[4]
    aileron1_pwm = data.channels[5]
    aileron2_pwm = data.channels[6]
    elevator_pwm = data.channels[7]


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
    joy.axes.append( round( 15500*(FR_pwm - 900.0)/1100.0, 1) )
    joy.axes.append( round( 15500*(RL_pwm - 900.0)/1100.0, 1) )
    joy.axes.append( round( 15500*(FL_pwm - 900.0)/1100.0, 1) )
    joy.axes.append( round( 15500*(RR_pwm - 900.0)/1100.0, 1) )
    joy.axes.append( round( 70*(aileron1_pwm - 1500.0)/500.0, 1) )
    joy.axes.append( round( 70*(aileron2_pwm - 1500.0)/500.0, 1) )
    joy.axes.append( round( 70*(elevator_pwm - 1500.0)/500.0, 1) )
    joy.axes.append( round( 0.0, 1) )
    joy.axes.append( round( 15500*(pusher_pwm - 1000.0)/1000.0, 1) )
    joy_pub.publish(joy)


def innosim_relay():
    rospy.init_node('innosim_relay', anonymous=True)
    rospy.Subscriber('/mavros/global_position/local', Odometry, odom_callback)
    rospy.Subscriber('/mavros/rc/out', RCOut, rc_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        innosim_relay()
    except rospy.ROSInterruptException:
        pass

