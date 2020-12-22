#!/usr/bin/env python

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCOut
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
import tf2_ros
import tf2_msgs
import geometry_msgs

attitude_pub = rospy.Publisher('/sim/attitude', QuaternionStamped, queue_size = 1)
joy_pub = rospy.Publisher('/sim/actuators', Joy, queue_size = 1)

gps_pub = rospy.Publisher('/sim/gps_position', NavSatFix, queue_size = 1)


# Here is some magic needed to give global altitude to simulator.
# GLOBAL_POSITION_INT (/mavros/global_position/global) has altitude relative to starting point. 
# GPS_RAW_INT (/mavros/global_position/raw/fix) has global altitude (we could tune it with `export PX4_HOME_ALT`), but publishing rate is low.
# So, we apply altitude correction in GLOBAL_POSITION_INT and send to sim.
global raw_alt, alt_init, alt_corr, has_raw_alt
raw_alt = 0.0
alt_init = False
alt_corr = 0.0
has_raw_alt = False

def global_gps_callback(data):
    global raw_alt, alt_init, alt_corr, has_raw_alt
    
    if alt_init:
        data.altitude += alt_corr
        gps_pub.publish(data)
    else:
        if has_raw_alt:
            alt_corr = raw_alt - data.altitude
            alt_init = True

def raw_gps_callback(data):
    global raw_alt, has_raw_alt

    raw_alt = data.altitude
    has_raw_alt = True

def odom_callback(data):
    q = QuaternionStamped()
    q.header = data.header
    q.quaternion = data.pose.pose.orientation
    attitude_pub.publish(q)

    # print(q.quaternion)
    # print(data.pose.pose)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
  
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation = data.pose.pose.position
    # t.transform.translation.y = msg.y
    # t.transform.translation.z = 0.0
    t.transform.rotation = data.pose.pose.orientation
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]
   
    br.sendTransform(t)

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

    FR_res = 15500*(FR_pwm - 900.0)/1100.0
    RL_res = 15500*(RL_pwm - 900.0)/1100.0
    FL_res = 15500*(FL_pwm - 900.0)/1100.0
    RR_res = 15500*(RR_pwm - 900.0)/1100.0
    aileron1_res = 70*(aileron1_pwm - 1500.0)/500.0
    aileron2_res = 70*(aileron2_pwm - 1500.0)/500.0
    elevator_res = 70*(elevator_pwm - 1500.0)/500.0
    rudder_res = 0.0
    thrust_res = 15500*(pusher_pwm - 1000.0)/1000.0

    joy.axes.append( round( FR_res, 1) )
    joy.axes.append( round( RL_res, 1) )
    joy.axes.append( round( FL_res, 1) )
    joy.axes.append( round( RR_res, 1) )
    joy.axes.append( round( aileron1_res, 1) )
    joy.axes.append( round( aileron2_res, 1 ) )
    joy.axes.append( round( elevator_res, 1) )
    joy.axes.append( round( rudder_res, 1) )
    joy.axes.append( round( thrust_res, 1) )

    for i in range(4):
        if joy.axes[i] < 0:
            joy.axes[i] = 0
            
    if joy.axes[8] < 0:
        joy.axes[8] = 0

    joy_pub.publish(joy)


def innosim_relay():
    rospy.init_node('innosim_relay', anonymous=True)
    rospy.Subscriber('/mavros/global_position/local', Odometry, odom_callback)
    rospy.Subscriber('/mavros/rc/out', RCOut, rc_callback)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_gps_callback)
    rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, raw_gps_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        innosim_relay()
    except rospy.ROSInterruptException:
        pass

