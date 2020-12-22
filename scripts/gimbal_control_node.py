#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from sensor_msgs.msg import CameraInfo

# https://github.com/carla-simulator/scenario_runner/blob/master/srunner/challenge/autoagents/ros_agent.py
def build_camera_info(attributes):
    """
    Private function to compute camera info
    camera info doesn't change over time
    """
    camera_info = CameraInfo()
    # store info without header
    camera_info.header.frame_id = "velodyne"
    camera_info.width = int(attributes['width'])
    camera_info.height = int(attributes['height'])
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = camera_info.width / (
        2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
    fy = fx
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.D = [0, 0, 0, 0, 0]
    camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
    return camera_info

if __name__ == '__main__':
    rospy.init_node('gimbal_control')

    listener = tf.TransformListener()

    gimbal_angle_pub = rospy.Publisher('/sim/gimbal_angle', geometry_msgs.msg.Vector3Stamped, queue_size=1)
    camera_info_pub = rospy.Publisher('/sim/camera_info', CameraInfo, queue_size=1)

    rate = rospy.Rate(100.0)

    br = tf.TransformBroadcaster()
    attributes = dict()
    attributes['width'] = 1920
    attributes['height'] = 1080
    attributes['fov'] = 26.9914
    camera_info = build_camera_info(attributes)
    
    while not rospy.is_shutdown():
        try:
            # listener.waitForTransform('/map', '/base_link', rospy.Time.now(), rospy.Duration(2.0))
            now = rospy.Time.now()
            # print('wait transform')
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            # print('got transform map -> baselink')
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

            msg = geometry_msgs.msg.Vector3Stamped()
            msg.header.stamp = now

            msg.vector.x = 0.0 # math.degrees(roll)
            msg.vector.y = 90.0 # math.degrees(pitch)
            msg.vector.z = 90 - math.degrees(yaw)

            gimbal_angle_pub.publish(msg)

            br.sendTransform(trans,
                     tf.transformations.quaternion_from_euler(0, math.radians(90), yaw),
                     now,
                     "gimbal",
                     "map")

        except (tf.Exception):
            continue

        rate.sleep()

        camera_info.header.stamp = rospy.Time.now()
        camera_info_pub.publish(camera_info)
