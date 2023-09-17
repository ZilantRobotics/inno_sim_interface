import rospy
import pyproj
import utm
from geometry_msgs.msg import PoseStamped, QuaternionStamped, Quaternion, Pose, Twist, Vector3
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# Initialize node
rospy.init_node('gps_attitude_to_pose')

# Set up publishers and subscribers
pose_pub = rospy.Publisher('/drone_pose', PoseStamped, queue_size=1)
odom_pub = rospy.Publisher('/drone_odom', Odometry, queue_size=1)
attitude = Quaternion()
position = NavSatFix()

# Variables to hold the origin of the local frame (first received GPS data)
origin_lat = None
origin_lon = None
origin_east = None
origin_north = None
origin_zone = None
origin_letter = None

def attitude_callback(msg):
    global attitude
    attitude = msg.quaternion

def gps_callback(msg):
    global position, origin_lat, origin_lon, origin_east, origin_north, origin_zone, origin_letter
    position = msg

    if origin_lat is None and origin_lon is None:
        # Store the first received GPS data as the origin of our local frame
        origin_lat = position.latitude
        origin_lon = position.longitude
        origin_east, origin_north, origin_zone, origin_letter = utm.from_latlon(origin_lat, origin_lon)

    else:
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "world"

        # Convert from LLA to UTM
        east, north, _, _ = utm.from_latlon(position.latitude, position.longitude, force_zone_number=origin_zone, force_zone_letter=origin_letter)

        # Subtract the first position in UTM format
        pose.pose.position.x = east - origin_east
        pose.pose.position.y = north - origin_north
        pose.pose.position.z = position.altitude 

        # Orientation
        pose.pose.orientation = attitude

        # Publishing the PoseStamped message
        pose_pub.publish(pose)

        # Creating and publishing the Odometry message
        odom_msg = Odometry()
        odom_msg.header = pose.header
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose.pose
        odom_msg.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))  # Assuming velocities are zero
        odom_pub.publish(odom_msg)

attitude_sub = rospy.Subscriber('/sim/attitude', QuaternionStamped, attitude_callback)
gps_sub = rospy.Subscriber('/sim/gps_position', NavSatFix, gps_callback)

rate = rospy.Rate(10) # 10 Hz

while not rospy.is_shutdown():
    rate.sleep()
