#! /usr/bin/env python
# shim to republish the odom message as a pose message (to make husky appear like anymal)
import time
import rospy

# ROS messages
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def on_odom_msg(data):
    m = PoseWithCovarianceStamped()
    m.header = data.header
    m.pose =  data.pose
    pose_pub.publish(m)


rospy.init_node('husky_odom_to_pose_shim', anonymous=True)
pose_pub = rospy.Publisher('/state_estimator/pose_in_odom', PoseWithCovarianceStamped, queue_size=10)
print "husky_odom_to_pose_shim started"

rospy.Subscriber("/odometry/filtered", Odometry, on_odom_msg)

rospy.spin()
