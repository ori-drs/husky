#! /usr/bin/env python
# shim to republish the odom message as a pose message (to make husky appear like anymal)
import time
import rospy

# ROS messages
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState

def on_joint_state_msg(data):
    m = JointState()
    m.header.stamp = data.header.stamp
    m.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
    m.position = [0,0,0,0]
    m.velocity = [0,0,0,0]
    m.effort = [0,0,0,0]

    pose_pub.publish(m)


rospy.init_node('husky_cdt_wheel_angle_shim', anonymous=True)
pose_pub = rospy.Publisher('/husky/joint_states', JointState, queue_size=10)
print "husky_cdt_wheel_angle_shim started"

rospy.Subscriber("/multisense/joint_states", JointState, on_joint_state_msg)

rospy.spin()
