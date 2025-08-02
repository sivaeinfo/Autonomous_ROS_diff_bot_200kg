#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion

# Function to create and send a goal
def send_goal(x, y, theta):
    rospy.loginfo("Sending goal to position: ({}, {}) with orientation: {}".format(x, y, theta))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "slamware_map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert angle (theta) to quaternion
    quaternion = quaternion_from_yaw(theta)
    goal.target_pose.pose.orientation = quaternion

    ac.send_goal(goal)
    ac.wait_for_result()

# Function to convert yaw angle to quaternion
def quaternion_from_yaw(yaw):
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(yaw / 2)
    quaternion.w = cos(yaw / 2)
    return quaternion

# Initialize the ROS node
rospy.init_node('sequential_goal_sender')

# Create a SimpleActionClient for move_base
ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
ac.wait_for_server()

# Define the list of goal positions and orientations
goal_positions = [
    (14.8, 0.32, -0.3),     # Goal 1
    (15.5, -13.89, -1.8),    # Goal 2
    (1.504, -15.2, 2.7),     # Goal 3
    (-0.063, -0.08, 1.5)        # Goal 4
]

#goal_positions = [
#    (17.8, -0.8, -0.3),     # Goal 1
#    (16.5, -16.65, -1.8),    # Goal 2
#    (-0.35, -16.3, 2.7),     # Goal 3
#    (-0.55, -1.5, 1.5)        # Goal 4
#]

# Send goals sequentially
for position in goal_positions:
    send_goal(position[0], position[1], position[2])

rospy.loginfo("Reached all goal positions")

