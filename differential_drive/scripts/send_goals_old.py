#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# Function to create and send a goal
def send_goal(x, y):
    rospy.loginfo("Sending goal to position: ({}, {})".format(x, y))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "slamware_map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    ac.send_goal(goal)
    ac.wait_for_result()

# Initialize the ROS node
rospy.init_node('sequential_goal_sender')

# Create a SimpleActionClient for move_base
ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
ac.wait_for_server()

# Define the list of goal positions
goal_positions = [
    (16.8, -1.00),  # Bathroom
    (16.4, -15.86),   # Bedroom
    (-0.35, -16.3),  # Front Door
    (-0.3, 0.5)    # Living Room
]

# Send goals sequentially
for position in goal_positions:
    send_goal(position[0], position[1])

rospy.loginfo("Reached all goal positions")

