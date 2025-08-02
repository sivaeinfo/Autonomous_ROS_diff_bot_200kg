#!/usr/bin/env python3

from sensor_msgs.msg import Imu

import rospy
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose
from tf.transformations import euler_from_quaternion

import math

# Constants - Adjust these according to your specific setup
WHEEL_CIRCUMFERENCE = 0.502655  # in meters (example value)
GEAR_RATIO = 100.0  # Ratio between the motor and the wheel
WHEEL_BASE = 0.44  # Distance between the left and right wheels in meters

odometer_x = 0.0
odometer_y = 0.0
previous_time = None

# Motor directions
left_motor_forward = True
right_motor_forward = True

# Initialize yaw_angle (z-axis rotation)
yaw_angle = 0.0

def rpm_to_linear_speed(rpm):
    """Calculate linear speed from RPM."""
    speed_m_per_s = (rpm / 60.0) * WHEEL_CIRCUMFERENCE 
    return speed_m_per_s

def update_motor_directions(msg):
    """Update motor directions based on the modbus/regs_write topic."""
    global left_motor_forward, right_motor_forward
    _, _, right_direction, left_direction = msg.data
    left_motor_forward = left_direction == 0
    right_motor_forward = right_direction == 0

def imu_callback(msg):
    """Extract yaw from IMU data and update the yaw_angle variable."""
    global yaw_angle
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    orientation_q = msg.orientation
    _, _, yaw_angle = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

def update_odometry(msg):
    """Update odometry data based on RPMs from modbus/regs_read."""
    global odometer_x, odometer_y, previous_time

    # Assuming message format is (250, 250)
    rpm_left, rpm_right = msg.data

    # Adjust the RPM based on the direction
    rpm_left = rpm_left if left_motor_forward else -rpm_left
    rpm_right = rpm_right if right_motor_forward else -rpm_right

    # Calculate linear speeds for each wheel
    linear_speed_left = rpm_to_linear_speed(rpm_left)
    linear_speed_right = rpm_to_linear_speed(rpm_right)

    # Average linear speed
    linear_speed = (linear_speed_left + linear_speed_right) / 2.0

    # Get the current time
    current_time = rospy.Time.now()
    if previous_time is None:
        previous_time = current_time

    # Time difference in seconds
    dt = (current_time - previous_time).to_sec()

    # Update position based on linear speed and IMU yaw angle
    delta_x = linear_speed * math.cos(yaw_angle) * dt
    delta_y = linear_speed * math.sin(yaw_angle) * dt

    odometer_x += delta_x
    odometer_y += delta_y

    # Convert yaw to a quaternion
    orientation_quat = [0.0, 0.0, math.sin(yaw_angle / 2.0), math.cos(yaw_angle / 2.0)]

    # Update the previous time
    previous_time = current_time

    # Prepare and publish the odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Set the position and orientation
    odom_msg.pose.pose = Pose()
    odom_msg.pose.pose.position.x = odometer_x
    odom_msg.pose.pose.position.y = odometer_y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = Quaternion(*orientation_quat)

    # Example covariance for the pose
    odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 0.01, 0, 0, 0,
                                0, 0, 0, 0.01, 0, 0,
                                0, 0, 0, 0, 0.01, 0,
                                0, 0, 0, 0, 0, 0.01]

    # Set the twist (linear speed)
    odom_msg.twist.twist = Twist()
    odom_msg.twist.twist.linear.x = linear_speed
    odom_msg.twist.twist.angular.z = 0.0  # Angular velocity taken care of by the IMU

    # Example covariance for the twist
    odom_msg.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01]

    # Publish the Odometry message
    odom_publisher.publish(odom_msg)
    print(odom_msg)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('motor_rpm_and_imu_to_odometer')

    # Subscribe to the motor RPM topic and motor direction topic
    rospy.Subscriber('modbus/regs_read', UInt16MultiArray, update_odometry)
    rospy.Subscriber('modbus/regs_write', UInt16MultiArray, update_motor_directions)

    # Subscribe to the IMU topic
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    # Publisher for the odometry data
    odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

    # Keep the node running
    rospy.spin()
