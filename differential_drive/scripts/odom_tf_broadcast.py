#!/usr/bin/env python
import roslib
import rospy
import tf
import math
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep

class EKFOdometry:
    def __init__(self):
        self.odom_data_pub = rospy.Publisher('odom', Odometry, queue_size=100)
        self.odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)
        self.odom = Odometry()
        self.current_enc_value_left = 0.0
        self.current_enc_value_right = 0.0
        
        # Subscribe to ROS topics
        rospy.Subscriber('modbus/regs_read', UInt16MultiArray, self.calc_leftandright, queue_size=100)
        rospy.Subscriber('initialpose', PoseStamped, self.set_initial_2d, queue_size=1)
        # Initial Pose
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.00000000001
        self.pi = 3.141592

        # Robot Physical constant
        self.w = 0.38
        self.rad_wh = 0.16
        self.enc_rev_pls = 1000
        self.gear_ratio = 25

        # Distance both wheels have traveled
        self.distance_left = 0.0
        self.distance_right = 0.0

        # Flag to see if the initial pose has been received
        self.initial_pose_received = False
        # Flag to start count
        self.initial_l_count = False
        self.initial_r_count = False

        # Initialize ROS node
        rospy.init_node('ekf_odom_pub')

        # Set the data fields of the odometry message
        self.odom.header.frame_id = 'odom'
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.twist.twist.linear.x = 0
        self.odom.twist.twist.linear.y = 0
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = 0
        self.odom_old.pose.pose.position.x = self.initial_x
        self.odom_old.pose.pose.position.y = self.initial_y
        self.odom_old.pose.pose.orientation.z = self.initial_theta
        
        # Set the desired callback rate
        self.callback_rate = rospy.Rate(1)  # 1 Hz
	
	# Initialize odom tf
        self.br = tf.TransformBroadcaster()
        self.odom_trans = TransformStamped()
        self.odom = Odometry()

        # Rate for the ROS node
        self.rate = rospy.Rate(10)

        self.TICKS_PER_METER = self.ticks_per_meter()

        # Initialize last counts
        self.initial_last_count_l = 0
        self.initial_last_count_r = 0

    def ticks_per_meter(self):
        TPM = (self.gear_ratio * self.enc_rev_pls * 1000) / (2 * self.pi * self.rad_wh*1000)
        return TPM

    # Callback to set the initial 2D pose from RViz or a manual pose publisher
    def set_initial_2d(self, rviz_click):
        self.odom_old.pose.pose.position.x = rviz_click.pose.position.x
        self.odom_old.pose.pose.position.y = rviz_click.pose.position.y
        self.odom_old.pose.pose.orientation.z = rviz_click.pose.orientation.z
        self.initial_pose_received = True

    # Calculate the distance that the left and right wheel has traveled since the last cycle
    def calc_leftandright(self, lt_rt_count):
        self.current_enc_value_left = lt_rt_count.data[0]
        self.current_enc_value_right = lt_rt_count.data[1]
        #rospy.loginfo("LEFT ENC VAL = %d & RIGHT ENC VAL = %d", self.current_enc_value_left, self.current_enc_value_right)
  
        
    def calc_wheel_distance(self, cur_left_encval, cur_right_encval):            
        if not self.initial_l_count:
            self.distance_left = 0.0
            self.initial_l_count = True
            #self.abs_count_left = lt_rt_count.data[0]
        else:
            #if lt_rt_count.data[0] != 0 and self.initial_last_count_l != 0:
            left_ticks = cur_left_encval - self.initial_last_count_l
            self.distance_left = left_ticks / self.TICKS_PER_METER
            rospy.loginfo("left TICKS LEFT = %d", left_ticks)
            #self.abs_count_left = lt_rt_count.data[0]
        if not self.initial_r_count:
            self.distance_right = 0.0
            self.initial_r_count = True
            #self.abs_count_right = lt_rt_count.data[1]
        else:
            #if lt_rt_count.data[1] != 0 and self.initial_last_count_r != 0:
            right_ticks = cur_right_encval - self.initial_last_count_r
            self.distance_right = right_ticks / self.TICKS_PER_METER
            rospy.loginfo("Right TICKS RIGHT = %d", right_ticks)     
        rospy.loginfo("left and right METER LEFT = %f  RIGHT = %f", self.distance_left, self.distance_right)
        self.initial_last_count_l = cur_left_encval
        self.initial_last_count_r = cur_right_encval
        

    # Publish a nav_msgs::Odometry message in quaternion format
    def publish_quat(self):
        odom_q = tf.transformations.quaternion_from_euler(0, 0, self.odom.pose.pose.orientation.z)
        odom_q = Quaternion(*odom_q)
        self.odom_trans.header.stamp = self.odom.header.stamp
        self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = odom_q
        self.odom.header.stamp = self.odom.header.stamp
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = self.odom.pose.pose.position.x
        self.odom.pose.pose.position.y = self.odom.pose.pose.position.y
        self.odom.pose.pose.position.z = self.odom.pose.pose.position.z
        self.odom.pose.pose.orientation = odom_q
        self.odom.twist.twist.linear.x = self.odom.twist.twist.linear.x
        self.odom.twist.twist.linear.y = self.odom.twist.twist.linear.y
        self.odom.twist.twist.linear.z = self.odom.twist.twist.linear.z
        self.odom.twist.twist.angular.x = self.odom.twist.twist.angular.x
        self.odom.twist.twist.angular.y = self.odom.twist.twist.angular.y
        self.odom.twist.twist.angular.z = self.odom.twist.twist.angular.z
        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                self.odom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                self.odom.pose.covariance[i] += 0.1
            else:
                self.odom.pose.covariance[i] = 0
        #publish transforms
        self.br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0,self.odom.pose.pose.orientation.z), self.odom.header.stamp, "base_link", "odom")
        self.odom_data_pub_quat.publish(self.odom)

    # Update odometry information
    def update_odom(self):
        # Calculate the average distance
        cycle_distance = (self.distance_right + self.distance_left) / 2

        # Calculate the number of radians the robot has turned since the last cycle
        cycle_angle = math.asin((self.distance_right - self.distance_left) / self.w)

        # Average angle during the last cycle
        avg_angle = cycle_angle / 2 + self.odom_old.pose.pose.orientation.z

        if avg_angle > self.pi:
            avg_angle -= 2 * self.pi
        elif avg_angle < -self.pi:
            avg_angle += 2 * self.pi

        # Calculate the new pose (x, y, and theta)
        self.odom.pose.pose.position.x = self.odom_old.pose.pose.position.x + math.cos(avg_angle) * cycle_distance
        self.odom.pose.pose.position.y = self.odom_old.pose.pose.position.y + math.sin(avg_angle) * cycle_distance
        self.odom.pose.pose.orientation.z = cycle_angle + self.odom_old.pose.pose.orientation.z
        rospy.loginfo("Position in X = %f m Y = %f m & theta = %f", self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.orientation.z)
	
        # Prevent lockup from a single bad cycle
        if math.isnan(self.odom.pose.pose.position.x) or math.isnan(self.odom.pose.pose.position.y) or math.isnan(
                self.odom.pose.pose.position.z):
            self.odom.pose.pose.position.x = self.odom_old.pose.pose.position.x
            self.odom.pose.pose.position.y = self.odom_old.pose.pose.position.y
            self.odom.pose.pose.orientation.z = self.odom_old.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if self.odom.pose.pose.orientation.z > self.pi:
            self.odom.pose.pose.orientation.z -= 2 * self.pi
        elif self.odom.pose.pose.orientation.z < -self.pi:
            self.odom.pose.pose.orientation.z += 2 * self.pi

        # Compute the velocity
        self.odom.header.stamp = rospy.Time.now()
        time_diff = self.odom.header.stamp.to_sec() - self.odom_old.header.stamp.to_sec()
        self.odom.twist.twist.linear.x = cycle_distance / time_diff
        self.odom.twist.twist.angular.z = cycle_angle / time_diff

        # Save the pose data for the next cycle
        self.odom_old.pose.pose.position.x = self.odom.pose.pose.position.x
        self.odom_old.pose.pose.position.y = self.odom.pose.pose.position.y
        self.odom_old.pose.pose.orientation.z = self.odom.pose.pose.orientation.z
        self.odom_old.header.stamp = self.odom.header.stamp

        # odom_data_pub.publish(odomNew);
        self.odom_data_pub.publish(self.odom)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.initial_pose_received or True:
                cur_l_encv = self.current_enc_value_left
                cur_r_encv = self.current_enc_value_right
                self.calc_wheel_distance(cur_l_encv, cur_r_encv) 
                self.update_odom()
                self.publish_quat()
            rate.sleep()

if __name__ == "__main__":
    ekf_odom_pub = EKFOdometry()
    #sleep(100)
    ekf_odom_pub.run()

