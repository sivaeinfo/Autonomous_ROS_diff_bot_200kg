#!/usr/bin/env python
import rospy
import sys
import os
from slamware_ros_sdk.srv import SyncSetStcm
from geometry_msgs.msg import Pose
from std_msgs.msg import String

def main():
	rospy.init_node('sync_set_stcm_node')
	nh = rospy.get_param('')
	
	# Get Map File Name from ROS Paramter Server (robust error handling)
	map_file_name = rospy.get_param('map_file_name', "/home/jetson/slam_ws/src/differential_drive/scripts/map.stcm")
	if map_file_name is None:
		rospy.logerr("Failed to retrieve 'map_file_name' parameter. Please set it on the paramter server.")
		return 1
	rospy.loginfo("Map file path: %s", map_file_name)
	
	#open Map File and read data
	try:
		with open(map_file_name, 'rb') as fin:
			raw_stcm = fin.read()
			data_size = len(raw_stcm) #direct size retrival
	except FileNotFoundError:	
			rospy.logerr("Map File '%s' not found!",map_file_name)
			return 1
	except IOError as e:
		rospy.logerr("Error reading map File: %s", str(e))
		return 1
	rospy.loginfo("Open Map File Success, read data size: %d bytes", data_size)
	
	#Create service client for '/slamware_ros_sdk_server_node/sync_set_stcm'
	service_name = '/slamware_ros_sdk_server_node/sync_set_stcm'
	try:
		rospy.wait_for_service(service_name, timeout=5.0) #wait service
	except rospy.ROSException:
		rospy.logerr("Failed to connect to service '%s' within timeout.", service_name)
		return 1
	
	# service request
	try:	
		srv = rospy.ServiceProxy(service_name, SyncSetStcm)
		pose = Pose()
		pose.position.x = 0
		pose.position.y = 0
		pose.position.z = 0
		pose.orientation.x = 0
		pose.orientation.y = 0
		pose.orientation.z = 0
		pose.orientation.w = 1
		response = srv(raw_stcm, Pose())      # Pass raw_stcm and an empty Pose
		if response is not None:
			rospy.loginfo("Set STCM Map Success")
			return 0
		else:
			rospy.logerr("Failed to set Stcm Map")
			return 1
	except rospy.ServiceException as e:
		rospy.logerr("Service Call Failed: %s", str(e))
		return 1
		
if __name__ == "__main__":
	sys.exit(main())
