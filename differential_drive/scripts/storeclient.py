#!/usr/bin/env python
import rospy
import sys
import os
from slamware_ros_sdk.srv import SyncGetStcm
from std_msgs.msg import String

def sync_set_stcm_node():
    rospy.init_node('sync_set_stcm_node')
    #nh = rospy.get_param('')
    client = rospy.ServiceProxy('slamware_ros_sdk_server_node/sync_get_stcm', SyncGetStcm)
    srv = SyncGetStcm()
    
    #Get file name from paramter server
    #map_file_name = nh.get_param("map_file_name", "")
    map_file_name = rospy.get_param('map_file_name',"/home/jetson/slam_ws/src/differential_drive/scripts/map.stcm")
    if not map_file_name:
    	rospy.logerr("Invalid File Path........")
    	return
    
    #call service
    try: 
    	response = client()
    	rospy.loginfo("Get raw_stcm data success, writing to file......")
    	
    	#check whether the target folder exits. create one if not
    	dir_name = os.path.dirname(map_file_name)
    	if not os.path.exists(dir_name):
    		os.makedirs(dir_name)
    	
    	with open(map_file_name,'wb') as fout:
    		fout.write(response.raw_stcm)
    	rospy.loginfo("raaw_stcm data size: %s", len(response.raw_stcm))
    	rospy.loginfo("Save map at %s success.......", map_file_name)
    except rospy.ServiceException as e:
    	rospy.logerr("Failed to get raw_stcm data: %s", e)
    	return 1
    return 0 

if __name__ == '__main__':
    sys.exit(sync_set_stcm_node())

