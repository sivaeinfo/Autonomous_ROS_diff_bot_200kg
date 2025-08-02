#!/usr/bin/env python
import rospy
from slamware_ros_sdk.srv import SyncGetStcm

def store_map_data(file_path):
    rospy.wait_for_service('/slamware_ros_sdk_server_node/sync_get_map')
    try:
        rospy.loginfo("Calling 'sync_get_map' service...")
        sync_get_map = rospy.ServiceProxy('/slamware_ros_sdk_server_node/sync_get_map', SyncGetMap)
        resp = sync_get_map()
        map_data = resp.map_data
        map_resolution = resp.map_resolution
        map_width = resp.map_width
        map_height = resp.map_height
        origin_x = resp.origin_x
        origin_y = resp.origin_y

        rospy.loginfo("Received map data with resolution: %f, width: %d, height: %d", map_resolution, map_width, map_height)
        
        # Create a PGM header
        pgm_header = f"P5\n{map_width} {map_height}\n255\n"

        # Convert map data to grayscale and store in bytearray
        grayscale_data = bytearray()
        for val in map_data:
            grayscale_data.append(int(val * 255))

        # Write the PGM header and data to file
        with open(file_path, 'wb') as f:
            f.write(bytearray(pgm_header, 'ascii') + grayscale_data)
        
        rospy.loginfo("Map data stored successfully at %s", file_path)
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == '__main__':
    rospy.init_node('map_data_storage_client')
    file_path = '/home/jetson/slam_ws/src/differential_drive/scripts/map.pgm'
    store_map_data(file_path)

