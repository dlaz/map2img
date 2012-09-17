#!/usr/bin/env python
import roslib; roslib.load_manifest('map2img')
from cv_bridge import CvBridge
import sensor_msgs.msg
import nav_msgs.msg
import numpy as np
import rospy
import cv

map_img_msg = None
img_pub = None
bridge = None

def map_to_arr(map_msg):
    map_arr = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
    
    # -1 is unknown. If we cast that to a uint8, it won't look right.
    # map_saver uses 205 for unknown values, so let's change all -1 cells to 205
    map_arr[map_arr == -1] = 205
    return np.uint8(np.flipud(map_arr))

def map_cb(msg):
    global map_img_msg
    map_img_msg = bridge.cv_to_imgmsg(cv.fromarray(map_to_arr(msg)))

if __name__ == '__main__':
    rospy.init_node('map2img')
    bridge = CvBridge()
    img_pub = rospy.Publisher('map_image', sensor_msgs.msg.Image)
    rospy.Subscriber('map', nav_msgs.msg.OccupancyGrid, map_cb)
    rate = rospy.Rate(10)
    rospy.loginfo('Waiting for map')
    while map_img_msg is None:
        rate.sleep()
    rospy.loginfo('Got map')        
    while not rospy.is_shutdown():
        img_pub.publish(map_img_msg)
        rate.sleep()