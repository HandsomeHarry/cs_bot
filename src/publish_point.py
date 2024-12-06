#!/usr/bin/env python

# run the following commands in the terminal to start the map server and rviz
#rosrun map_server map_server maps/world3_map.yaml
#rosrun rviz rviz -d path/to/your_rviz_config.rviz

import rospy
from geometry_msgs.msg import PointStamped

def point_callback(msg):
    rospy.loginfo("Received point: x=%f, y=%f, z=%f", msg.point.x, msg.point.y, msg.point.z)

def listener():
    rospy.init_node('point_listener', anonymous=True)
    rospy.Subscriber('/clicked_point', PointStamped, point_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()