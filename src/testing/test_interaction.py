#!/usr/bin/env python
# coding=utf-8


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from test_navigation import NavigationTest

if __name__ == '__main__':
    try:
        navigator = NavigationTest()

        if not navigator.move_to_goal(1, 0.0):
            rospy.logwarn("something failed")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("interrupted :(")
