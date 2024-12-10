#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
import sys
import os

# Add the path to the robot_controller module
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from robot_controller import CSRobotController

class TestRobotNavigation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('test_robot_navigation', anonymous=True)

        # Create an instance of the robot controller
        self.controller = CSRobotController()

        # Define two points for navigation
        self.point1 = Point(x=1.0, y=1.0, z=0.0)
        self.point2 = Point(x=2.0, y=2.0, z=0.0)

    def navigate_to_points(self):
        """Test navigation between two points."""
        rospy.loginfo("Navigating to Point 1")
        self.controller.move_to_position(self.point1)
        rospy.sleep(5)  # Wait for some time to simulate travel

        rospy.loginfo("Navigating to Point 2")
        self.controller.move_to_position(self.point2)
        rospy.sleep(5)  # Wait for some time to simulate travel

if __name__ == '__main__':
    try:
        test_nav = TestRobotNavigation()
        test_nav.navigate_to_points()
    except rospy.ROSInterruptException:
        pass 