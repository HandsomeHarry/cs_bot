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

        # Define a specific point for navigation
        self.target_point = Point(x=3.0, y=3.0, z=0.0)  # Change these coordinates to your desired target

    def navigate_to_target(self):
        """Test navigation to a specific target point."""
        rospy.loginfo(f"Navigating to Target Point: {self.target_point}")
        self.controller.move_to_position(self.target_point)
        rospy.sleep(5)  # Wait for some time to simulate travel

if __name__ == '__main__':
    try:
        test_nav = TestRobotNavigation()
        test_nav.navigate_to_target()
    except rospy.ROSInterruptException:
        pass 