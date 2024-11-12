#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import CompressedImage
import actionlib
from player import Player
from gun import Gun

if __name__ == "__main__":
    rospy.init_node('blue_robot_test_node', anonymous=True)
    namespace = rospy.get_param("~namespace", "robot1")  # Default to "robot1"
    player = Player('robot2', True)

    top_x, top_y = 0, 2.0
    player.move_to(top_x, top_y)
    
    # Start moving to the top of the map
    # Spin to keep the node active
    rospy.spin()
