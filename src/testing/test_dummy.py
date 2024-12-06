#!/usr/bin/env python

import rospy
import cv2
import numpy as np

class Dummy:
    def __init__(self):
        rospy.init_node('cs_robot_controller', anonymous=True)
        
        # robot state
        self.robot_name = rospy.get_param('~robot_name', 'robot2')
        self.health = 100
        self.robot_state_sub = rospy.Subscriber('/game/robot_states', RobotStateMsg, self.robot_state_callback)
        
