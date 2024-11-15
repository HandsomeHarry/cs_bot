#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import CompressedImage
from player import Player
from gun import Gun
from transitions import Machine

class CTBot(Player):
    def __init__(self, namespace):
        super().__init__(namespace, is_CT=True)
        self.current_patrol_index = 0

    def on_enter_moving_and_searching(self):
        """Called when entering moving_and_searching state"""
        super().on_enter_moving_and_searching()
        self.patrol_points()  # Start patrol behavior

    def patrol_points(self):
        """Implement patrol behavior"""
        patrol_points = [
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0)
        ]
        
        while self.state == 'moving_and_searching':
            x, y = patrol_points[self.current_patrol_index]
            self.move_to_point(x, y)
            self.current_patrol_index = (self.current_patrol_index + 1) % len(patrol_points)

    def on_enter_holding_position(self):
        """Called when entering holding_position state"""
        super().on_enter_holding_position()
        rospy.loginfo(f"{self.namespace}: CT holding defensive position")

if __name__ == "__main__":
    rospy.init_node('ct_bot_test_node', anonymous=True)
    namespace = rospy.get_param("~namespace", "robot1")
    ct_bot = CTBot(namespace)
    
    # Start patrolling
    ct_bot.start_moving()
    
    rospy.spin()
