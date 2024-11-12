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

class BlueRobotPlayer(Player):
    def __init__(self):
        super().__init__(namespace="robot2", is_CT=True)

    def move_to_top(self):
        # Example coordinates at the top of the map, adjust as necessary
        top_x, top_y = 0, 2.0
        self.move_to(top_x, top_y)

    def image_cb(self, msg):
        if self.enemy_detected:
            return  # Stop processing if already detected
        
        # Decode the image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Extract the central strip
        height, width, _ = image.shape
        center_strip = image[int(height * 0.4):int(height * 0.6), :]

        # Convert to HSV and detect red color
        hsv_image = cv2.cvtColor(center_strip, cv2.COLOR_BGR2HSV)
        lower_bound, upper_bound = self.get_color_bounds(self.enemy_color)
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Check if any red color is detected in the center strip
        if cv2.countNonZero(mask) > 0:
            self.enemy_detected = True
            self.stop_and_turn_to_enemy()

    def stop_and_turn_to_enemy(self):
        # Stop the robot
        self.move_base_client.cancel_all_goals()
        
        # Rotate to face the enemy
        twist = Twist()
        twist.angular.z = 0.3  # Adjust rotation speed as necessary
        self.cmd_vel_pub.publish(twist)
        
        rospy.loginfo("Enemy detected! Robot2 is turning to face the enemy.")

    def get_color_bounds(self, color):
        if color == (0, 0, 255):  # Red
            return (0, 100, 100), (10, 255, 255)

if __name__ == "__main__":
    rospy.init_node('blue_robot_test_node', anonymous=True)
    namespace = rospy.get_param("~namespace", "robot1")  # Default to "robot1"
    player = Player('robot2', True)

    top_x, top_y = 2.0,0.0
    player.publish_pose(-2,0.0)
    player.move_to(top_x, top_y)
    
    # Start moving to the top of the map
    # Spin to keep the node active
    rospy.spin()
