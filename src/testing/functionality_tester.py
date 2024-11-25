#!/usr/bin/env python
# coding=utf-8

import rospy
import unittest
import rostest
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class FunctionalityTester:
    def __init__(self):
        rospy.init_node('functionality_tester')
        self.bridge = CvBridge()
        
        # test saving data
        self.robot1_state = None
        self.robot2_state = None
        self.combat_events = []
        self.robot1_images = []
        self.robot2_images = []

        # subscribe related topics
        rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.robot1_image_callback)
        rospy.Subscriber('/robot2/camera/rgb/image_raw', Image, self.robot2_image_callback)

        # publishers
        self.robot1_cmd = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.robot2_cmd = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)

        # wait for all topics to be ready
        rospy.sleep(2)

    def robot_state_callback(self, msg):
        if msg.robot_name == 'robot1':
            self.robot1_state = msg
        else:
            self.robot2_state = msg

    def combat_event_callback(self, msg):
        self.combat_events.append(msg)

    def robot1_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.robot1_images.append(cv_image)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def robot2_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.robot2_images.append(cv_image)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def test_basic_movement(self):
        """test basic movement"""
        rospy.loginfo("Testing basic movement...")
        
        # create move command
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0

        # record initial position
        initial_pos = self.robot1_state.position if self.robot1_state else None
        
        # send move command
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.robot1_cmd.publish(twist)
            rospy.sleep(0.1)

        # check if position changed
        final_pos = self.robot1_state.position if self.robot1_state else None
        
        if initial_pos and final_pos:
            distance = ((final_pos.x - initial_pos.x)**2 + 
                       (final_pos.y - initial_pos.y)**2)**0.5
            rospy.loginfo(f"Movement test - Distance traveled: {distance}m")
            return distance > 0.1
        return False

    def test_enemy_detection(self):
        """test enemy detection"""
        rospy.loginfo("Testing enemy detection...")
        
        if len(self.robot1_images) == 0:
            rospy.logwarn("No images received from robot1")
            return False

        # get the latest image
        latest_image = self.robot1_images[-1]
        
        # convert to HSV color space
        hsv = cv2.cvtColor(latest_image, cv2.COLOR_BGR2HSV)
        
        # detect blue (enemy color)
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # check if a large enough color block is detected
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                     cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            rospy.loginfo(f"Enemy detection test - Largest blue area: {area}")
            return area > 500
        return False

    def test_combat_system(self):
        """test combat system"""
        rospy.loginfo("Testing combat system...")
        
        # clear previous combat events
        self.combat_events = []
        
        # move robots face to face
        self.move_robots_face_to_face()
        
        # wait for a few seconds, so robots have a chance to engage
        rospy.sleep(5)
        
        # check if any combat events are generated
        combat_count = len(self.combat_events)
        rospy.loginfo(f"Combat test - Number of combat events: {combat_count}")
        return combat_count > 0

    def move_robots_face_to_face(self):
        """move robots face to face"""
        twist1 = Twist()
        twist2 = Twist()
        
        # set turning commands
        twist1.angular.z = 0.5
        twist2.angular.z = -0.5
        
        # execute turning commands
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.robot1_cmd.publish(twist1)
            self.robot2_cmd.publish(twist2)
            rospy.sleep(0.1)

    def run_all_tests(self):
        """run all tests"""
        rospy.loginfo("Starting functionality tests...")
        
        # run movement test
        movement_result = self.test_basic_movement()
        rospy.loginfo(f"Movement test {'PASSED' if movement_result else 'FAILED'}")
        
        # run enemy detection test
        detection_result = self.test_enemy_detection()
        rospy.loginfo(f"Enemy detection test {'PASSED' if detection_result else 'FAILED'}")
        
        # run combat system test
        combat_result = self.test_combat_system()
        rospy.loginfo(f"Combat system test {'PASSED' if combat_result else 'FAILED'}")
        
        return movement_result, detection_result, combat_result

if __name__ == '__main__':
    try:
        tester = FunctionalityTester()
        movement_result, detection_result, combat_result = tester.run_all_tests()
        
        # output final test results
        rospy.loginfo("\n=== Test Results ===")
        rospy.loginfo(f"Basic Movement: {'PASSED' if movement_result else 'FAILED'}")
        rospy.loginfo(f"Enemy Detection: {'PASSED' if detection_result else 'FAILED'}")
        rospy.loginfo(f"Combat System: {'PASSED' if combat_result else 'FAILED'}")
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
