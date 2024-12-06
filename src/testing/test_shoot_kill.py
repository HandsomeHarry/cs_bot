#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class TestShooter:
    def __init__(self):
        rospy.init_node('detection_robot', anonymous=True)
        print('starting')
        
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.enemy_alive = True
        self.turn_speed = 0.00075

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        moments = cv2.moments(cv2.inRange(hsv, (0, 225, 35), (5, 255, 55))) # dark red - robot 2
        if moments['m00'] > 0 and self.enemy_alive: # alive enemy spotted
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            
            center_x = frame.shape[1] // 2
            offset = cx - center_x
            threshold = frame.shape[1] * 0.01  # 1% frame width
            
            if abs(offset) > threshold:
                self.twist.angular.z = -self.turn_speed * offset  # Negative for right, positive for left
                self.twist.linear.x = 0.0
                cv2.circle(frame, (cx, cy), 20, (255,165,0), -1)
            else:
                self.twist.angular.z = 0.0  # Stop turning
                self.twist.linear.x = 0.0
                cv2.circle(frame, (cx, cy), 20, (128,0,128), -1)
                self.shoot()
        else:
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0.0
        
        self.cmd_vel_pub.publish(self.twist)
        
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)
    
    def shoot(self):
        print('implementing later')


if __name__ == '__main__':
    try:
        detector = TestShooter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass