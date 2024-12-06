#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class TestAllDetection:
    def __init__(self):
        rospy.init_node('detection_robot', anonymous=True)
        print('starting')
        
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        self.twist = Twist()

        self.color_ranges = {
            'blue': ((115, 225, 200), (125, 255, 255)),    
            'dark_blue': ((115, 225, 35), (125, 255, 55)),  
            'red': ((0, 225, 235), (5, 255, 255)),    
            'dark_red': ((0, 225, 35), (5, 255, 55))       
        }

    def get_color_mask(self, hsv_image, color):
        """
        Returns a mask for the specified color from the given HSV image.
        """
        if color not in self.color_ranges:
            raise ValueError(f"Color '{color}' is not defined.")

        ranges = self.color_ranges[color]

        lower, upper = ranges
        mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
        return mask

    def draw_circle(self, moments, frame, R,G,B):
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(frame, (cx, cy), 20, (B,G,R), -1)

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        red = cv2.moments(self.get_color_mask(hsv,'red'))
        darkred = cv2.moments(self.get_color_mask(hsv,'dark_red'))
        blue = cv2.moments(self.get_color_mask(hsv,'blue'))
        darkblue = cv2.moments(self.get_color_mask(hsv,'dark_blue'))
        
        self.draw_circle(red,frame,128,0,128)
        self.draw_circle(darkred,frame,255,165,0)
        self.draw_circle(blue,frame,0,255,0)
        self.draw_circle(darkblue,frame,128,255,128)

            
        self.twist.angular.z = 0.5
        self.twist.linear.x = 0.0
        
        self.cmd_vel_pub.publish(self.twist)
        
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = TestAllDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass