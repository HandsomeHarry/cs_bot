#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class TestDetection:
    def __init__(self):
        rospy.init_node('blue_detection_robot', anonymous=True)
        print('starting')
        
        # Camera subscriber
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.image_callback)
        
        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        
        # CVBridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Movement command
        self.twist = Twist()

    def image_callback(self, data):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Convert to HSV color space for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define blue color range
        lower_blue = np.array([100, 200, 50])
        upper_blue = np.array([140, 255, 255])
        
        # Mask for detecting blue
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        moments = cv2.moments(mask)
        
        # Check if blue is detected
        if moments['m00'] > 0:
            print('blue detected!')
            # Calculate centroid of the blue area
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            
            # Draw a red circle around the detected blue object
            cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
            
            # Determine turn direction to center the blue object
            center_x = frame.shape[1] // 2
            offset = cx - center_x
            threshold = frame.shape[1] * 0.01  # 1% frame width
            
            if abs(offset) > threshold:
                self.twist.angular.z = -0.0005 * offset  # Negative for right, positive for left
                self.twist.linear.x = 0.0
            else:
                print('facing blue, ready to shoot :)')
                self.twist.angular.z = 0.0  # Stop turning
                self.twist.linear.x = 0.0
        else:
            print('blue not detected')
            # If blue is not detected, spin in circles
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0.0
        
        # Publish movement command
        self.cmd_vel_pub.publish(self.twist)
        
        # Display the current frame
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = TestDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
