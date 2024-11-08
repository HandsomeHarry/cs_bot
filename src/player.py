import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import CompressedImage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Player:
    def __init__(self, namespace, is_CT, gun=None):
        self.CT = is_CT
        self.state = None
        self.gun = gun if gun is not None else Gun()
        self.move_base_client = actionlib.SimpleActionClient(f'{namespace}/move_base', MoveBaseAction)
        self.image_sub = rospy.Subscriber(f'{namespace}/raspicam_node/image/compressed', CompressedImage, self.image_cb)
        self.cmd_vel_pub = rospy.Publisher(f'{namespace}/cmd_vel', Twist, queue_size=10)

    def move_to(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.wait_for_server()
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        print(f"{self.namespace} has reached destination: ({x}, {y})")

    def image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height, width, _ = image.shape
        center_x = width // 2  # Horizontal center of the image
        tolerance = 20  # Define tolerance for target alignment with the center

        center_strip = image[int(height * 0.4):int(height * 0.6), :]  # 40%-60% height

        hsv_image = cv2.cvtColor(center_strip, cv2.COLOR_BGR2HSV)
        lower_bound, upper_bound = self.get_color_bounds()  # Get color bounds for the target
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours: # If enemy color is detected
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:  
                target_x = int(M["m10"] / M["m00"])

                distance_from_center = target_x - center_x

                if abs(distance_from_center) <= tolerance:
                    self.shoot()  # Call the shoot method if aligned
                    rospy.loginfo("Target aligned. Shooting.")
                else:
                    self.turn(distance_from_center)  # Call the turn method if not aligned
                    rospy.loginfo(f"Target detected. Turning by {distance_from_center} pixels.")
            else:
                rospy.loginfo("No valid target detected within contours.")
        else:
            rospy.loginfo("No target detected in center strip.")


    def get_color_bounds(self):
        if self.CT:  # Terrorists red
            return (0, 100, 100), (10, 255, 255)
        else:  # Blue
            return (110, 100, 100), (130, 255, 255)

    def turn(self, distance):
        # Turn based on distance from center
        twist = Twist()
        twist.angular.z = -0.005 * distance  # Adjust turning speed based on distance
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Turning with distance {distance} from center")

    def shoot(self):
        rospy.loginfo("Shooting!")
