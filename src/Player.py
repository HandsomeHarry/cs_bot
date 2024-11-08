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
        self.gun = gun if gun is not None else Gun("Rifle", damage=20, fire_rate=0.2, accuracy=0.8, ammo_capacity=30)
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

        # Process image to find enemy color in the center strip
        height, width, _ = image.shape
        center_strip = image[int(height * 0.4):int(height * 0.6), :]  # Taking a strip in the center

        # Convert to HSV and create a mask for the enemy color
        hsv_image = cv2.cvtColor(center_strip, cv2.COLOR_BGR2HSV)
        lower_bound, upper_bound = self.get_color_bounds()
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        target_visible = cv2.countNonZero(mask) > 0

        if target_visible:
            self.is_target_in_sight = True
            self.turn_to()
            self.shoot()
        else:
            self.is_target_in_sight = False

    def get_color_bounds(self):
        if not self.CT:  # Terrorists red
            return (0, 100, 100), (10, 255, 255)
        else:  # Blue
            return (110, 100, 100), (130, 255, 255)

    def turn_to(self):
        twist = Twist()
        twist.angular.z = 0.3  # Rotate slowly to aim
        self.cmd_vel_pub.publish(twist)
        print(f"{self.namespace} is turning to target.")

    def shoot(self):
        if self.is_target_in_sight and self.gun.shoot():
            print(f"{self.namespace} is shooting at the target!")
