#!/usr/bin/env python
# coding=utf-8

# Teams:
# robot 1, 2: T, Red, Yellow
# robot 3, 4: CT, Green, Blue

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from cs_bot.msg import GameStateMsg, RobotStateMsg
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import tf
import math
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gun import Gun

class BombSite(Enum):
    A = Point(x=10.0, y=10.0, z=0.0)  # make this a random point in the rectangular range of two points?
    B = Point(x=-10.0, y=-10.0, z=0.0)# or read from a text file? @anson pls implement this

class CSRobotController:
    def __init__(self):
        rospy.init_node('cs_robot_controller', anonymous=True)
        
        # robot state
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        self.health = 100
        self.is_alive = True
        self.team = rospy.get_param('~team', 'T')
        self.weapon = Gun(rospy.get_param('~weapon', 'rifle'))
        self.position = Pose()
        self.orientation = 0.0
        self.detected_enemies = []
        self.is_planting = False
        self.is_defusing = False
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        
        # visual recognition parameters
        self.color_ranges = {
            'blue': ((115, 225, 225), (125, 255, 255)),    
            'dark_blue': ((115, 225, 35), (125, 255, 55)),  
            'red': ((0, 225, 235), (5, 255, 255)),    
            'dark_red': ((0, 225, 35), (5, 255, 55))       
        }

        self.min_enemy_area = 50  # minimum detection area
        self.enemy_detection_threshold = 0.01  # detection confidence threshold
        
        # tactical parameters
        self.patrol_points = self.generate_patrol_points()
        self.current_patrol_index = 0
        self.target_position = None
        self.state = "SEARCHING"  # SEARCHING, ENGAGING, AVOIDING
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.robot_name), Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/game/robot_states', RobotStateMsg, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/{}/debug_image'.format(self.robot_name), Image, queue_size=1)
        self.debug_mask_pub = rospy.Publisher('/{}/debug_mask'.format(self.robot_name), Image, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.game_state_callback)
        rospy.Subscriber('/{}/scan'.format(self.robot_name), LaserScan, self.laser_callback)
        rospy.Subscriber('/{}/camera/image_raw'.format(self.robot_name), Image, self.camera_callback)
        rospy.Subscriber('/{}/odom'.format(self.robot_name), Odometry, self.odom_callback)        rospy.loginfo("%s initialized as %s" % (self.robot_name, self.team))

        # Add map subscriber
        self.map_data = None
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Add move_base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()

    def start_planting(self):
        self.is_planting = True

    def stop_planting(self):
        self.is_planting = False

    def start_defusing(self):
        self.is_defusing = True

    def stop_defusing(self):
        self.is_defusing = False
            
    def odom_callback(self, msg):
        self.position = msg.pose.pose
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.orientation = euler[2]

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(e)

    def get_color_mask(self, hsv_image, color):
        """
        Returns a mask for the specified color from the given HSV image.
        
        :param hsv_image: Input image in HSV format.
        :param color: Color name ('blue', 'dark_red', etc.).
        :return: Mask of the specified color.
        """
        if color not in self.color_ranges:
            raise ValueError(f"Color '{color}' is not defined.")

        ranges = self.color_ranges[color]

        if color in ['red', 'dark_red']:  # Handle colors with two ranges
            lower1, upper1, lower2, upper2 = ranges
            mask1 = cv2.inRange(hsv_image, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv_image, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = ranges
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))

        return mask


    def process_image(self, image):
        """process image and detect enemy robots"""
        try:
            pass
        except Exception as e:
            rospy.logerr(f"Error in process_image: {e}")
            
    def game_state_callback(self, msg):
        """process game state callback"""
        if msg.robot_name != self.robot_name:
            if msg.team == self.team:
                self.teammates_positions[msg.robot_name] = msg.position
            else:
                self.enemies_positions[msg.robot_name] = msg.position
                
            if not msg.is_alive:
                if msg.robot_name in self.teammates_positions:
                    del self.teammates_positions[msg.robot_name]
                if msg.robot_name in self.enemies_positions:
                    del self.enemies_positions[msg.robot_name]

    def generate_patrol_points(self):
        """generate patrol points"""
        patrol_points = [
            Point(x=5.0, y=5.0, z=0.0),
            Point(x=-5.0, y=5.0, z=0.0),
            Point(x=-5.0, y=-5.0, z=0.0),
            Point(x=5.0, y=-5.0, z=0.0),
            BombSite.A.value,
            BombSite.B.value
        ]
        random.shuffle(patrol_points)
        return patrol_points

    def get_next_patrol_point(self):
        """get next patrol point"""
        point = self.patrol_points[self.current_patrol_index]
        self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
        return point

    def shoot(self):
        """shooting"""
        pass    

    def publish_state(self):
        """publish robot state"""
        msg = RobotStateMsg()
        msg.robot_name = self.robot_name
        msg.team = self.team
        msg.health = self.health
        msg.is_alive = self.is_alive
        msg.weapon_type = self.weapon.name
        msg.is_planting = self.is_planting
        msg.is_defusing = self.is_defusing
        self.state_pub.publish(msg)

    def take_damage(self, damage):
        """process taking damage"""
        if self.is_alive:
            self.health -= damage
            if self.health <= 0:
                self.health = 0
                self.is_alive = False
                rospy.loginfo("%s has been eliminated" % self.robot_name)


    def run(self):
        """main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.is_alive:
            # publish state
            self.publish_state()
            
            # state machine handling
            if self.avoiding:
                self.state = "AVOIDING"
            elif self.detected_enemies:
                self.state = "ENGAGING"
                self.handle_combat()
            else:
                self.state = "SEARCHING"
                # execute patrol task
                if not self.target_position or self.is_target_reached():
                    self.target_position = self.get_next_patrol_point()
                
                if not self.avoiding:
                    self.move_to_position(self.target_position)
            
            rate.sleep()

    def map_callback(self, msg):
        self.map_data = msg

    def move_to_position(self, target):
        if self.avoiding:
            return

        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = target
        goal.target_pose.pose.orientation.w = 1.0

        # Send goal to move_base
        self.move_base.send_goal(goal)

if __name__ == '__main__':
    try:
        controller = CSRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
