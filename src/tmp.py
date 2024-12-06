#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, Pose, PoseWithCovarianceStamped
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
    A = Point(x=10.0, y=10.0, z=0.0)
    B = Point(x=-10.0, y=-10.0, z=0.0)

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
        self.teammates_positions = {}
        self.enemies_positions = {}
        self.avoiding = False
        
        # AMCL parameters
        self.initial_pose_sent = False
        self.amcl_pose = None
        
        # visual recognition parameters
        self.color_ranges = {
            'blue': ((115, 225, 225), (125, 255, 255)),    
            'dark_blue': ((115, 225, 35), (125, 255, 55)),  
            'red': ((0, 225, 235), (5, 255, 255)),    
            'dark_red': ((0, 225, 35), (5, 255, 55))       
        }

        self.min_enemy_area = 50
        self.enemy_detection_threshold = 0.01
        
        # tactical parameters
        self.patrol_points = self.generate_patrol_points()
        self.current_patrol_index = 0
        self.target_position = None
        self.state = "SEARCHING"
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.robot_name), Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/game/robot_states', RobotStateMsg, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/{}/debug_image'.format(self.robot_name), Image, queue_size=1)
        self.debug_mask_pub = rospy.Publisher('/{}/debug_mask'.format(self.robot_name), Image, queue_size=1)
        self.initial_pose_pub = rospy.Publisher('/{}/initialpose'.format(self.robot_name), PoseWithCovarianceStamped, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.game_state_callback)
        rospy.Subscriber('/{}/scan'.format(self.robot_name), LaserScan, self.laser_callback)
        rospy.Subscriber('/{}/camera/image_raw'.format(self.robot_name), Image, self.camera_callback)
        rospy.Subscriber('/{}/amcl_pose'.format(self.robot_name), PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Add move_base client
        self.move_base = actionlib.SimpleActionClient('/{}/move_base'.format(self.robot_name), MoveBaseAction)
        self.move_base.wait_for_server()

        rospy.loginfo("%s initialized as %s" % (self.robot_name, self.team))

    def amcl_callback(self, msg):
        """Handle AMCL pose updates"""
        self.amcl_pose = msg.pose.pose
        self.position = self.amcl_pose
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.orientation = euler[2]

    def set_initial_pose(self):
        """Set initial pose for AMCL"""
        if not self.initial_pose_sent:
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = "map"
            initial_pose.header.stamp = rospy.Time.now()
            
            # Set initial position and orientation
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            initial_pose.pose.pose.position.z = 0.0
            initial_pose.pose.pose.orientation.w = 1.0
            
            # Set covariance
            initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            
            self.initial_pose_pub.publish(initial_pose)
            self.initial_pose_sent = True

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

    def is_target_reached(self):
        """Check if current target position is reached"""
        if not self.target_position or not self.position:
            return False
            
        distance = math.sqrt(
            (self.position.position.x - self.target_position.x) ** 2 +
            (self.position.position.y - self.target_position.y) ** 2
        )
        return distance < 0.5  # 0.5 meters threshold

    def run(self):
        """main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        
        # Set initial pose for AMCL
        self.set_initial_pose()
        
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