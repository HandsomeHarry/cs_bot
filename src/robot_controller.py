#!/usr/bin/env python
# coding=utf-8

# Teams:
# robot 1, 2: T, Red, Yellow
# robot 3, 4: CT, Green, Blue

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from cs_bot.msg import GameStateMsg, RobotStateMsg, CombatEvent
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

class CSRobotController:
    def __init__(self):
        rospy.init_node('cs_robot_controller', anonymous=True)
        
        # robot state
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        self.health = 100
        self.is_alive = True
        self.team = rospy.get_param('~team', 'CT')
        self.weapon = Gun(rospy.get_param('~weapon', 'rifle'))
        self.position = Pose()
        self.orientation = 0.0
        self.is_planting = False
        self.is_defusing = False
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.map_data = None
        self.turn_speed = 0.00075

        #game state stuff
        self.dead_players = []
        self.round_time_remaining = 90
        self.bomb_being_planted = False

        self.color_to_robot = { # hard coded in for sake of simplicity - can change
        'blue': 'robot1',
        'red': 'robot2',
        'dark_blue': 'robot3',
        'dark_red': 'robot4'
        }
        
        self.color_ranges = {
            'blue': ((115, 225, 225), (125, 255, 255)),    
            'dark_blue': ((115, 225, 35), (125, 255, 55)),  
            'red': ((0, 225, 235), (5, 255, 255)),    
            'dark_red': ((0, 225, 35), (5, 255, 55))       
        }

        if self.team == 'CT':
            self.enemy_colors = ['red','dark_red']
        else: # terrorist
            self.enemy_colors = ['blue','dark_blue']

        self.enemy_detection_threshold = 1500  # amount of pixels needed to detect
        
        # tactical parameters
        self.patrol_points = self.generate_patrol_points()
        self.current_patrol_index = 0
        self.target_position = None
        self.state = "SEARCHING"  # SEARCHING, ENGAGING, AVOIDING
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.robot_name), Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/game/robot_states', RobotStateMsg, queue_size=1)
        self.shoot_pub = rospy.Publisher('/game/shoot', CombatEvent, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/{}/camera/image_raw'.format(self.robot_name), Image, self.process_image)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)  
        rospy.Subscriber('/{}/robot_state'.format(self.robot_name), RobotStateMsg, self.update_state) # robots own status
        rospy.Subscriber('/game/state', GameStateMsg, self.game_state_callback)

        # Add move_base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("%s initialized as %s" % (self.robot_name, self.team))

        rospy.Timer(rospy.Duration(0.1), self.publish_state) # publish state 10/s

    def update_state(self, msg):
        if self.robot_name == msg.robot_name:
            self.health = msg.health
            self.is_alive = msg.is_alive
            self.is_planting = msg.is_planting
            self.is_defusing = msg.is_defusing

    def start_planting(self):
        self.is_planting = True

    def stop_planting(self):
        self.is_planting = False

    def start_defusing(self):
        self.is_defusing = True

    def stop_defusing(self):
        self.is_defusing = False

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


    def process_image(self, image):
        """process image and detect enemy robots"""
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for enemy_color in enemy_colors:
                if enemy_color not in self.dead_players: # only scan for alive enemies
                    enemy_mask = self.get_color_mask(hsv, enemy_color)
                    enemy_moments = cv2.moments(enemy_mask)
                    if moments['m00'] > self.enemy_detection_threshold: # enemy spotted
                        cx = int(moments['m10'] / moments['m00'])
                        cy = int(moments['m01'] / moments['m00'])
                        
                        center_x = frame.shape[1] // 2
                        offset = cx - center_x
                        threshold = frame.shape[1] * 0.02  # 2% frame width
                        
                        if abs(offset) > threshold:
                            self.twist.angular.z = -self.turn_speed * offset  # Negative for right, positive for left
                            self.twist.linear.x = 0.0
                        else:
                            self.twist.angular.z = 0.0  # Stop turning
                            self.twist.linear.x = 0.0
                            self.shoot(enemy_color)
        except Exception as e
            rospy.logerr(f"Error in process_image: {e}")
            
    def game_state_callback(self, msg):
        """process game state callback"""
        self.dead_players = msg.dead_players
        self.round_time_remaining = msg.round_time_remaining
        self.bomb_being_planted = msg.bomb_planted

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

    def shoot(self, enemy_color):
        """shooting"""
        damage_dealt = self.gun.shoot()
        if damage_dealt > 0:
            msg = CombatEvent()
            msg.attacker_name = self.robot_name
            msg.target_name = self.color_to_robot.get(enemy_color)
            msg.damage_dealt = damage_dealt
            self.shoot_pub.publish(msg)

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

    def run(self):
        """main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.is_alive:
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
