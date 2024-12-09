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
        self.enemy_spotted = False
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

        # rospy.Timer(rospy.Duration(0.1), self.publish_state) # publish state 10/s

        # Add patrol state variables
        self.is_patrolling = False
        self.current_goal_active = False

        # Initialize patrol only for CT side
        if self.team == 'CT':
            self.is_patrolling = True
            self.start_patrol()

        # Add new game state variables
        self.game_phase = "PREP"
        self.bomb_location = Point()
        self.spawn_points = {
            'CT': [
                Point(x=-2.0, y=-2.0, z=0.0),  # CT spawn 1
                Point(x=-2.0, y=-1.5, z=0.0)   # CT spawn 2
            ],
            'T': [
                Point(x=2.0, y=2.0, z=0.0),    # T spawn 1
                Point(x=2.0, y=1.5, z=0.0)     # T spawn 2
            ]
        }
        self.spawn_point = self.assign_spawn_point()
        
        # Add new publisher for bomb events
        self.bomb_event_pub = rospy.Publisher('/game/bomb_events', String, queue_size=1)

    def update_state(self, msg):
        if self.robot_name == msg.robot_name:
            self.health = msg.health
            self.is_alive = msg.is_alive
            self.is_planting = msg.is_planting
            self.is_defusing = msg.is_defusing

    def process_image(self, image):
        """process image and detect enemy robots"""
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for enemy_color in self.enemy_colors:
                enemy_robot = self.color_to_robot.get(enemy_color)
                if enemy_robot not in self.dead_players: # only scan for alive enemies
                    enemy_mask = self.get_color_mask(hsv, enemy_color)
                    moments = cv2.moments(enemy_mask)
                    if moments['m00'] > self.enemy_detection_threshold: # enemy spotted
                        self.enemy_spotted = True
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
                            self.shoot(enemy_robot)
                        return
            self.enemy_spotted = False
        except Exception as e:
            rospy.logerr(f"Error in process_image: {e}")

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

    def game_state_callback(self, msg):
        """process game state callback"""
        self.dead_players = msg.dead_players
        self.round_time_remaining = msg.round_time_remaining
        self.bomb_being_planted = msg.bomb_planted
        self.game_phase = msg.game_phase
        self.bomb_location = msg.bomb_location
        
        # Handle state changes based on game phase
        self.handle_game_phase()

    def generate_patrol_points(self):
        """preset patrol points"""
        patrol_points = [
            Point(x=-1.76, y=-0.75, z=0.0),
            Point(x=1.30, y=-0.62, z=0.0),
            Point(x=0.91, y=1.21, z=0.0),
            Point(x=-0.51, y=0.45, z=0.0),
        ]
        random.shuffle(patrol_points)
        return patrol_points

    def get_next_patrol_point(self):
        """get next patrol point"""
        point = self.patrol_points[self.current_patrol_index]
        self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
        return point

    def shoot(self, enemy_robot):
        """shooting"""
        damage_dealt = self.gun.shoot()
        if damage_dealt > 0:
            msg = CombatEvent()
            msg.attacker_name = self.robot_name
            msg.target_name = enemy_robot
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

    def start_patrol(self):
        """Initialize patrol sequence"""
        if not self.patrol_points:
            rospy.logwarn(f"{self.robot_name}: No patrol points available")
            return
            
        self.current_patrol_index = 0
        self.send_next_patrol_point()

    def send_next_patrol_point(self):
        """Send next patrol point to move_base"""
        if not self.is_patrolling or not self.is_alive:
            return

        target = self.patrol_points[self.current_patrol_index]
        
        # Create and send move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = target
        goal.target_pose.pose.orientation.w = 1.0

        self.current_goal_active = True
        self.move_base.send_goal(
            goal,
            done_cb=self.patrol_goal_done_callback
        )

    def stay_put(self):
        """stay put"""
        self.move_base.cancel_goal()


    def patrol_goal_done_callback(self, status, result):
        """Callback for when a patrol point is reached"""
        self.current_goal_active = False
        if not self.is_patrolling:
            return
            
        # Move to next patrol point
        self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
        self.send_next_patrol_point()

    # def run(self):
    #     """main run loop (deprecated)"""
    #     rate = rospy.Rate(10)  # 10Hz
    #     while not rospy.is_shutdown() and self.is_alive:
    #         self.publish_state()

    #         # state machine handling
    #         if self.avoiding:
    #             self.state = "AVOIDING"
    #             self.is_patrolling = False
    #             if self.current_goal_active:
    #                 self.move_base.cancel_goal()
    #         elif self.detected_enemies:
    #             self.state = "ENGAGING"
    #             self.is_patrolling = False
    #             if self.current_goal_active:
    #                 self.move_base.cancel_goal()
    #             self.handle_combat()
    #         else:
    #             self.state = "SEARCHING"
    #             # Resume patrol for CT team
    #             if self.team == 'CT' and not self.is_patrolling:
    #                 self.is_patrolling = True
    #                 self.send_next_patrol_point()

    #         rate.sleep()

    def map_callback(self, msg):
        self.map_data = msg

    def move_to_position(self, target):
        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = target     # point type e.g. Point(x=1.30, y=-0.62, z=0.0)
        goal.target_pose.pose.orientation.w = 1.0

        # Send goal to move_base
        self.move_base.send_goal(goal)

    def assign_spawn_point(self):
        """Assign a spawn point based on team and robot number"""
        robot_num = int(self.robot_name[-1]) - 1  # Extract number from robot name
        team_spawns = self.spawn_points[self.team]
        spawn_idx = robot_num % len(team_spawns)
        return team_spawns[spawn_idx]

    def handle_game_phase(self):
        """Handle different game phases"""
        if self.game_phase == "PREP":
            self.is_patrolling = False
            self.move_to_position(self.spawn_point)
            
        elif self.game_phase == "ACTIVE":
            if self.team == "CT" and not self.is_patrolling:
                self.is_patrolling = True
                self.start_patrol()
            
        elif self.game_phase == "BOMB_PLANTED":
            if self.team == "CT":
                self.is_patrolling = False
                self.move_to_position(self.bomb_location)
                # When close to bomb, start defusing
                if self.is_near_position(self.bomb_location, threshold=0.5):
                    self.start_defusing()
                    # Notify game manager that defusing has started
                    self.bomb_event_pub.publish("DEFUSE_START")

    def is_near_position(self, target_pos, threshold=0.5):
        """Check if robot is near the bomb"""
        dx = self.position.position.x - target_pos.x
        dy = self.position.position.y - target_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        return distance < threshold

if __name__ == '__main__':
    try:
        controller = CSRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
