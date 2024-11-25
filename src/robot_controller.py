#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cs_game.msg import GameStateMsg, RobotStateMsg
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import tf
import math
import random

class WeaponType(Enum):
    RIFLE = {"damage": 25, "speed": 0.2, "reload_time": 2.5, "range": 30}
    SNIPER = {"damage": 100, "speed": 0.1, "reload_time": 3.5, "range": 50}
    SMG = {"damage": 15, "speed": 0.3, "reload_time": 1.5, "range": 15}

class BombSite(Enum):
    A = Point(x=10.0, y=10.0, z=0.0)  # A site coordinates
    B = Point(x=-10.0, y=-10.0, z=0.0) # B site coordinates

class CSRobotController:
    def __init__(self):
        rospy.init_node('cs_robot_controller', anonymous=True)
        
        # robot state
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        self.health = 100
        self.is_alive = True
        self.team = rospy.get_param('~team', 'T')
        self.weapon = WeaponType[rospy.get_param('~weapon', 'RIFLE')]
        self.position = Pose()
        self.orientation = 0.0
        self.ammo = 30
        self.last_shot_time = 0
        self.reload_state = False
        self.detected_enemies = []
        self.is_planting = False
        self.is_defusing = False
        
        # visual recognition parameters
        self.enemy_color_ranges = {
            'T': {  # red team color range
                'lower': np.array([0, 120, 100]),   # HSV red range lower limit
                'upper': np.array([10, 255, 255])   # HSV red range upper limit
            },
            'CT': {  # blue team color range
                'lower': np.array([100, 120, 100]), # HSV blue range lower limit
                'upper': np.array([130, 255, 255])  # HSV blue range upper limit
            }
        }
        self.min_enemy_area = 50  # minimum detection area
        self.enemy_detection_threshold = 0.01  # detection confidence threshold
        
        # obstacle avoidance parameters
        self.obstacle_threshold = 0.7
        self.danger_threshold = 0.4
        self.safe_distance = 1.0
        self.obstacle_detected = False
        self.avoiding = False
        self.scan_ranges = []
        self.scan_angle_min = 0
        self.scan_angle_increment = 0
        
        # obstacle avoidance region definition (angle)
        self.regions = {
            'front': {'min': -30, 'max': 30},
            'front_left': {'min': 30, 'max': 60},
            'left': {'min': 60, 'max': 120},
            'back_left': {'min': 120, 'max': 150},
            'back': {'min': 150, 'max': -150},
            'back_right': {'min': -150, 'max': -120},
            'right': {'min': -120, 'max': -60},
            'front_right': {'min': -60, 'max': -30},
        }
        
        # tactical parameters
        self.patrol_points = self.generate_patrol_points()
        self.current_patrol_index = 0
        self.target_position = None
        self.state = "SEARCHING"  # SEARCHING, ENGAGING, AVOIDING
        
        # PID control parameters
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0
        
        # position error tolerance
        self.position_tolerance = 0.5
        self.angle_tolerance = 0.1
        
        # game state
        self.teammates_positions = {}
        self.enemies_positions = {}
        
        # ROS interfaces
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.robot_name), Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/game/robot_states', RobotStateMsg, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/{}/debug_image'.format(self.robot_name), Image, queue_size=1)
        self.debug_mask_pub = rospy.Publisher('/{}/debug_mask'.format(self.robot_name), Image, queue_size=1)
        
        # subscribers
        self.setup_subscribers()
        rospy.loginfo("%s initialized as %s" % (self.robot_name, self.team))
    def setup_subscribers(self):
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.game_state_callback)
        rospy.Subscriber('/{}/scan'.format(self.robot_name), LaserScan, self.laser_callback)
        rospy.Subscriber('/{}/camera/image_raw'.format(self.robot_name), Image, self.camera_callback)
        rospy.Subscriber('/{}/odom'.format(self.robot_name), Odometry, self.odom_callback)

    def start_planting(self):
        self.is_planting = True

    def stop_planting(self):
        self.is_planting = False

    def start_defusing(self):
        self.is_defusing = True

    def stop_defusing(self):
        self.is_defusing = False

    def angle_to_index(self, angle):
        normalized_angle = angle
        if angle < self.scan_angle_min:
            normalized_angle += 2 * math.pi
        index = int((normalized_angle - self.scan_angle_min) / self.scan_angle_increment)
        return max(0, min(index, len(self.scan_ranges) - 1))

    def get_region_distances(self, min_angle, max_angle):
        start_idx = self.angle_to_index(math.radians(min_angle))
        end_idx = self.angle_to_index(math.radians(max_angle))
        
        if start_idx <= end_idx:
            return self.scan_ranges[start_idx:end_idx]
        else:
            return np.concatenate([self.scan_ranges[start_idx:], self.scan_ranges[:end_idx]])

    def check_obstacles(self):
        obstacles = {}
        for region, angles in self.regions.items():
            distances = self.get_region_distances(angles['min'], angles['max'])
            valid_distances = distances[~np.isnan(distances) & ~np.isinf(distances)]
            if len(valid_distances) > 0:
                min_dist = np.min(valid_distances)
                obstacles[region] = min_dist < self.obstacle_threshold and min_dist > 0.1
        return obstacles

    def laser_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
        
        obstacles = self.check_obstacles()
        
        if any(obstacles.values()):
            self.obstacle_detected = True
            self.handle_obstacle_detection(obstacles)
        else:
            self.obstacle_detected = False
            if self.avoiding:
                self.avoiding = False
                if self.target_position:
                    self.move_to_position(self.target_position)

    def handle_obstacle_detection(self, obstacles):
        cmd = Twist()
        self.avoiding = True
        
        if obstacles.get('front', False):
            cmd.linear.x = -0.2  # backward
            
            if obstacles.get('left', False) and not obstacles.get('right', False):
                cmd.angular.z = -1.0  # right turn
            elif obstacles.get('right', False) and not obstacles.get('left', False):
                cmd.angular.z = 1.0   # left turn
            else:
                cmd.angular.z = 1.0   # default left turn
                
        elif obstacles.get('left', False):
            cmd.angular.z = -0.5  # right turn
            cmd.linear.x = 0.1
        elif obstacles.get('right', False):
            cmd.angular.z = 0.5   # left turn
            cmd.linear.x = 0.1
        else:
            self.avoiding = False
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd)

    def is_target_reached(self):
        if self.target_position is None:
            return False
            
        dx = self.target_position.x - self.position.position.x
        dy = self.target_position.y - self.position.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        target_angle = math.atan2(dy, dx)
        angle_diff = abs(math.atan2(math.sin(target_angle - self.orientation), 
                                  math.cos(target_angle - self.orientation)))
                                  
        return distance < self.position_tolerance and angle_diff < self.angle_tolerance

    def move_to_position(self, target):
        if self.avoiding:
            return
            
        if self.is_target_reached():
            self.target_position = None
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
            
        dx = target.x - self.position.position.x
        dy = target.y - self.position.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        cmd = Twist()
        
        angle_diff = math.atan2(math.sin(target_angle - self.orientation), 
                              math.cos(target_angle - self.orientation))
        
        cmd.angular.z = self.Kp_angular * angle_diff
        
        if abs(angle_diff) < self.angle_tolerance:
            cmd.linear.x = self.Kp_linear * distance
            
        cmd.linear.x = min(max(cmd.linear.x, -0.5), 0.5)
        cmd.angular.z = min(max(cmd.angular.z, -1.0), 1.0)
        
        if not self.avoiding:
            self.cmd_vel_pub.publish(cmd)
            
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

    def process_image(self, image):
        """process image and detect enemy robots"""
        try:
            # convert to HSV color space
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # get enemy team color range
            enemy_team = 'CT' if self.team == 'T' else 'T'
            color_range = self.enemy_color_ranges[enemy_team]
            
            # create mask
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            
            # morphological operations
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # clear previous detection results
            self.detected_enemies = []
            
            # create debug image
            debug_image = image.copy()
            
            # add debug information
            cv2.putText(debug_image, f"Team: {self.team}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_enemy_area:
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # calculate confidence
                    aspect_ratio = float(w) / h
                    confidence = area / (image.shape[0] * image.shape[1])
                    
                    # add additional shape verification
                    if 0.5 < aspect_ratio < 2.0 and confidence > self.enemy_detection_threshold:
                        # calculate target center
                        center_x = x + w/2
                        center_y = y + h/2
                        
                        # calculate relative position in image
                        relative_x = (center_x - image.shape[1]/2) / image.shape[1]
                        relative_y = (center_y - image.shape[0]/2) / image.shape[0]
                        
                        # estimate distance (based on target size)
                        estimated_distance = 5.0 * math.sqrt(image.shape[0] * image.shape[1] / (w * h))
                        
                        # create target point
                        enemy_point = Point()
                        enemy_point.x = self.position.position.x + estimated_distance * math.cos(self.orientation + relative_x * math.pi/3)
                        enemy_point.y = self.position.position.y + estimated_distance * math.sin(self.orientation + relative_x * math.pi/3)
                        enemy_point.z = 0.0
                        
                        self.detected_enemies.append(enemy_point)
                        
                        # draw detection box and information on debug image
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(debug_image, 
                                  f'Enemy D:{estimated_distance:.1f}m C:{confidence:.3f}',
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # add debug information
                        rospy.loginfo(f"Enemy detected: Distance={estimated_distance:.2f}m, "
                                    f"Confidence={confidence:.3f}, Area={area}")
            
                # add detection count on debug image
            cv2.putText(debug_image, f"Detected: {len(self.detected_enemies)}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
                
                # publish processed mask image for debugging
                mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
                self.debug_mask_pub.publish(mask_msg)
            except Exception as e:
                rospy.logerr(f"Error publishing debug image: {e}")
                
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

    def reload_weapon(self):
        """reload weapon"""
        if not self.reload_state:
            self.reload_state = True
            self.last_shot_time = rospy.Time.now().to_sec()

    def can_shoot(self):
        """check if can shoot"""
        current_time = rospy.Time.now().to_sec()
        if self.reload_state:
            if current_time - self.last_shot_time >= self.weapon.value['reload_time']:
                self.reload_state = False
                self.ammo = 30
                return True
            return False
        return self.ammo > 0

    def shoot(self):
        """execute shooting"""
        if self.can_shoot():
            self.ammo -= 1
            if self.ammo <= 0:
                self.reload_weapon()
            return True
        return False

    def publish_state(self):
        """publish robot state"""
        msg = RobotStateMsg()
        msg.robot_name = self.robot_name
        msg.team = self.team
        msg.health = self.health
        msg.position = self.position
        msg.is_alive = self.is_alive
        msg.weapon_type = self.weapon.name
        msg.is_planting = self.is_planting
        msg.is_defusing = self.is_defusing
        self.state_pub.publish(msg)

    def handle_combat(self):
        """handle combat state"""
        if self.detected_enemies:
            # select closest enemy as target
            closest_enemy = min(self.detected_enemies, 
                              key=lambda p: math.hypot(p.x - self.position.position.x,
                                                     p.y - self.position.position.y))
            
            dx = closest_enemy.x - self.position.position.x
            dy = closest_enemy.y - self.position.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # calculate target angle
            target_angle = math.atan2(dy, dx)
            angle_diff = math.atan2(math.sin(target_angle - self.orientation),
                                  math.cos(target_angle - self.orientation))
            
            cmd = Twist()
            
            # if in weapon range
            if distance <= self.weapon.value['range']:
                # aim at target
                if abs(angle_diff) < self.angle_tolerance:
                    # shoot
                    if self.shoot():
                        rospy.loginfo(f"{self.robot_name} shooting at enemy")
                else:
                    # turn to target
                    cmd.angular.z = self.Kp_angular * angle_diff
            else:
                # move towards target
                cmd.angular.z = self.Kp_angular * angle_diff
                if abs(angle_diff) < self.angle_tolerance:
                    cmd.linear.x = self.Kp_linear * distance
            
            # limit speed
            cmd.linear.x = min(max(cmd.linear.x, -0.5), 0.5)
            cmd.angular.z = min(max(cmd.angular.z, -1.0), 1.0)
            
            if not self.avoiding:
                self.cmd_vel_pub.publish(cmd)

    def run(self):
        """main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
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

if __name__ == '__main__':
    try:
        controller = CSRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
