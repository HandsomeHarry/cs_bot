#!/usr/bin/env python

import rospy
import yaml
import random
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
import time
from nav_msgs.msg import OccupancyGrid

class Demo:
    def __init__(self):
        rospy.init_node('demo_controller')
        rospy.loginfo("Initializing demo controller")
        # Load config
        config_path = os.path.join(os.path.dirname(__file__), '../config/map_config.yaml')
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Initialize move_base clients for both robots
        self.robots = {
            'robot1': actionlib.SimpleActionClient('/robot1/move_base', MoveBaseAction),
            'robot2': actionlib.SimpleActionClient('/robot2/move_base', MoveBaseAction)
        }

        # Wait for move_base servers
        for robot_name, client in self.robots.items():
            rospy.loginfo(f"Waiting for {robot_name} move_base...")
            client.wait_for_server()

        # Store initial positions (using CT spawn points for demo)
        self.initial_positions = {
            'robot1': self.config['spawn_points']['CT'][0],
            'robot2': self.config['spawn_points']['CT'][1]
        }

        # Track which robots have reached the goal
        self.reached_goal = {
            'robot1': False,
            'robot2': False
        }

    def create_goal(self, point):
        """Create a MoveBaseGoal from a point dictionary"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point['x']
        goal.target_pose.pose.position.y = point['y']
        goal.target_pose.pose.position.z = point['z']
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def goal_callback(self, robot_name, status, result):
        """Callback for when a robot reaches its goal"""
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"{robot_name} reached the goal!")
            self.reached_goal[robot_name] = True

    def move_robots(self, target_point):
        """Send both robots to the target point"""
        self.reached_goal = {'robot1': False, 'robot2': False}
        
        # Send goals to both robots
        for robot_name, client in self.robots.items():
            goal = self.create_goal(target_point)
            client.send_goal(
                goal,
                done_cb=lambda status, result, rn=robot_name: self.goal_callback(rn, status, result)
            )

        # Wait for first robot to reach goal
        while not any(self.reached_goal.values()) and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Cancel remaining robot's goal
        for robot_name, client in self.robots.items():
            if not self.reached_goal[robot_name]:
                client.cancel_goal()

        rospy.loginfo("First robot reached goal! Waiting 3 seconds...")
        rospy.sleep(3)

        # Send both robots back to their initial positions
        for robot_name, client in self.robots.items():
            goal = self.create_goal(self.initial_positions[robot_name])
            client.send_goal(goal)

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Randomly select site1 or site2
            site = random.choice(['site1', 'site2'])
            target = self.config['bomb_site'][site]
            
            rospy.loginfo(f"Moving robots to {site}")
            self.move_robots(target)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        demo = Demo()
        demo.run()
    except rospy.ROSInterruptException:
        pass 