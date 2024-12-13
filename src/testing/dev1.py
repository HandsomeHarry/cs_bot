#!/usr/bin/env python
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MultiRobotNavigator:
    def __init__(self):
        rospy.init_node('devious_navigator')
        
        # Initialize action clients for both robots
        self.robots = {
            'robot1': self.setup_robot('robot1'),
            'robot2': self.setup_robot('robot2')
        }
        
        rospy.loginfo("Navigation system ready for both robots")

    def setup_robot(self, robot_name):
        """Setup move_base action client for a robot"""
        client = actionlib.SimpleActionClient(
            f'/{robot_name}/move_base',
            MoveBaseAction
        )
        rospy.loginfo(f"Waiting for {robot_name} move_base server...")
        client.wait_for_server()
        rospy.loginfo(f"Connected to {robot_name} move_base server")
        return client

    def move_robot(self, robot_name, x, y):
        """Send a movement goal for a specific robot"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending {robot_name} to: x={x}, y={y}")
        self.robots[robot_name].send_goal(goal)

    def wait_for_results(self):
        """Wait for both robots to complete their movements"""
        results = {}
        for robot_name, client in self.robots.items():
            client.wait_for_result()
            state = client.get_state()
            success = (state == actionlib.GoalStatus.SUCCEEDED)
            results[robot_name] = success
            status = "succeeded" if success else "failed"
            rospy.loginfo(f"{robot_name} movement {status}")
        return results

if __name__ == '__main__':
    try:
        navigator = MultiRobotNavigator()
        
        # Example movement commands
        # Robot 1 goes to point A, Robot 2 goes to point B
        navigator.move_robot('robot1', 3.0, 4.0)
        navigator.move_robot('robot2', -3.0, -4.0)
        
        # Wait for both robots to complete their movements
        results = navigator.wait_for_results()
        
        rospy.loginfo("Navigation test completed")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted")
