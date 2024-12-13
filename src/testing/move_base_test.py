#!/usr/bin/env python
import rospy
import actionlib
import time
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Patrol:
  def __init__(self, name):
    self.name = name
    self.client = actionlib.SimpleActionClient(f'/{self.name}/move_base', MoveBaseAction)
    self.client.wait_for_server()
  
  def execute(self):
    goal = self.__set_goal_pose(pose)
    print("Going for goal: ", goal)
    self.client.send_goal(goal)
    self.client.wait_for_result()
  
  #=======Callback Methods=======

  def cancel_movement(self):
    self.client.cancel_goal()

  def __set_goal_pose(self, pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose
  
  def __parse_message(self, msg):
    split_msg = msg.data.split(', ')
    return int(split_msg[0]), split_msg[1]

    time.sleep(1)