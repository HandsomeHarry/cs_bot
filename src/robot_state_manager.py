#!/usr/bin/env python
# coding=utf-8

import rospy
from cs_bot.msg import RobotStateMsg, GameStateMsg
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class RobotStateManager:
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        self.team = rospy.get_param('~team', 'T')
        
        # robot state
        self.health = 100
        self.is_alive = True
        self.weapon_type = 'rifle'  # default weapon
        self.is_planting = False
        self.is_defusing = False
        self.position = Pose()
        
        # publisher
        self.state_pub = rospy.Publisher('/game/robot_states', RobotStateMsg, queue_size=10)
        
        # subscribers
        self.game_state_sub = rospy.Subscriber('/game/state', GameStateMsg, self.game_state_callback)
        self.position_sub = rospy.Subscriber('/%s/pose' % self.robot_name, Pose, self.position_callback)
        
        # state update timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_state)
        
    def game_state_callback(self, msg):
        """process game state update"""
        # check if self is in alive list
        if self.team == 'T':
            self.is_alive = self.robot_name in msg.alive_t_players
        else:
            self.is_alive = self.robot_name in msg.alive_ct_players
            
    def position_callback(self, msg):
        """update robot position"""
        self.position = msg
        
    def publish_state(self, event):
        """publish robot state"""
        msg = RobotStateMsg()
        msg.robot_name = self.robot_name
        msg.team = self.team
        msg.health = self.health
        msg.position = self.position
        msg.is_alive = self.is_alive
        msg.weapon_type = self.weapon_type
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

if __name__ == '__main__':
    try:
        rospy.init_node('robot_state_manager', anonymous=True)
        robot_state_manager = RobotStateManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
