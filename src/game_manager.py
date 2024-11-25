#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
from cs_bot.msg import GameStateMsg, RobotStateMsg

class GameManager:
    def __init__(self):
        rospy.init_node('game_manager', anonymous=True)
        
        # game state
        self.round_time = 90
        self.round_active = False
        self.bomb_planted = False
        self.bomb_time = 40
        self.bomb_location = Point()
        
        # player state
        self.t_players = []
        self.ct_players = []
        self.robot_states = {}  # store robot states
        
        # publisher
        self.game_state_pub = rospy.Publisher('/game/state', GameStateMsg, queue_size=1)
        
        # subscriber
        self.robot_state_sub = rospy.Subscriber('/game/robot_states', RobotStateMsg, self.robot_state_callback)
        
        # timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        
    def robot_state_callback(self, msg):
        """manage robot state"""
        self.robot_states[msg.robot_name] = msg
        self.update_team_lists()
        
    def update_team_lists(self):
        """update team member lists"""
        self.t_players = []
        self.ct_players = []
        for name, state in self.robot_states.items():
            if state.is_alive:
                if state.team == 'T':
                    self.t_players.append(name)
                else:
                    self.ct_players.append(name)
                    
    def timer_callback(self, event):
        """timer callback, update game state"""
        if self.round_active:
            if self.bomb_planted:
                self.bomb_time -= 1
                if self.bomb_time <= 0:
                    self.end_round("Terrorists")  # bomb exploded, T wins
            else:
                self.round_time -= 1
                if self.round_time <= 0:
                    self.end_round("Counter-Terrorists")  # time runs out, CT wins
                    
        self.publish_game_state()
        
    def publish_game_state(self):
        """publish game state"""
        msg = GameStateMsg()
        msg.round_time_remaining = self.round_time
        msg.game_phase = self.get_game_phase()
        msg.bomb_planted = self.bomb_planted
        msg.bomb_location = self.bomb_location
        msg.alive_t_players = self.t_players
        msg.alive_ct_players = self.ct_players
        self.game_state_pub.publish(msg)
        
    def get_game_phase(self):
        """get current game phase"""
        if not self.round_active:
            return "PREP"
        elif self.bomb_planted:
            return "BOMB_PLANTED"
        else:
            return "ACTIVE"
            
    def end_round(self, winner):
        """end round"""
        self.round_active = False
        rospy.loginfo("%s win" % winner)
        self.reset_round()
        
    def reset_round(self):
        """reset round state"""
        self.round_time = 90
        self.bomb_time = 40
        self.bomb_planted = False
        self.bomb_location = Point()
        
    def start_round(self):
        """start new round"""
        self.round_active = True
        self.reset_round()
        rospy.loginfo("New round started")

if __name__ == '__main__':
    try:
        game_manager = GameManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
