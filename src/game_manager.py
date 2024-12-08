#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
from cs_bot.msg import GameStateMsg, RobotStateMsg, CombatEvent

class GameManager:
    def __init__(self):
        rospy.init_node('game_manager', anonymous=True)
        
        # game state
        self.round_time = 90
        self.round_active = False
        self.bomb_planted = False
        self.bomb_time = 40
        self.bomb_location = Point()
        self.dead_players = []
        self.robot_states = {}
        self.publishers = {}
        
        # publisher
        self.game_state_pub = rospy.Publisher('/game/state', GameStateMsg, queue_size=1)
        # subscriber
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.robot_state_callback)
        rospy.Subscriber('/game/shoot', CombatEvent, self.handle_combat)
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        rospy.Timer(rospy.Duration(0.1), self.publish_game_state)

    def robot_state_callback(self, msg):
        """manage robot state"""
        self.robot_states[msg.robot_name] = msg

    def handle_combat(self, msg):
        target = msg.target_name
        if target in self.robot_states:
            self.robot_states[target].health -= msg.damage_dealt  # Decrease health
            if self.robot_states.get(target).health <= 0:
                self.robot_states[target].health = 0
                self.robot_states[target].is_alive = False  # kill
                self.dead_players.append(target)
        else:
            print('Could not find robot ' + target)
      
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

    def update_robot_states(self):
        """timer callback, update each player"""
        for robot_name, robot_state in self.robot_states.items():
            # Create a publisher if not already created
            if robot_name not in self.publishers:
                topic = f'/{robot_name}/robot_state'
                self.publishers[robot_name] = rospy.Publisher(topic, RobotStateMsg, queue_size=1)
            
            # Publish the robot state
            self.publishers[robot_name].publish(robot_state)
        
    def publish_game_state(self, event):
        """publish game state"""
        msg = GameStateMsg()
        msg.round_time_remaining = self.round_time
        msg.game_phase = self.get_game_phase()
        msg.bomb_planted = self.bomb_planted
        msg.bomb_location = self.bomb_location
        msg.dead_players = self.dead_players
        self.game_state_pub.publish(msg)
        self.update_robot_states()
        
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
