#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
from cs_bot.msg import GameStateMsg, RobotStateMsg, CombatEvent
from std_msgs.msg import String

class GameManager:
    def __init__(self):
        rospy.init_node('game_manager', anonymous=True)
        
        # game state
        self.round_time = 90
        self.round_active = False
        self.bomb_planted = False
        self.bomb_time = 30
        self.bomb_location = Point()
        self.dead_players = []
        self.robot_states = {}
        self.publishers = {}
        self.t_win_count = 0
        self.ct_win_count = 0
        self.round_number = 0  # Add round number tracking
        
        # publisher
        self.game_state_pub = rospy.Publisher('/game/state', GameStateMsg, queue_size=1)
        # subscriber
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.robot_state_callback)
        rospy.Subscriber('/game/shoot', CombatEvent, self.handle_combat)
        rospy.Subscriber('/game/bomb_events', String, self.handle_bomb_events)
        
        
        self.defusing_in_progress = False
        self.defuse_time = 5  # Time needed to defuse in seconds
        self.defuse_timer = 0
        
        # Add subscriber for game control
        rospy.Subscriber('/game/control', String, self.handle_game_control)

    def robot_state_callback(self, msg):
        """manage robot state"""
        #rospy.loginfo(f"Received robot state for: {msg.robot_name}")  # Changed to loginfo for visibility
        self.robot_states[msg.robot_name] = msg

    def handle_combat(self, msg):
        target = msg.target_name
        if target in self.robot_states:
            self.robot_states[target].health -= msg.damage_dealt  # Decrease health
            rospy.loginfo(f"{target} is hit by {msg.damage_dealt} damage")
            if self.robot_states.get(target).health <= 0:
                self.robot_states[target].health = 0
                self.robot_states[target].is_alive = False  # kill
                self.dead_players.append(target)
                rospy.loginfo(f"{target} is dead")
        else:
            print('Could not find robot ' + target)
      
    def timer_callback(self):
        """timer callback, update game state"""
        if self.round_active:
            if self.bomb_planted:
                self.bomb_time -= 1
                
                # Handle defusing
                if self.defusing_in_progress:
                    self.defuse_timer -= 1
                    if self.defuse_timer <= 0:
                        self.bomb_planted = False
                        self.defusing_in_progress = False
                        self.end_round("Counter-Terrorists")  # CT wins by defusal
                        return
                
                if self.bomb_time <= 0:
                    self.end_round("Terrorists")  # bomb exploded, T wins
            else:
                self.round_time -= 1
                if self.round_time <= 0:
                    self.end_round("Counter-Terrorists")  # time runs out, CT wins

    def update_robot_states(self):
        """timer callback, update each player"""
        if self.robot_states == {}:
            return
        #rospy.loginfo(f"Current robot states: {self.robot_states}")  # Changed to loginfo for visibility
        for robot_name, robot_state in self.robot_states.items():
            #rospy.loginfo(f"Publishing state for robot: {robot_name}")  # Changed to loginfo for visibility
            if robot_name not in self.publishers:
                topic = f'/{robot_name}/robot_state'
                self.publishers[robot_name] = rospy.Publisher(topic, RobotStateMsg, queue_size=1)
            self.publishers[robot_name].publish(robot_state)
        
    def publish_game_state(self):
        """publish game state"""
        msg = GameStateMsg()
        msg.round_time_remaining = self.round_time
        msg.game_phase = self.get_game_phase()
        msg.bomb_planted = self.bomb_planted
        msg.bomb_location = self.bomb_location
        msg.dead_players = self.dead_players
        msg.bomb_time_remaining = self.bomb_time
        msg.round_number = self.round_number
        msg.ct_score = self.ct_win_count
        msg.t_score = self.t_win_count
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
        
        if winner == "Terrorists":
            self.t_win_count += 1
        else:
            self.ct_win_count += 1
        
        rospy.loginfo("T : CT\n%d : %d" % (self.t_win_count, self.ct_win_count))
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
        self.round_number += 1  # Increment round number

        rospy.loginfo("starting round...")

        rate_timer = rospy.Rate(1)  # 1 Hz for self.timer_callback
        rate_publisher = rospy.Rate(10)  # 10 Hz for self.publish_game_state

        while self.round_active and not rospy.is_shutdown():
            self.timer_callback()
            for _ in range(10):  # Publish 10 times in one-second loop
                self.publish_game_state()
                rate_publisher.sleep()

            rate_timer.sleep()


    def handle_bomb_events(self, msg):
        """Handle bomb-related events"""
        if msg.data == "DEFUSE_START":
            self.defusing_in_progress = True
            self.defuse_timer = self.defuse_time
        elif msg.data == "DEFUSE_INTERRUPT":
            self.defusing_in_progress = False
        elif msg.data == "BOMB PLANTED":
            self.bomb_being_planted = True

    def handle_game_control(self, msg):
        """Handle game control messages"""
        if msg.data == "START_ROUND":
            self.start_round()
            self.round_active = True
            rospy.loginfo("Round reset and started")

if __name__ == '__main__':
    try:
        game_manager = GameManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
