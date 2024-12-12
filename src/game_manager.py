#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
from cs_bot.msg import GameStateMsg, RobotStateMsg, CombatEvent
from std_msgs.msg import String
from python_qt_binding import QtCore, QtGui, QtWidgets
import sys

class GameManager:
    def __init__(self):
        rospy.init_node('game_manager', anonymous=True)
        
        # game state
        self.round_time = 90
        self.round_active = True
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
        rospy.Subscriber('/game/bomb_events', String, self.handle_bomb_events)
        
        self.defusing_in_progress = False
        self.defuse_time = 5  # Time needed to defuse in seconds
        self.defuse_timer = 0

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
        rospy.loginfo("round started")

    def handle_bomb_events(self, msg):
        """Handle bomb-related events"""
        if msg.data == "DEFUSE_START":
            self.defusing_in_progress = True
            self.defuse_timer = self.defuse_time
        elif msg.data == "DEFUSE_INTERRUPT":
            self.defusing_in_progress = False

class GameUI(QtWidgets.QWidget):
    def __init__(self):
        super(GameUI, self).__init__()
        rospy.init_node('game_ui', anonymous=True)
        self.initUI()
        
        # Subscribe to game state and robot states
        rospy.Subscriber('/game/state', GameStateMsg, self.update_game_state)
        self.robot_states = {}
        
        # Timer for UI updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Update every 100ms

    def initUI(self):
        self.setWindowTitle('CS Bot Game Status')
        layout = QtWidgets.QVBoxLayout()
        
        # Game state section
        game_group = QtWidgets.QGroupBox('Game State')
        game_layout = QtWidgets.QVBoxLayout()
        self.round_timer = QtWidgets.QLabel('Round Timer: 90')
        self.game_phase = QtWidgets.QLabel('Phase: PREP')
        game_layout.addWidget(self.round_timer)
        game_layout.addWidget(self.game_phase)
        game_group.setLayout(game_layout)
        
        # Robot states section
        robot_group = QtWidgets.QGroupBox('Robot Status')
        self.robot_layout = QtWidgets.QVBoxLayout()
        robot_group.setLayout(self.robot_layout)
        
        layout.addWidget(game_group)
        layout.addWidget(robot_group)
        self.setLayout(layout)
        
        # Set window size and position
        self.resize(300, 400)
        self.show()

    def update_game_state(self, msg):
        """Update game state information"""
        self.current_game_state = msg

    def update_robot_state(self, msg):
        """Update individual robot state"""
        self.robot_states[msg.robot_name] = msg
        
    def update_ui(self):
        """Update UI elements"""
        try:
            # Update game state
            if hasattr(self, 'current_game_state'):
                self.round_timer.setText(f'Round Timer: {self.current_game_state.round_time_remaining}s')
                self.game_phase.setText(f'Phase: {self.current_game_state.game_phase}')
            
            # Update robot states
            for robot_name, state in self.robot_states.items():
                # Create or update robot status widgets
                if not hasattr(self, f'robot_label_{robot_name}'):
                    label = QtWidgets.QLabel()
                    self.robot_layout.addWidget(label)
                    setattr(self, f'robot_label_{robot_name}', label)
                
                # Update label with team, health, and gun
                label = getattr(self, f'robot_label_{robot_name}')
                team = "T" if "1" in robot_name.lower() else "CT"
                health_color = "green" if state.health > 50 else "orange" if state.health > 25 else "red"
                label.setText(
                    f'<div style="margin:5px; padding:5px; border:1px solid gray;">'
                    f'<b>{robot_name}</b> ({team})<br>'
                    f'Health: <span style="color:{health_color}">{state.health}</span><br>'
                    f'Gun: {state.current_gun}'
                    f'</div>'
                )
                
        except Exception as e:
            rospy.logerr(f"Error updating UI: {str(e)}")

if __name__ == '__main__':
    try:
        app = QtWidgets.QApplication(sys.argv)
        ui = GameUI()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
