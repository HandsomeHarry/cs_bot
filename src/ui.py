#!/usr/bin/env python
# coding=utf-8

import rospy
from python_qt_binding import QtCore, QtGui, QtWidgets
from cs_bot.msg import GameStateMsg, RobotStateMsg
import sys

class GameUI(QtWidgets.QWidget):
    def __init__(self):
        super(GameUI, self).__init__()
        rospy.init_node('game_ui', anonymous=True)
        
        # Add publisher for game control
        self.game_control_pub = rospy.Publisher('/game/control', String, queue_size=1)
        
        self.initUI()
        
        # Store robot states
        self.robot_states = {}
        
        # Subscribe to game state and robot states
        rospy.Subscriber('/game/state', GameStateMsg, self.update_game_state)
        rospy.Subscriber('/game/robot_states', RobotStateMsg, self.robot_state_callback)
        
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
        
        # Add reset button
        self.reset_button = QtWidgets.QPushButton('Reset Round')
        self.reset_button.clicked.connect(self.reset_round)
        game_layout.addWidget(self.reset_button)
        
        # Existing game state widgets
        self.round_timer = QtWidgets.QLabel('Round Timer: 90s')
        self.game_phase = QtWidgets.QLabel('Phase: PREP')
        self.bomb_status = QtWidgets.QLabel('Bomb: Not Planted')
        game_layout.addWidget(self.round_timer)
        game_layout.addWidget(self.game_phase)
        game_layout.addWidget(self.bomb_status)
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
        """Update game state information from GameStateMsg"""
        self.current_game_state = msg

    def robot_state_callback(self, msg):
        """Update robot state information from RobotStateMsg"""
        self.robot_states[msg.robot_name] = {
            'state': msg,
            'last_update': rospy.Time.now()
        }
        
    def update_ui(self):
        """Update UI elements"""
        try:
            # Update game state
            if hasattr(self, 'current_game_state'):
                self.round_timer.setText(f'Round Timer: {self.current_game_state.round_time_remaining}s')
                self.game_phase.setText(f'Phase: {self.current_game_state.game_phase}')
                bomb_status = "Planted" if self.current_game_state.bomb_planted else "Not Planted"
                self.bomb_status.setText(f'Bomb: {bomb_status}')
            
            # Update robot states
            current_time = rospy.Time.now()
            for robot_name, data in self.robot_states.items():
                state = data['state']
                last_update = data['last_update']
                
                # Create or update robot status widgets
                if not hasattr(self, f'robot_label_{robot_name}'):
                    label = QtWidgets.QLabel()
                    self.robot_layout.addWidget(label)
                    setattr(self, f'robot_label_{robot_name}', label)
                
                # Update label
                label = getattr(self, f'robot_label_{robot_name}')
                team = state.team
                health_color = "green" if state.health > 50 else "orange" if state.health > 25 else "red"
                alive_status = "ALIVE" if state.is_alive else "DEAD"
                time_diff = (current_time - last_update).to_sec()
                
                # Special states
                special_status = []
                if state.is_planting:
                    special_status.append("PLANTING")
                if state.is_defusing:
                    special_status.append("DEFUSING")
                status_str = f" ({', '.join(special_status)})" if special_status else ""
                
                label.setText(
                    f'<div style="margin:5px; padding:5px; border:1px solid gray;">'
                    f'<b>{robot_name}</b> ({team})<br>'
                    f'Health: <span style="color:{health_color}">{state.health}</span><br>'
                    f'Status: {alive_status}{status_str}<br>'
                    f'Weapon: {state.weapon_type}<br>'
                    f'<span style="color:gray; font-size:smaller">Updated: {time_diff:.1f}s ago</span>'
                    f'</div>'
                )
                
        except Exception as e:
            rospy.logerr(f"Error updating UI: {str(e)}")

    def reset_round(self):
        """Reset the round when button is clicked"""
        try:
            self.game_control_pub.publish("RESET_ROUND")
            rospy.loginfo("Reset round command sent")
        except Exception as e:
            rospy.logerr(f"Failed to send reset command: {str(e)}")

if __name__ == '__main__':
    try:
        app = QtWidgets.QApplication(sys.argv)
        ui = GameUI()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass