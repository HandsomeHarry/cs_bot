#!/usr/bin/env python

import rospy
import sys
import os
from random import random
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_controller import CSRobotController
from transitions import Machine

class CTNormie(CSRobotController):
    states = [
        'patrolling',
        'engaging',
        'defending_site',
        'defusing',
        'retreating',
        'dead',
        'resetting',
        'waiting'
    ]

    def __init__(self):
        super().__init__()

        self.is_patrolling = False
        self.current_goal_active = False
        self.spawn_point = None
        self.is_alive = True

        self.machine = Machine(
            model=self,
            states=[
                {'name': 'patrolling', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'engaging', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'defending_site', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'defusing', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'retreating', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'dead', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'resetting', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'},
                {'name': 'waiting', 'on_enter': 'on_state_enter', 'on_exit': 'on_state_exit'}
            ],
            initial='patrolling'
        )

        self.machine.add_transition('enemy_spotted', ['patrolling', 'defending_site'], 'engaging', conditions=['is_enemy_visible'])
        self.machine.add_transition('enemy_lost', 'engaging', 'patrolling')
        self.machine.add_transition('bomb_planted', ['patrolling', 'engaging'], 'defending_site')
        self.machine.add_transition('start_defuse', ['defending_site', 'patrolling'], 'defusing')
        self.machine.add_transition('defuse_complete', 'defusing', 'patrolling')
        self.machine.add_transition('reset', '*', 'resetting')
        self.machine.add_transition('reset_complete', 'resetting', 'waiting')

    def on_state_enter(self):
        rospy.loginfo(f"Entering state: {self.state}")

    def on_state_exit(self):
        rospy.loginfo(f"Exiting state: {self.state}")

    def is_enemy_visible(self):
        return random() > 0.5  # Placeholder for real detection

    def publish_state(self):
        rospy.loginfo(f"Current state: {self.state}")

    def handle_patrolling(self):
        if not self.is_patrolling:
            self.is_patrolling = True
            try:
                self.send_next_patrol_point()
            except Exception as e:
                rospy.logerr(f"Error in patrolling: {e}")
                self.is_patrolling = False

    def run(self):
        if not rospy.is_initialized():
            rospy.init_node('ct_normie_controller')
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.is_alive:
            try:
                self.publish_state()

                if self.state == 'patrolling':
                    self.handle_patrolling()
                elif self.state == 'engaging':
                    self.handle_engaging()

            except Exception as e:
                rospy.logerr(f"Error in run loop: {e}")
                self.reset()

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = CTNormie()
        controller.run()
    except rospy.ROSInterruptException:
        pass