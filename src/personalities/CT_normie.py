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

    def handle_engaging(self):
        """Handle the engaging state behavior when fighting enemies"""
        rospy.loginfo("In engaging state...")
        self.shoot_enemy()      # how to shoot enemy?
        pass

    def handle_defending_site(self):
        """Handle the defending site behavior when protecting bomb sites"""
        rospy.loginfo("In defending site state...")
        self.move_to_position(self.bomb_site)
        pass

    def handle_defusing(self):
        """Handle the defusing state behavior when attempting to defuse the bomb"""
        rospy.loginfo("In defusing state...")
        
        pass

    def handle_retreating(self):
        """Handle the retreating state behavior when falling back"""
        rospy.loginfo("In retreating state...")
        pass

    def handle_dead(self):
        """Handle the dead state behavior"""
        rospy.loginfo("In dead state...")
        pass

    def handle_resetting(self):
        """Handle the resetting state behavior between rounds"""
        rospy.loginfo("In resetting state...")
        pass

    def handle_waiting(self):
        """Handle the waiting state behavior"""
        rospy.loginfo("In waiting state...")
        pass

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
                elif self.state == 'defending_site':
                    self.handle_defending_site()
                elif self.state == 'defusing':
                    self.handle_defusing()
                elif self.state == 'retreating':
                    self.handle_retreating()
                elif self.state == 'dead':
                    self.handle_dead()
                elif self.state == 'resetting':
                    self.handle_resetting()
                elif self.state == 'waiting':
                    self.handle_waiting()

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