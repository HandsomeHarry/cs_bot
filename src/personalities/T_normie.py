from transitions import Machine
from geometry_msgs.msg import Point
import rospy
import random
import time
from cs_robot_controller import CSRobotController

class TNormie(CSRobotController):
    states = [
        'going_to_bomb_site',
        'engaging_enemy',
        'planting_bomb',
        'protecting_bomb'
    ]

    def __init__(self):
        super().__init__()

        self.machine = Machine(
            model=self,
            states=TNormie.states,
            initial='going_to_bomb_site',
            before_state_change='on_exit_state',
            after_state_change='on_enter_state'
        )

        self.machine.add_transition('enemy_detected', 'going_to_bomb_site', 'engaging_enemy', conditions=['is_enemy_visible'])
        self.machine.add_transition('enemy_lost', 'engaging_enemy', 'going_to_bomb_site', unless=['is_enemy_visible'])
        self.machine.add_transition('arrived_at_bomb_site', 'going_to_bomb_site', 'planting_bomb', conditions=['is_near_bomb_site'])
        self.machine.add_transition('bomb_planted', 'planting_bomb', 'protecting_bomb')

    def on_enter_state(self):
        rospy.loginfo(f"Entering state: {self.state}")

    def on_exit_state(self):
        rospy.loginfo(f"Exiting state: {self.state}")

    def is_enemy_visible(self):
        return self.enemy_spotted

    def is_near_bomb_site(self):
        return self.is_near_position(self.bomb_location, threshold=0.5)

    def find_closest_bomb_site(self):
        robot_position = self.position.position
        closest_site = None
        closest_distance = float('inf')

        for site in self.bomb_sites:
            dx = robot_position.x - site.x
            dy = robot_position.y - site.y
            distance = (dx**2 + dy**2)**0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_site = site

        return closest_site

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.is_alive:
            self.publish_state()

            if self.state == 'going_to_bomb_site':
                if not self.enemy_spotted:
                    closest_bomb_site = self.find_closest_bomb_site()
                    self.move_to_position(closest_bomb_site)
                else:
                    self.cancel_movement()
                    self.enemy_detected()

            elif self.state == 'engaging_enemy':
                if not self.enemy_spotted:
                    self.enemy_lost()
                else:
                    self.cancel_movement()

            elif self.state == 'planting_bomb':
                if self.is_near_position(self.bomb_location, threshold=0.5):
                    self.cancel_movement()
                    self.start_planting()
                    rospy.loginfo("Planting bomb...")
                    self.bomb_event_pub.publish("BOMB_PLANTED")
                    time.sleep(5)  # Simulate planting time
                    self.stop_planting()
                    self.bomb_planted()

            elif self.state == 'protecting_bomb':
                rospy.loginfo("Protecting the bomb. Scanning the area...")
                # Implement logic to scan and engage enemies around the bomb

            rate.sleep()

if __name__ == '__main__':
    try:
        robot = TNormie()
        robot.run()
    except rospy.ROSInterruptException:
        pass
