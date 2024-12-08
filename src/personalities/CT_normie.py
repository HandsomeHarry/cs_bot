from transitions import Machine
from robot_controller import CSRobotController
import rospy

class CTNormie(CSRobotController):
    states = ['patrolling', 'engaging', 'defending_site', 'defusing', 'retreating']

    def __init__(self):
        super().__init__()

        # Initialize the state machine
        self.machine = Machine(
            model=self,
            states=CTNormie.states,
            initial='patrolling'
        )

        # Define transitions
        self.machine.add_transition(
            trigger='enemy_spotted',
            source=['patrolling', 'defending_site'],
            dest='engaging'
        )

        self.machine.add_transition(
            trigger='enemy_lost',
            source='engaging',
            dest='patrolling'
        )

        self.machine.add_transition(
            trigger='bomb_planted',
            source=['patrolling', 'engaging'],
            dest='defending_site'
        )

        self.machine.add_transition(
            trigger='start_defuse',
            source=['defending_site', 'patrolling'],
            dest='defusing'
        )

        self.machine.add_transition(
            trigger='defuse_complete',
            source='defusing',
            dest='patrolling'
        )

        self.machine.add_transition(
            trigger='health_low',
            source=['engaging', 'defending_site'],
            dest='retreating'
        )

        self.machine.add_transition(
            trigger='health_recovered',
            source='retreating',
            dest='patrolling'
        )

    def run(self):
        """Override the run loop with state machine logic"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown() and self.is_alive:
            self.publish_state()

            # State machine transitions based on conditions
            if self.health < 30:
                self.health_low()
            elif self.health > 50 and self.state == 'retreating':
                self.health_recovered()

            # State-specific behaviors
            if self.state == 'patrolling':
                if not self.is_patrolling:
                    self.is_patrolling = True
                    self.send_next_patrol_point()

            elif self.state == 'engaging':
                self.is_patrolling = False
                if self.current_goal_active:
                    self.move_base.cancel_goal()
                # Combat logic here

            elif self.state == 'defending_site':
                self.is_patrolling = False
                # Implement site defense behavior

            elif self.state == 'defusing':
                self.is_patrolling = False
                self.start_defusing()
                # Implement defuse behavior

            elif self.state == 'retreating':
                self.is_patrolling = False
                # Implement retreat behavior

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = CTNormie()
        controller.run()
    except rospy.ROSInterruptException:
        pass
