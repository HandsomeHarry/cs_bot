from transitions import Machine
from robot_controller import CSRobotController
import rospy
from transitions.extensions import GraphMachine
from IPython.display import Image
import os

class CTNormie(CSRobotController):
    states = ['patrolling', 'engaging', 'defending_site', 'defusing', 'retreating', 'dead', 'resetting', 'waiting']

    def __init__(self):
        super().__init__()

        # Initialize the state machine
        self.machine = GraphMachine(     # this replaces the state machine in parent class (self.state)
            model=self,
            states=CTNormie.states,
            initial='patrolling',
            show_conditions=True
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
            trigger='reset',
            source='*',
            dest='resetting'
        )

        self.machine.add_transition(
            trigger='reset_complete',
            source='resetting',
            dest='waiting'
        )

    def run(self):
        """Override the run loop with state machine logic"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown() and self.is_alive:
            self.publish_state()

            #'patrolling', 'engaging', 'defending_site', 'defusing', 'retreating'

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

            # After death and before next round
            # 'dead', 'resetting', 'waiting'
            elif self.state == 'dead':
                # Implement dead behavior
                self.is_patrolling = False
                self.stay_put()

            elif self.state == 'resetting':
                # Implement resetting behavior
                self.move_to_position(self.spawn_point)

            elif self.state == 'waiting':
                # Implement waiting behavior
                self.is_patrolling = False
                self.stay_put()

            rate.sleep()

    def draw_diagram(self):
        """Generate a PNG image of the state machine"""
        # Draw the diagram
        self.graph.draw('ct_normie_states.png', prog='dot')
        
        # Optional: Display the image if in Jupyter notebook
        return Image('ct_normie_states.png')

if __name__ == '__main__':
    try:
        controller = CTNormie()
        controller.draw_diagram()  # This will save the diagram
        controller.run()
    except rospy.ROSInterruptException:
        pass
