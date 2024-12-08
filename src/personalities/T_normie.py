from transitions import Machine
from robot_controller import CSRobotController
import rospy
from move_base_msgs.msg import MoveBaseGoal

# TODO: make it work

class TNormie(CSRobotController):
    states = ['patrolling', 'engaging', 'attacking_site', 'planting_bomb', 'bomb_being_defused', 'retreating']

    def __init__(self):
        super().__init__()

        # Initialize the state machine
        self.machine = Machine(     # this replaces the state machine in parent class (self.state)
            model=self,
            states=TNormie.states,
            initial='attacking_site'
        )

        # Define transitions
        self.machine.add_transition(
            trigger='enemy_spotted',
            source=['patrolling', 'attacking_site'],
            dest='engaging'
        )

        self.machine.add_transition(
            trigger='enemy_lost',
            source='engaging',
            dest='patrolling'
        )

        self.machine.add_transition(
            trigger='bomb_planted',
            source=['attacking_site'],
            dest='attacking_site'
        )

        self.machine.add_transition(
            trigger='bomb_being_defused',
            source=['patrolling'],
            dest='defusing'
        )

        self.machine.add_transition(
            trigger='defuse_complete',
            source='defusing',
            dest='patrolling'
        )

    def run(self):
        """Override the run loop with state machine logic"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown() and self.is_alive:
            self.publish_state()

            # State-specific behaviors
            if self.state == 'patrolling':      # after bomb is planted, robot will patrol the site
                if not self.is_patrolling:
                    self.is_patrolling = True
                    self.send_next_patrol_point()   # moves to next patrol point

            elif self.state == 'engaging':
                self.is_patrolling = False
                if self.current_goal_active:
                    self.move_base.cancel_goal()
                # TODO: Combat logic here

            elif self.state == 'attacking_site':
                self.is_patrolling = False
                # TODO: Implement site defense behavior

            elif self.state == 'planting_bomb':
                self.is_patrolling = False
                self.start_defusing()
                # TODO: Plant bomb

            elif self.state == 'bomb_being_defused':
                self.is_patrolling = False
                # move to bomb
                if self.current_goal_active:
                    self.move_base.cancel_goal()  # Cancel any existing movement
                
                # Create a new goal to move to the bomb site
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                
                # Use first bomb site corner as target location
                goal.target_pose.pose.position = self.bomb_sites[0]
                goal.target_pose.pose.orientation.w = 1.0
                
                # Send goal to move_base
                self.move_base.send_goal(goal)
                self.current_goal_active = True

                # TODO: Defuse bomb

            elif self.state == 'retreating':
                self.is_patrolling = False
                # TODO: Implement retreat behavior

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TNormie()
        controller.run()
    except rospy.ROSInterruptException:
        pass
