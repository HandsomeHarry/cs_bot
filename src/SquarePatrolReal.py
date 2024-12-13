import rospy
from geometry_msgs.msg import Twist
import time
from cs_robot_controller import CSRobotController

class SquarePatrolRobot():
    def __init__(self):
        super().__init__()
        self.square_side_length = 0.25  # 0.5 meters
        self.speed = 0.1  # Linear speed in m/s
        self.rotation_speed = 0.35  # Angular speed in rad/s
        self.side_duration = self.square_side_length / self.speed  # Time to travel one side
        self.turn_duration = (3.14159 / 2) / self.rotation_speed  # Time to turn 90 degrees
        self.rate = rospy.Rate(10)  # 10 Hz

    def move_forward(self):
        
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.publish_state()
            if not self.is_alive:
                rate.sleep()
                break
            if self.game_phase == "PREP":
                self.is_patrolling = False
                if not self.is_at_spawn:
                    self.move_to_position(self.spawn_points[self.team])
                    for spawn_point in self.spawn_points[self.team]:
                        if self.is_near_position(spawn_point, threshold=0.3):
                            self.is_at_spawn = True
                            self.cancel_movement()

            elif self.game_phase == "ACTIVE":
                self.is_at_spawn = False
                if self.team == "CT":
                    self.patrol()
                else: # T
                    self.move_to_position(self.bomb_sites[0])
                    if self.is_near_position(self.bomb_sites[0], threshold=0.3):
                        self.start_planting()
                        rospy.loginfo("planting!")
                        # Notify game manager that planting has started
                        self.bomb_event_pub.publish("PLANT_START")
                
            elif self.game_phase == "BOMB_PLANTED":
                if self.team == "CT":
                    self.is_patrolling = False
                    self.move_to_position(self.bomb_location)
                    # When close to bomb, start defusing
                    if self.is_near_position(self.bomb_location, threshold=0.5):
                        self.start_defusing()
                        # Notify game manager that defusing has started
                        self.bomb_event_pub.publish("DEFUSE_START")
            rate.sleep()
    def patrol(self):
        for _ in range(4):  # Four sides of the square
            # Move forward
            start_time = time.time()
            while time.time() - start_time < self.side_duration:
                self.move_forward()
                self.rate.sleep()

            # Stop briefly
            self.stop()
            rospy.sleep(1.5)

            # Turn 90 degrees
            start_time = time.time()
            while time.time() - start_time < self.turn_duration:
                self.turn()
                self.rate.sleep()

            # Stop briefly
            self.stop()
            rospy.sleep(0.5)

        rospy.loginfo(f"Completed one square patrol cycle for {self.robot_name}")

if __name__ == '__main__':
    try:
        robot = SquarePatrolRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
