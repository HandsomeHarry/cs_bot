#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# DO NOT USE STILL IN DEVELOPMENT - zared
class BasicMovement:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.twist = Twist()

        self.state = 'FINDING_WALL'
        self.ideal_distance = 0.21
        self.front_distance_limit = 0.35
        self.wall_detect_threshold = 0.7
        self.side_distance_limit = 0.15

        self.angular_speed = 0.5
        self.adjustment_factor = None
        self.speed = 0.1
        self.wall_scan_speed = 0.135 # speed when turning corner

        self.left_distance = None
        self.front_distance = None
        self.front_left_distance = None

    def scan_cb(self, msg):
        self.front_distance = min(msg.ranges[0:20])
        self.left_distance = min(msg.ranges[75:90]) # go a little before 90 so robot can see wall when turning
        self.front_left_distance = min(msg.ranges[35:45]) # for when wall ends
        self.adjustment_factor = self.front_left_distance / 1.414

    def follow_wall(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.front_distance is None or self.left_distance is None:
                continue
            
            if self.state == 'FINDING_WALL':
                self.twist.linear.x = self.speed * 2
                self.twist.angular.z = 0
                print(self.front_distance, "looking for wall")
                if self.front_distance < self.front_distance_limit:
                    self.state = 'WALL_FOLLOWING'

            elif self.state == 'WALL_FOLLOWING':
                if self.front_distance < self.front_distance_limit:
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = -self.angular_speed * 2
                    print(f"{self.front_distance:.3f} - obstacle ahead while following wall")
                else:
                    error = self.ideal_distance - self.left_distance
                    error = max(-self.angular_speed, min(self.angular_speed, error)) # for killing outliers

                    if self.front_left_distance > (self.adjustment_factor + 0.1) or self.left_distance > self.ideal_distance:
                        self.twist.linear.x = self.wall_scan_speed
                        self.twist.angular.z = max(self.angular_speed + error, self.angular_speed)
                        print(f"{(self.left_distance):.3f} - searching left side")
                    elif self.front_left_distance < (self.adjustment_factor - 0.1) or self.left_distance < self.side_distance_limit:
                        self.twist.linear.x = self.speed
                        self.twist.angular.z = -self.angular_speed + error # pid output should be negative here so add
                        print(f"{self.left_distance:.3f} - too close to left wall")
                    else:
                        self.twist.linear.x = self.speed * 2 # go faster if parallel
                        self.twist.angular.z = error
                        print(f"{self.left_distance:.3f} - maintaining wall distance")
            
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
