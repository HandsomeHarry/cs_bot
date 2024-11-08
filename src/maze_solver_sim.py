#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeSolverSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.twist = Twist()

        self.state = 'SEARCHING_WALL'  # Can be 'SEARCHING_WALL', 'APPROACHING_WALL', or 'FOLLOWING_WALL'
        
        self.wall_distance = 0.2  # Desired distance from the wall (in meters)
        self.front_distance_threshold = 0.35  # Distance to consider an obstacle in front (for following)
        self.wall_detection_threshold = 0.7  # Threshold to detect a wall when searching
        self.side_distance_threshold = 0.15   # Distance to consider an obstacle on the side

        self.angular_speed_left = 0.5  # Angular speed for left turns
        self.angular_speed_right = 1.0  # Increased angular speed for right turns (turn more aggressively)

        self.left_side_distance = None  # Distance to the left wall
        self.front_distance = None      # Distance to the obstacle in front
        self.front_left_distance = None

        self.gen2 = None

        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.3
        self.integral = 0
        self.prev_error = 0
    
    def scan_cb(self, msg):
        """Callback function for processing LaserScan data."""
        self.front_distance = min(msg.ranges[0:20])  # front ranges
        self.left_side_distance = min(msg.ranges[70:90])  # Left side ranges (40 degrees)
        self.front_left_distance = min(msg.ranges[35:45])
        self.gen2 = self.front_left_distance / 1.414

    def pid_control(self, error):
        """Compute PID control value."""
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def follow_wall(self):
        """Controls the robot to move straight, find a wall, and follow it."""
        rate = rospy.Rate(10)  # Loop at 10 Hz
        while not rospy.is_shutdown():
            if self.front_distance is None or self.left_side_distance is None:
                continue  # Wait until the first scan data is received
            
            if self.state == 'SEARCHING_WALL':
                self.twist.linear.x = 0.2  # Move straight
                self.twist.angular.z = 0  # No turning
                print(self.front_distance, "searching wall")
                # Check if a wall is detected in front
                if self.front_distance < self.wall_detection_threshold:
                    self.state = 'APPROACHING_WALL'

            elif self.state == 'APPROACHING_WALL':
                # Continue moving toward the wall until it's within close range for wall following
                self.twist.linear.x = 0.1  # Slow down as it approaches the wall
                print(f"{self.front_distance:.3f}", f"{self.left_side_distance:.3f}", f"{self.front_left_distance:.3f}", "approaching wall")
                # If the robot is close enough to the wall, start following it
                if self.front_distance < self.front_distance_threshold:
                    self.state = 'FOLLOWING_WALL'

            elif self.state == 'FOLLOWING_WALL':
                # Check if there is an obstacle in front
                if self.front_distance < self.front_distance_threshold:
                    # Turn right if an obstacle is directly ahead
                    self.twist.linear.x = 0.1  # Slowly moving forward
                    self.twist.angular.z = -self.angular_speed_right  # Turn right aggressively
                    print(f"{self.front_distance:.3f}", "following wall, ob ahead")
                else:
                    error = self.wall_distance - self.left_side_distance
                    pid_output = self.pid_control(error)
                    
                    # Limit the PID output to maintain similar speeds
                    pid_output = max(-0.5, min(0.5, pid_output))
                    
                    if self.front_left_distance > (self.gen2 + 0.1) or self.left_side_distance > self.wall_distance:
                        # If no wall is on the left, turn left to find the wall
                        self.twist.linear.x = 0.12  # Move forward slowly
                        self.twist.angular.z = 0.45 + pid_output  # Turn left with PID adjustment
                        print(f"{self.front_distance:.3f}", "where's wall on left?")
                    elif self.front_left_distance < (self.gen2 - 0.1) or self.left_side_distance < self.side_distance_threshold:
                        # If too close to the left wall, turn right slightly
                        self.twist.linear.x = 0.1  # Move forward
                        self.twist.angular.z = -0.5 + pid_output  # Turn right with PID adjustment
                        print(f"{self.front_distance:.3f}", "oops too close")
                    else:
                        # Move forward while maintaining the distance to the left wall
                        self.twist.linear.x = 0.2  # Move forward
                        self.twist.angular.z = pid_output  # Adjust direction based on PID
                        print(f"{self.front_distance:.3f}", "following wall")
            # Publish the movement command
            self.cmd_vel_pub.publish(self.twist)

            # Sleep to maintain the loop rate
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
