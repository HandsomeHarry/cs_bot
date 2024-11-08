#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, Float32, String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class ServerNode:
    def __init__(self):
        rospy.init_node('server_node', anonymous=True)

        # Game State Variables
        self.round_timer = 60  # Countdown timer (seconds)
        self.bomb_planted = False
        self.bomb_timer = 30  # Countdown once bomb is planted
        self.players_alive = [True, True, True, True]  # Alive status for each robot
        self.robot_health = [100, 100, 100, 100]  # Starting health

        # Visualization
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

        # Setup Publishers
        self.round_timer_pub = rospy.Publisher('/round_timer', Int32, queue_size=10)
        self.bomb_planted_pub = rospy.Publisher('/bomb_planted', Bool, queue_size=10)
        self.players_alive_pub = rospy.Publisher('/players_alive', Bool, queue_size=10)

        # Initialize Subscribers for each bot
        self.init_subscribers('bot1', 0)
        self.init_subscribers('bot2', 1)
        self.init_subscribers('bot3', 2)
        self.init_subscribers('bot4', 3)

        # Start timer thread
        self.timer_thread = threading.Thread(target=self.round_timer_thread)
        self.timer_thread.start()

    def init_subscribers(self, robot_name, idx):
        # Robot location
        rospy.Subscriber(f"/{robot_name}/location", Odometry, self.location_callback, idx)
        
        # Robot status and actions
        rospy.Subscriber(f"/{robot_name}/is_dead", Bool, self.dead_callback, idx)
        rospy.Subscriber(f"/{robot_name}/is_shooting", Bool, self.shooting_callback, idx)
        rospy.Subscriber(f"/{robot_name}/gun", String, self.gun_callback, idx)
        rospy.Subscriber(f"/{robot_name}/health", Float32, self.health_callback, idx)

    def round_timer_thread(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if not any(self.players_alive):
                rospy.loginfo("Round ends: All players are dead.")
                break

            # Check if bomb is planted and adjust timer
            if self.bomb_planted:
                if self.bomb_timer > 0:
                    self.bomb_timer -= 1
                else:
                    rospy.loginfo("Bomb exploded! Round ends.")
                    break
            elif self.round_timer > 0:
                self.round_timer -= 1
            else:
                rospy.loginfo("Round timer expired! Round ends.")
                break

            # Publish timers
            self.round_timer_pub.publish(self.round_timer)
            self.bomb_planted_pub.publish(self.bomb_planted)
            self.players_alive_pub.publish(self.players_alive)

            rate.sleep()

    def location_callback(self, msg, idx):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        # Store location if needed for visualization
        self.robot_positions[idx] = (x, y)

    def dead_callback(self, msg, idx):
        if msg.data:
            self.players_alive[idx] = False
            rospy.loginfo(f"Player {idx + 1} is dead.")

    def shooting_callback(self, msg, idx):
        if msg.data:
            # Calculate if the shots are hitting based on the robot's location and pose
            rospy.loginfo(f"Player {idx + 1} is shooting.")

            # Here, you'd check if the shots hit based on pose, distance, and enemy positions

    def gun_callback(self, msg, idx):
        # Track gun type if needed for damage calculations
        self.robot_guns[idx] = msg.data

    def health_callback(self, msg, idx):
        health = self.robot_health[idx]
        if health <= 0:
            self.players_alive[idx] = False
            rospy.loginfo(f"Player {idx + 1} is dead.")
        else:
            self.robot_health[idx] = msg.data

    def update_map(self, robot_positions, bombsite_location, enemy_positions=[]):
        """Visualizes the positions of robots, the bombsite, and detected enemies on a 2D map with unique colors for each robot."""
        
        # Define colors for each robot
        robot_colors = ['red', 'yellow', 'green', 'blue']

        # Clear and set up the plot
        self.ax.clear()
        self.ax.set_xlim(-10, 10)  # Adjust limits based on map size
        self.ax.set_ylim(-10, 10)
        self.ax.set_title("Game Map Visualization")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")

        # Plot bombsite
        bombsite_circle = Circle(bombsite_location, 0.5, color='purple', alpha=0.5, label="Bombsite")
        self.ax.add_patch(bombsite_circle)

        # Plot robot positions with unique colors
        for i, (x, y) in enumerate(robot_positions):
            robot_circle = Circle((x, y), 0.3, color=robot_colors[i % len(robot_colors)], alpha=0.6)
            self.ax.add_patch(robot_circle)
            self.ax.text(x, y + 0.5, f"Robot {i + 1}", ha='center', color=robot_colors[i % len(robot_colors)])

        # Plot enemy positions (if any detected)
        for ex, ey in enemy_positions:
            enemy_circle = Circle((ex, ey), 0.3, color='black', alpha=0.6, label="Enemy")
            self.ax.add_patch(enemy_circle)

        plt.legend()
        plt.draw()
        plt.pause(0.1)  # Pause to allow the plot to update

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_map()
            rate.sleep()

if __name__ == "__main__":
    try:
        server = ServerNode()
        server.run()
    except rospy.ROSInterruptException:
        pass
