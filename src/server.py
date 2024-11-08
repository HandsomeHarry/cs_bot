#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, Float32, String, Int32MultiArray
from geometry_msgs.msg import Pose
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
        self.robot_positions = [(0, 0)] * 4  # Initialize positions for 4 robots
        self.robot_weapon = [None] * 4  # Initialize weapon types for 4 robots
        self.bombsite_location = (5, 5)  # Example bombsite location

        # Visualization setup
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

        # Setup Publishers
        self.round_timer_pub = rospy.Publisher('/round_timer', Int32, queue_size=10)
        self.bomb_planted_pub = rospy.Publisher('/bomb_planted', Bool, queue_size=10)
        self.players_alive_pub = rospy.Publisher('/players_alive', Int32MultiArray, queue_size=10)

        # Initialize Subscribers for each robot
        self.init_subscribers('bot1', 0)
        self.init_subscribers('bot2', 1)
        self.init_subscribers('bot3', 2)
        self.init_subscribers('bot4', 3)

        # Start timer thread
        self.timer_thread = threading.Thread(target=self.round_timer_thread)
        self.timer_thread.start()

    def init_subscribers(self, robot_name, idx):
        rospy.Subscriber(f"/{robot_name}/location", Odometry, self.location_callback, idx)
        rospy.Subscriber(f"/{robot_name}/is_dead", Bool, self.dead_callback, idx)
        rospy.Subscriber(f"/{robot_name}/is_shooting", Bool, self.shooting_callback, idx)
        rospy.Subscriber(f"/{robot_name}/weapon", String, self.weapon_callback, idx)
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

            # Publish timers and alive status
            self.round_timer_pub.publish(self.round_timer)
            self.bomb_planted_pub.publish(self.bomb_planted)
            alive_array = Int32MultiArray(data=[int(alive) for alive in self.players_alive])
            self.players_alive_pub.publish(alive_array)

            rate.sleep()

    def location_callback(self, msg, idx):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.robot_positions[idx] = (x, y)

    def dead_callback(self, msg, idx):
        if msg.data:
            self.players_alive[idx] = False
            rospy.loginfo(f"Player {idx + 1} is dead.")

    def shooting_callback(self, msg, idx):
        if msg.data:
            rospy.loginfo(f"Player {idx + 1} is shooting.")
            # Check if shots are hitting based on location, pose, etc. (add logic if needed)

    def weapon_callback(self, msg, idx):
        self.robot_weapon[idx] = msg.data

    def health_callback(self, msg, idx):
        self.robot_health[idx] = msg.data
        if self.robot_health[idx] <= 0:
            self.players_alive[idx] = False
            rospy.loginfo(f"Player {idx + 1} is dead.")

    def update_map(self, robot_positions, bombsite_location):
        """Visualizes the positions of all robots and the bombsite on a 2D map without differentiating teams."""
        
        # Define unique colors for each robot
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

        # Plot each robot's position with unique colors
        for i, (x, y) in enumerate(robot_positions):
            robot_circle = Circle((x, y), 0.3, color=robot_colors[i % len(robot_colors)], alpha=0.6)
            self.ax.add_patch(robot_circle)
            self.ax.text(x, y + 0.5, f"Robot {i + 1}", ha='center', color=robot_colors[i % len(robot_colors)])

        # Draw and update the map visualization
        plt.legend()
        plt.draw()
        plt.pause(0.1)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_map(self.robot_positions, self.bombsite_location)
            rate.sleep()

if __name__ == "__main__":
    try:
        server = ServerNode()
        server.run()
    except rospy.ROSInterruptException:
        pass
