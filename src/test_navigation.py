#!/usr/bin/env python
# coding=utf-8


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

class NavigationTest:
    def __init__(self):
        rospy.init_node('test_navigation')
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        
        # Create a move_base action client
        self.client = actionlib.SimpleActionClient(
            '/{}/move_base'.format(self.robot_name),
            MoveBaseAction
        )
        rospy.loginfo("waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("move_base server connected")

        # Subscribe to lidar data
        self.laser_sub = rospy.Subscriber(
            '/{}/scan'.format(self.robot_name),
            LaserScan,
            self.laser_callback
        )

        # Subscribe to the local costmap
        self.costmap_sub = rospy.Subscriber(
            '/{}/move_base/local_costmap/costmap'.format(self.robot_name),
            OccupancyGrid,
            self.costmap_callback
        )

        self.min_obstacle_distance = float('inf')
        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Filter useless data
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            self.min_obstacle_distance = min(valid_ranges)
            if self.min_obstacle_distance < 0.5:  # set obstacle detection boundary <----
                if not self.obstacle_detected:
                    rospy.loginfo("Obstacle detected! Distance: {:.2f}m".format(self.min_obstacle_distance))
                    self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def costmap_callback(self, msg):
        # check for obstacles in costmap
        obstacle_cells = sum(1 for cell in msg.data if cell > 50)
        if obstacle_cells > 0:
            rospy.loginfo("detected {} obstacle grids in costmap".format(obstacle_cells))

    def move_to_goal(self, x, y, w=1.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo("Directing to: x={}, y={}".format(x, y))
        self.client.send_goal(goal)
        
        # monitor navigation state
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrived at goal")
                return True
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn("im in deep trouble")
                return False
            elif state == actionlib.GoalStatus.REJECTED:
                rospy.logwarn("goal rejected, wtf are you doing")
                return False
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        navigator = NavigationTest()
        # test points
        test_points = [
            (3.0, 4.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (6.0, 0.0)
        ]
        
        for x, y in test_points:
            if not navigator.move_to_goal(x, y):
                rospy.logwarn("something failed")
                break
            rospy.sleep(1.0)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("interrupted :(")
