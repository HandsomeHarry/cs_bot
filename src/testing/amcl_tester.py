#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def set_initial_pose():
    """Publishes the initial pose estimate for robot1."""
    rospy.loginfo("Setting initial pose for robot1...")
    initial_pose_pub = rospy.Publisher('/robot1/initialpose', PoseWithCovarianceStamped, queue_size=10)

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "robot1/map"  # Frame for the pose
    initial_pose.header.stamp = rospy.Time.now()

    # Set the robot's initial position
    initial_pose.pose.pose.position.x = -1.72
    initial_pose.pose.pose.position.y = 1.63
    initial_pose.pose.pose.position.z = 0.01

    # Set the robot's initial orientation (facing forward)
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    # Set covariance (uncertainty) for the pose
    initial_pose.pose.covariance[0] = 0.25  # x variance
    initial_pose.pose.covariance[7] = 0.25  # y variance
    initial_pose.pose.covariance[35] = 0.0685375  # yaw variance

    # Publish the initial pose
    for _ in range(10):  # Publish multiple times to ensure AMCL receives it
        initial_pose_pub.publish(initial_pose)
        rospy.sleep(0.1)

    rospy.loginfo("Initial pose published.")

def send_goal():
    """Sends a navigation goal to robot1's move_base."""
    rospy.loginfo("Sending navigation goal to robot1...")
    goal_pub = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped, queue_size=10)

    goal = PoseStamped()
    goal.header.frame_id = "robot1/map"  # Frame for the goal
    goal.header.stamp = rospy.Time.now()

    # Set the target position
    goal.pose.position.x = 0.0
    goal.pose.position.y = -1.43
    goal.pose.position.z = 0.0

    # Set the target orientation (facing forward)
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    # Publish the goal
    rospy.sleep(1)  # Allow time for the publisher to set up
    goal_pub.publish(goal)

    rospy.loginfo("Goal published")

if __name__ == '__main__':
    try:
        rospy.init_node('robot1_pose_and_goal', anonymous=True)

        # Set initial pose
        set_initial_pose()

        # Send move goal
        send_goal()

        rospy.spin()  # Keep the node alive if needed
    except rospy.ROSInterruptException:
        pass
