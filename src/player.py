import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import CompressedImage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from gun import Gun
from transitions import Machine

class Player:
    # Define all possible states
    states = ['idle', 'moving_and_searching', 'holding_position', 'shooting', 'returning_to_spawn']

    def __init__(self, namespace, is_CT, gun=None):
        self.namespace = namespace
        self.CT = is_CT
        self.gun = gun if gun is not None else Gun()
        
        # Initialize ROS components
        self.move_base_client = actionlib.SimpleActionClient(f'{namespace}/move_base', MoveBaseAction)
        self.image_sub = rospy.Subscriber(f'{namespace}/raspicam_node/image/compressed', CompressedImage, self.image_cb)
        self.cmd_vel_pub = rospy.Publisher(f'{namespace}/cmd_vel', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher(f'{namespace}/target_pose', PoseStamped, queue_size=10)

        # Store spawn point and current goal
        self.spawn_point = {'x': 0, 'y': 0}  # Set your spawn coordinates
        self.current_goal = None
        
        # Initialize the state machine
        self.machine = Machine(
            model=self,
            states=Player.states,
            initial='idle',
            transitions=[
                # Moving and searching transitions
                {'trigger': 'start_moving', 'source': ['idle', 'shooting'], 'dest': 'moving_and_searching'},
                
                # Combat transitions
                {'trigger': 'target_acquired', 'source': 'moving_and_searching', 'dest': 'shooting'},
                {'trigger': 'target_lost', 'source': 'shooting', 'dest': 'moving_and_searching'},
                
                # Hold position transitions
                {'trigger': 'hold_position', 'source': ['moving_and_searching', 'shooting'], 'dest': 'holding_position'},
                {'trigger': 'resume_moving', 'source': 'holding_position', 'dest': 'moving_and_searching'},
                
                # Return to spawn transitions
                {'trigger': 'return_command', 'source': '*', 'dest': 'returning_to_spawn'},
                {'trigger': 'reached_spawn', 'source': 'returning_to_spawn', 'dest': 'idle'},
                
                # Stop transition
                {'trigger': 'stop', 'source': '*', 'dest': 'idle'}
            ]
        )

        self.machine.add_transition(
            trigger='target_acquired',
            source=['moving_and_searching', 'holding_position'],
            dest='shooting'
        )

    def move_to_point(self, x, y):
        try:
            server_reached = self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0))
            if not server_reached:
                rospy.logerr(f"{self.namespace}: Move base action server not available!")
                return False
            # ... rest of the function
        except rospy.ROSException as e:
            rospy.logerr(f"{self.namespace}: Move base action server error: {e}")
            return False

    def return_to_spawn(self):
        """Command robot to return to spawn point"""
        self.return_command()
        self.move_to_point(self.spawn_point['x'], self.spawn_point['y'])

    def movement_done_callback(self, status, result):
        """Callback for when move_base completes a goal"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"{self.namespace}: Reached goal position")
            if self.state == 'returning_to_spawn':
                self.reached_spawn()
        else:
            rospy.logwarn(f"{self.namespace}: Failed to reach goal, status: {status}")

    def movement_feedback_callback(self, feedback):
        """Handle move_base feedback"""
        if self.state == 'moving_and_searching':
            # Check if we're getting closer to goal
            rospy.logdebug(f"{self.namespace}: Moving to goal...")

    def image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height, width, _ = image.shape
        center_x = width // 2
        tolerance = 20

        center_strip = image[int(height * 0.4):int(height * 0.6), :]
        hsv_image = cv2.cvtColor(center_strip, cv2.COLOR_BGR2HSV)
        
        if self.CT:
            mask1 = cv2.inRange(hsv_image, self.get_color_bounds()[0], self.get_color_bounds()[1])
            mask2 = cv2.inRange(hsv_image, self.get_color_bounds()[2], self.get_color_bounds()[3])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_image, self.get_color_bounds()[0], self.get_color_bounds()[1])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                target_x = int(M["m10"] / M["m00"])
                distance_from_center = target_x - center_x

                if abs(distance_from_center) <= tolerance:
                    if self.state == 'moving_and_searching':
                        self.target_acquired()
                    self.shoot()
                else:
                    if self.state == 'shooting':
                        self.target_lost()
                    self.turn(distance_from_center)
            else:
                if self.state == 'shooting':
                    self.target_lost()
        else:
            if self.state == 'shooting':
                self.target_lost()

    def on_enter_idle(self):
        """Called when entering idle state"""
        rospy.loginfo(f"{self.namespace}: Entering idle state")
        self.stop_movement()

    def on_enter_moving_and_searching(self):
        """Called when entering moving_and_searching state"""
        rospy.loginfo(f"{self.namespace}: Moving and searching for targets")

    def on_enter_shooting(self):
        """Called when entering shooting state"""
        rospy.loginfo(f"{self.namespace}: Target acquired - shooting")
        self.move_base_client.cancel_all_goals()  # Stop movement while shooting

    def on_enter_returning_to_spawn(self):
        """Called when entering returning_to_spawn state"""
        rospy.loginfo(f"{self.namespace}: Returning to spawn point")

    def stop_movement(self):
        """Helper function to stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.move_base_client.cancel_all_goals()

    def turn(self, distance):
        twist = Twist()
        twist.angular.z = -0.005 * distance
        self.cmd_vel_pub.publish(twist)

    def shoot(self):
        rospy.loginfo(f"{self.namespace}: Shooting!")
        self.gun.shoot()

    def get_color_bounds(self):
        if self.CT:  # Terrorists red
            return [(0, 100, 100), (10, 255, 255),  # Lower red range
                    (170, 100, 100), (180, 255, 255)]  # Upper red range
        else:  # Blue
            return [(110, 100, 100), (130, 255, 255)]  # Blue range

