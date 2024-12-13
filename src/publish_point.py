#!/usr/bin/env python

# run the following commands in the terminal to start the map server and rviz
#rosrun map_server map_server maps/world4.yaml
#rosrun rviz rviz -d rviz_config.rviz
#!/usr/bin/env python3

import rospy
import rospkg
import subprocess
import os
import signal
import yaml
from geometry_msgs.msg import PointStamped

class PointRecorder:
    def __init__(self):
        rospy.init_node('point_recorder', anonymous=True)
        self.points = {}
        self.labels = [
            'T_spawn1', 'T_spawn2',           # Two T spawn points
            'CT_spawn1', 'CT_spawn2',         # Two CT spawn points
            'site1', 'site2',   # possible bomb locations
            'patrol_point1', 'patrol_point2', 'patrol_point3', 'patrol_point4'  # Patrol points
        ]
        self.current_label_index = 0
        
        # Get path to config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.filename = os.path.join(script_dir, '..', 'config', 'map_config.yaml')

    def clicked_point_callback(self, msg):
        if self.current_label_index < len(self.labels):
            point = {
                'x': round(msg.point.x, 2),
                'y': round(msg.point.y, 2),
                'z': round(msg.point.z, 2)
            }
            
            # Store point with its label
            self.points[self.labels[self.current_label_index]] = point
            
            print(f"Recorded {self.labels[self.current_label_index]}: (x={point['x']:.2f}, y={point['y']:.2f})")
            self.current_label_index += 1

            if self.current_label_index == len(self.labels):
                print("All points recorded. Saving to file...")
                self.save_points()
                rospy.signal_shutdown("Recording complete.")

    def save_points(self):
        # Create config directory if it doesn't exist
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        
        # Organize data for YAML
        map_config = {
            'spawn_points': {
                'T': [
                    self.points['T_spawn1'],
                    self.points['T_spawn2']
                ],
                'CT': [
                    self.points['CT_spawn1'],
                    self.points['CT_spawn2']
                ]
            },
            'bomb_site': {
                'site1': self.points['site1'],
                'site2': self.points['site2']
            },
            'patrol_points': [
                self.points['patrol_point1'],
                self.points['patrol_point2'],
                self.points['patrol_point3'],
                self.points['patrol_point4']
            ]
        }
        
        # Write to YAML file
        with open(self.filename, 'w') as file:
            yaml.dump(map_config, file, default_flow_style=False)
        
        print(f"Points saved to {self.filename}")

    def listen_for_points(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        print("\nPlease use the publish point tool in RViz to click the following points in order:")
        for label in self.labels:
            print(f" - {label}")
        rospy.spin()

if __name__ == '__main__':
    try:
        # Get the path to the cs_bot package
        rospack = rospkg.RosPack()
        cs_bot_path = rospack.get_path('cs_bot')

        # Launch map server
        map_file_path = f"{cs_bot_path}/maps/world4.yaml"
        map_server_cmd = ['rosrun', 'map_server', 'map_server', map_file_path]
        map_server_proc = subprocess.Popen(map_server_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("map server launched")

        # Launch RViz
        rviz_cmd = ['rosrun', 'rviz', 'rviz', '-d', 'rviz/map_pointer.rviz']
        rviz_proc = subprocess.Popen(rviz_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("rviz launched")

        # Wait for RViz to load
        print("Waiting 1 second for RViz to load...")
        rospy.sleep(1)

        # Start point recorder
        recorder = PointRecorder()
        recorder.listen_for_points()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Terminate subprocesses
        print("Shutting down in 3 secs...")
        rospy.sleep(3)
        if 'map_server_proc' in locals():
            os.kill(map_server_proc.pid, signal.SIGINT)
        if 'rviz_proc' in locals():
            os.kill(rviz_proc.pid, signal.SIGINT)