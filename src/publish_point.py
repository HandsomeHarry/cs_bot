#!/usr/bin/env python

# run the following commands in the terminal to start the map server and rviz
#rosrun map_server map_server maps/world3_map.yaml
#rosrun rviz rviz -d path/to/your_rviz_config.rviz
#!/usr/bin/env python3

import rospy
import rospkg
import subprocess
import os
import signal
from datetime import datetime
from geometry_msgs.msg import PointStamped

class PointRecorder:
    def __init__(self):
        rospy.init_node('point_recorder', anonymous=True)
        self.points = []
        self.labels = ['T_start', 'CT_start', 'site_topleft', 'site_bottomright']
        self.current_label_index = 0
        self.filename = f"site_points.txt"

    def clicked_point_callback(self, msg):
        if self.current_label_index < len(self.labels):
            point = {
                'label': self.labels[self.current_label_index],
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            }
            self.points.append(point)
            print(f"Recorded {point['label']}: (x={point['x']:.2f}, y={point['y']:.2f})")
            self.current_label_index += 1

            if self.current_label_index == len(self.labels):
                print("All points recorded. Saving to file...")
                self.save_points()
                rospy.signal_shutdown("Recording complete.")

    def save_points(self):
        with open(self.filename, 'w') as file:
            for point in self.points:
                file.write(f"{point['label']} {point['x']} {point['y']} {point['z']}\n")
        print(f"Points saved to {self.filename}")

    def listen_for_points(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        print("\nPlease use the publish point tool on the top right\
                \n and click the following points in RViz in order:")
        for label in self.labels:
            print(f" - {label}")
        rospy.spin()


if __name__ == '__main__':
    try:

        # Get the path to the cs_bot package
        rospack = rospkg.RosPack()
        cs_bot_path = rospack.get_path('cs_bot')  # Resolves the absolute path to the cs_bot package

        # Construct the path to the maps/world3_map.pgm file
        map_file_path = f"{cs_bot_path}/maps/world3_map.yaml"
        # Launch map server
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
        if map_server_proc:
            os.kill(map_server_proc.pid, signal.SIGINT)
        if rviz_proc:
            os.kill(rviz_proc.pid, signal.SIGINT)