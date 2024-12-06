#!/usr/bin/env python

# run the following commands in the terminal to start the map server and rviz
#rosrun map_server map_server maps/world3_map.yaml
#rosrun rviz rviz -d path/to/your_rviz_config.rviz
#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal
from datetime import datetime
from geometry_msgs.msg import PointStamped
import rospkg

class PointRecorder:
    def __init__(self):
        rospy.init_node('point_recorder', anonymous=True)
        self.points = []
        self.labels = ['T_start', 'CT_start', 'site_topleft', 'site_bottomright']
        self.current_label_index = 0
        self.filename = f"point_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

    def clicked_point_callback(self, msg):
        if self.current_label_index < len(self.labels):
            point = {
                'label': self.labels[self.current_label_index],
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            }
            self.points.append(point)
            rospy.loginfo(f"Recorded {point['label']}: (x={point['x']}, y={point['y']}, z={point['z']})")
            self.current_label_index += 1

            if self.current_label_index == len(self.labels):
                rospy.loginfo("All points recorded. Saving to file...")
                self.save_points()
                rospy.signal_shutdown("Recording complete.")

    def save_points(self):
        with open(self.filename, 'w') as file:
            for point in self.points:
                file.write(f"{point['label']} {point['x']} {point['y']} {point['z']}\n")
        rospy.loginfo(f"Points saved to {self.filename}")

    def listen_for_points(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        rospy.loginfo("Please click the following points in RViz in order:")
        for label in self.labels:
            rospy.loginfo(f" - {label}")
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
        rospy.loginfo("Waiting 2 seconds for RViz to load...")
        rospy.sleep(2)

        # Start point recorder
        recorder = PointRecorder()
        recorder.listen_for_points()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    finally:
        # Terminate subprocesses
        if map_server_proc:
            os.kill(map_server_proc.pid, signal.SIGINT)
        if rviz_proc:
            os.kill(rviz_proc.pid, signal.SIGINT)