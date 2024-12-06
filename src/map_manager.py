#!/usr/bin/env python
# coding=utf-8

import rospy
import yaml
import tf
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
import pandas as pd
import os

class MapManager:
    def __init__(self):
        rospy.init_node('map_manager', anonymous=True)
        
        # map basic information
        self.bomb_sites = []  # location of the single bomb site (two points that forms a rectangle)
        self.spawn_points = {'T': [], 'CT': []}  # spawn points
        
        # publishers
        self.marker_pub = rospy.Publisher('/game/map_markers', MarkerArray, queue_size=1)
        self.map_pub = rospy.Publisher('/game/map', OccupancyGrid, queue_size=1)
        
        # load map config
        self.load_map_config()
        
        # timer to publish map markers
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def load_map_config(self):
        """load map information from CSV file"""
        try:
            # Get the CSV file path
            csv_file = os.path.join(os.path.dirname(__file__), '../site_points.csv')
            
            # Read CSV using pandas
            df = pd.read_csv(csv_file)
            
            # Convert DataFrame rows to Point objects
            for _, row in df.iterrows():
                point = Point(float(row['x']), float(row['y']), float(row['z']))
                
                if row['label'] == 'T_spawn':
                    self.spawn_points['T'] = [point]
                elif row['label'] == 'CT_spawn':
                    self.spawn_points['CT'] = [point]
                elif row['label'].startswith('site_corner'):
                    self.bomb_sites.append(point)
            
            # Initialize empty lists for unused config items
            self.safe_zones = []
            self.walls = []
            self.fiducial_markers = []
            
        except Exception as e:
            rospy.logerr(f"Failed to load map config from CSV: {str(e)}")

    def create_marker(self, position, marker_type, id, color, scale):
        """create visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "game_map"
        marker.id = id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color = ColorRGBA(*color)
        return marker

    def publish_markers(self, event=None):
        """publish all map markers"""
        marker_array = MarkerArray()
        
        # add bomb site markers
        for i, site in enumerate(self.bomb_sites):
            marker = self.create_marker(
                site,
                Marker.CYLINDER,
                i,
                (1.0, 0.0, 0.0, 0.7),  # red
                (1.0, 1.0, 0.2)
            )
            marker_array.markers.append(marker)
        
        # add spawn point markers
        id_counter = len(self.bomb_sites)
        for team, points in self.spawn_points.items():
            color = (1.0, 0.0, 0.0, 0.7) if team == 'T' else (0.0, 0.0, 1.0, 0.7)
            for point in points:
                marker = self.create_marker(
                    point,
                    Marker.CUBE,
                    id_counter,
                    color,
                    (0.5, 0.5, 0.1)
                )
                marker_array.markers.append(marker)
                id_counter += 1
        
        # publish markers
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        map_manager = MapManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
