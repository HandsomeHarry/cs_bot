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
        self.patrol_points = []  # patrol points
        
        # publishers
        self.marker_pub = rospy.Publisher('/game/map_markers', MarkerArray, queue_size=1)
        self.map_pub = rospy.Publisher('/game/map', OccupancyGrid, queue_size=1)
        
        # load map config
        self.load_map_config()
        
        # timer to publish map markers
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def load_map_config(self):
        """Load map information from YAML file"""
        try:
            # Get the YAML file path
            yaml_file_path = os.path.join(os.path.dirname(__file__), '../config/map_config.yaml')
            
            # Read YAML file
            with open(yaml_file_path, 'r') as file:
                map_config = yaml.safe_load(file)
            
            # Extract spawn points
            self.spawn_points = map_config['spawn_points']
            
            # Extract bomb site corners
            self.bomb_sites = [
                Point(**map_config['bomb_site']['corner1']),
                Point(**map_config['bomb_site']['corner2'])
            ]
            
            # Extract patrol points
            self.patrol_points = [
                Point(x=point['x'], y=point['y'], z=point['z'])
                for point in map_config.get('patrol_points', [])
            ]
            
        except Exception as e:
            rospy.logerr(f"Failed to load map config from YAML: {str(e)}")

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

    def get_spawn_points(self):
        """Return the spawn points."""
        return self.spawn_points

    def get_patrol_points(self):
        """Return the patrol points."""
        return self.patrol_points

if __name__ == '__main__':
    try:
        rospy.init_node("map_manager")
        rospy.loginfo("Map Manager Node Started")
        map_manager = MapManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
