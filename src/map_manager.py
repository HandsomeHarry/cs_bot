#!/usr/bin/env python
# coding=utf-8

import rospy
import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import os
import subprocess

class MapManager:
    def __init__(self):
        # map basic information
        self.bomb_sites = []  
        self.spawn_points = {'T': [], 'CT': []}  
        self.patrol_points = []  
        
        # publishers
        self.marker_pub = rospy.Publisher('/game/map_markers', MarkerArray, queue_size=1)
        
        # Start map_server node
        self.start_map_server()
        
        # load game config
        self.load_game_config()
        
        # timer to publish map markers
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def start_map_server(self):
        """Start the map_server node with the correct map"""
        try:
            map_yaml_path = os.path.join(os.path.dirname(__file__), '../maps/world3_map.yaml')
            rospy.loginfo(f"Starting map_server with map: {map_yaml_path}")
            subprocess.Popen(['rosrun', 'map_server', 'map_server', map_yaml_path])
        except Exception as e:
            rospy.logerr(f"Failed to start map_server: {str(e)}")

    def load_game_config(self):
        """Load game information from YAML file"""
        try:
            game_config_path = os.path.join(os.path.dirname(__file__), '../config/map_config.yaml')
            with open(game_config_path, 'r') as file:
                game_config = yaml.safe_load(file)
            
            # Extract spawn points
            self.spawn_points = {
                team: [Point(x=p['x'], y=p['y'], z=p['z']) for p in points]
                for team, points in game_config['spawn_points'].items()
            }
            
            # Extract bomb site corners
            self.bomb_sites = [
                Point(**game_config['bomb_site']['corner1']),
                Point(**game_config['bomb_site']['corner2'])
            ]
            
            # Extract patrol points
            self.patrol_points = [
                Point(x=point['x'], y=point['y'], z=point['z'])
                for point in game_config.get('patrol_points', [])
            ]
            
        except Exception as e:
            rospy.logerr(f"Failed to load game config: {str(e)}")

    def create_marker(self, position, marker_type, id, color, scale):
        """Create visualization marker"""
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
        """Publish all map markers"""
        marker_array = MarkerArray()
        
        # Add bomb site markers
        for i, site in enumerate(self.bomb_sites):
            marker = self.create_marker(
                site,
                Marker.CYLINDER,
                i,
                (1.0, 0.0, 0.0, 0.7),  # red
                (1.0, 1.0, 0.2)
            )
            marker_array.markers.append(marker)
        
        # Add spawn point markers
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
        
        # Publish markers
        self.marker_pub.publish(marker_array)

    def get_spawn_points(self):
        """Return the spawn points."""
        return self.spawn_points

    def get_patrol_points(self):
        """Return the patrol points."""
        return self.patrol_points

    def get_patrol_points(self):
        return self.patrol_points

if __name__ == '__main__':
    try:
        rospy.init_node("map_manager")
        rospy.loginfo("Map Manager Node Started")
        map_manager = MapManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
