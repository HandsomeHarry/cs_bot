#!/usr/bin/env python3

import yaml
import os
from pathlib import Path

class LaunchHelper:
    def __init__(self):
        self.package_path = str(Path(__file__).parent.parent)
        self.config_path = os.path.join(self.package_path, 'config', 'map_config.yaml')
        self.launch_file_path = os.path.join(self.package_path, 'launch', 'spawn_robots.launch')
        
        # Map robots to their teams and spawn indices
        self.robot_configs = {
            'roba': {'team': 'CT', 'index': 0},    # CT first spawn
            'robb': {'team': 'T', 'index': 0},     # T first spawn
            'robc': {'team': 'CT', 'index': 1},    # CT second spawn
            'rafael': {'team': 'T', 'index': 1}    # T second spawn
        }
        
    def load_map_config(self):
        """Load map configuration from YAML"""
        with open(self.config_path, 'r') as file:
            return yaml.safe_load(file)

    def update_launch_file(self):
        """Update the launch file with spawn points from config"""
    def update_launch_file(self):
        """Update the launch file with spawn points from config"""
        config = self.load_map_config()
        
        # Read the launch file
        with open(self.launch_file_path, 'r') as file:
            lines = file.readlines()

        # Process the file line by line
        for i in range(len(lines)):
            if '<!-- BEGIN' in lines[i]:
                # Extract robot name from the comment
                robot_name = lines[i].split('BEGIN')[1].strip().strip('-->')
                robot_name = robot_name.strip()
                
                if robot_name in self.robot_configs:
                    robot_config = self.robot_configs[robot_name]
                    team = robot_config['team']
                    spawn_index = robot_config['index']

                    # Get spawn coordinates
                    x = config['spawn_points'][team][spawn_index]['x']
                    y = config['spawn_points'][team][spawn_index]['y']

                    # Update the next few lines
                    for j in range(i + 1, i + 5):
                        if 'name="init_pose"' in lines[j]:
                            lines[j] = f'    <arg name="init_pose" value="-x {x} -y {y} -z 0 -Y 0" />\n'
                        elif 'name="init_pose_x"' in lines[j]:
                            lines[j] = f'    <arg name="init_pose_x" value="{x}"/>\n'
                        elif 'name="init_pose_y"' in lines[j]:
                            lines[j] = f'    <arg name="init_pose_y" value="{y}"/>\n'

        # Write the modified content back to the file
        with open(self.launch_file_path, 'w') as file:
            file.writelines(lines)

if __name__ == '__main__':
    launcher = LaunchHelper()
    launcher.update_launch_file()
    launcher.update_launch_file()