#!/usr/bin/env python3

import rospy
import yaml
import os
import subprocess
from pathlib import Path

class LaunchHelper:
    def __init__(self):
        self.package_path = str(Path(__file__).parent.parent)
        self.config_path = os.path.join(self.package_path, 'config', 'map_config.yaml')
        
    def load_map_config(self):
        """Load map configuration from YAML"""
        with open(self.config_path, 'r') as file:
            return yaml.safe_load(file)

    def launch_game(self):
        """Launch the game with parameters from config"""
        config = self.load_map_config()
        
        # Prepare spawn point arguments
        launch_args = [
            'roslaunch',
            'cs_bot',
            'sim_2v2.launch',
            f'ct_spawn_1_x:={config["spawn_points"]["CT"][0]["x"]}',
            f'ct_spawn_1_y:={config["spawn_points"]["CT"][0]["y"]}',
            f'ct_spawn_2_x:={config["spawn_points"]["CT"][1]["x"]}',
            f'ct_spawn_2_y:={config["spawn_points"]["CT"][1]["y"]}',
            f't_spawn_1_x:={config["spawn_points"]["T"][0]["x"]}',
            f't_spawn_1_y:={config["spawn_points"]["T"][0]["y"]}',
            f't_spawn_2_x:={config["spawn_points"]["T"][1]["x"]}',
            f't_spawn_2_y:={config["spawn_points"]["T"][1]["y"]}'
        ]
        for arg in launch_args:
            print(arg, end=' ')

        # Launch the game with arguments
        subprocess.run(launch_args)

if __name__ == '__main__':
    launcher = LaunchHelper()
    launcher.launch_game()