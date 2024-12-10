#!/usr/bin/env python

import rospy
import yaml
import os

def load_yaml_params():
    # Initialize a ROS node
    rospy.init_node('set_params', anonymous=True)

    # Load the YAML file
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'map_config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    # Set parameters
    rospy.set_param('/spawn_points', config['spawn_points'])

if __name__ == '__main__':
    try:
        load_yaml_params()
    except rospy.ROSInterruptException:
        pass 