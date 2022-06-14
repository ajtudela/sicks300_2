# Dummy launch

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Read the YAML parameters file
    config = os.path.join(
        get_package_share_directory('sicks300_2'),
        'config',
        'dummy_config.yaml'
        )

    # Start a sicks300_2 node
    sicks300_2_node = Node(
            package = 'sicks300_2',
            namespace = '',
            executable = 'sicks300_2',
            name = 'laser_front',
            parameters = [config],
            emulate_tty = True
        )

    # Start a filter node
    filter_node = Node(
            package = 'laser_filters',
            namespace = '',
            executable = 'scan_to_scan_filter_chain',
            name = 'scan_filter',
            parameters = [config],
            emulate_tty = True
        )

    return LaunchDescription([
        sicks300_2_node, filter_node,
    ])