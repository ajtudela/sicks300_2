#!/usr/bin/env python3

'''
    Launches a Sick S300 laser scanner node and a filter node.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import launch.events
import lifecycle_msgs.msg

def generate_launch_description():

    # Default filenames and where to find them
    sicks300_dir = get_package_share_directory('sicks300_2')

    # Read the YAML parameters file.
    default_sicks300_param_file = os.path.join(sicks300_dir, 'config', 'dummy_config.yaml')

    # Create the launch configuration variables.
    sicks300_param_file = LaunchConfiguration('sicks300_param_file', default=default_sicks300_param_file)

    # Map these variables to arguments: can be set from the command line or a default will be used
    sicks300_param_file_launch_arg = DeclareLaunchArgument(
        'sicks300_param_file',
        default_value=default_sicks300_param_file,
        description='Full path to the Sicks300 parameter file to use'
    )

    # Prepare the sicks300_2 node.
    sicks300_2_node = LifecycleNode(
            package = 'sicks300_2',
            namespace = '',
            executable = 'sicks300_2',
            name = 'laser_front',
            parameters = [sicks300_param_file],
            emulate_tty = True
        )

    # Prepare the filter node.
    filter_node = Node(
            package = 'laser_filters',
            namespace = '',
            executable = 'scan_to_scan_filter_chain',
            name = 'scan_filter',
            parameters = [sicks300_param_file],
            emulate_tty = True
        )

    # When the sick node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_sick_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = sicks300_2_node,
            goal_state = 'inactive',
            entities = [
                EmitEvent(event = ChangeState(
                    lifecycle_node_matcher = launch.events.matches_action(sicks300_2_node),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the sick node reaches the 'active' state, start the filter node.
    register_event_handler_for_sick_reaches_active_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = sicks300_2_node,
            goal_state = 'active',
            entities = [
                filter_node
            ],
        )
    )

    # Make the sicks300_2 node take the 'configure' transition.
    emit_event_to_request_that_sick_does_configure_transition = EmitEvent(
            event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(sicks300_2_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        sicks300_param_file_launch_arg,
        register_event_handler_for_sick_reaches_inactive_state,
        register_event_handler_for_sick_reaches_active_state,
        sicks300_2_node,
        emit_event_to_request_that_sick_does_configure_transition,
    ])