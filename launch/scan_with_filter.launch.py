#!/usr/bin/env python3
# Copyright (c) 2022 Alberto J. Tudela Roldán
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Launches a Sick S300 laser scanner node and a filter node."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

import launch.events
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():
    # Default filenames and where to find them
    sicks300_dir = get_package_share_directory('sicks300_2')

    # Read the YAML parameters file.
    default_sicks300_param_file = os.path.join(sicks300_dir, 'params', 'default.yaml')

    # Create the launch configuration variables.
    sicks300_param_file = LaunchConfiguration(
        'sicks300_param_file', default=default_sicks300_param_file)

    # Map these variables to arguments: can be set from the command line or a default will be used
    sicks300_param_file_launch_arg = DeclareLaunchArgument(
        'sicks300_param_file',
        default_value=default_sicks300_param_file,
        description='Full path to the Sicks300 parameter file to use'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the sicks300_2 node.
    sicks300_2_node = LifecycleNode(
        package='sicks300_2',
        namespace='',
        executable='sicks300_2',
        name='laser_front',
        parameters=[sicks300_param_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['laser_front:=', LaunchConfiguration('log-level')]]
    )

    # Prepare the filter node.
    filter_node = Node(
        package='sicks300_2',
        namespace='',
        executable='scan_filter',
        name='scan_filter',
        parameters=[sicks300_param_file],
        emulate_tty=True,
        output='screen',
        remappings=[
            ('/scan_filtered', '/scan/filtered')],
        arguments=[
            '--ros-args',
            '--log-level', ['scan_filter:=', LaunchConfiguration('log-level')]]
    )

    # laser_filter_node = Node(
    #     package='laser_filters',
    #     namespace='',
    #     executable='scan_to_scan_filter_chain',
    #     name='scan_filter',
    #     parameters=[sicks300_param_file],
    #     emulate_tty=True,
    #     output='screen',
    #     remappings=[
    #         ('/scan_filtered', '/scan/filtered')],
    #     arguments=[
    #         '--ros-args',
    #         '--log-level', ['scan_filter:=', LaunchConfiguration('log-level')]]
    # )

    # When the sick node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_sick_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sicks300_2_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(sicks300_2_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the sick node reaches the 'active' state, start the filter node.
    register_event_handler_for_sick_reaches_active_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sicks300_2_node,
            goal_state='active',
            entities=[
                filter_node
            ],
        )
    )

    # Make the sicks300_2 node take the 'configure' transition.
    emit_event_to_request_that_sick_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(sicks300_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        sicks300_param_file_launch_arg,
        declare_log_level_arg,
        register_event_handler_for_sick_reaches_inactive_state,
        register_event_handler_for_sick_reaches_active_state,
        sicks300_2_node,
        emit_event_to_request_that_sick_does_configure_transition,
    ])
