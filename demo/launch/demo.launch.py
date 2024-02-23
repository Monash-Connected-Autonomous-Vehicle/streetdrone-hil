import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=os.path.join(get_package_share_directory(
                'demo'), 'config', 'objects.json')
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_id',
            default_value='control'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),

				launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'passive': launch.substitutions.LaunchConfiguration('passive'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_spawn_objects.launch.py')
            ),
				    launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
                'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle'),
                'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'set_initial_pose.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'control_id': launch.substitutions.LaunchConfiguration('control_id')
            }.items()
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
