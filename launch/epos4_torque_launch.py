import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('epos4_drive')
    default_params_file = os.path.join(pkg_share, 'config', 'epos4_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file.'
    )

    epos4_node = Node(
        package='epos4_drive',
        executable='epos4_torque_node',
        name='epos4_torque_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        epos4_node
    ])

