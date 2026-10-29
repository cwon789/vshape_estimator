from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ros2_bot_estimator_lidar_vshape'), 'config', 'vshape.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='ros2_bot_estimator_lidar_vshape', executable='vshape_node',
             name='vshape_estimator', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
