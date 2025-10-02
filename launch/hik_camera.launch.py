from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('hik_camera'),
        'config',
        'params.yaml'
    )
    
    hik_camera_node = Node(
        package='hik_camera',
        executable='hik_camera_node',
        name='hik_camera',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        hik_camera_node
    ])