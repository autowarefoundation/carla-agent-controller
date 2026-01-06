from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('carla_agent_controller'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='carla_agent_controller',
            namespace='carla_agent_controller',
            executable='carla_agent_controller',
            name='carla_agent_controller',
            parameters=[param],
        ),
    ])