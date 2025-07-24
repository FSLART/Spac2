from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    spac_params_file = os.path.join(
        get_package_share_directory('spac2_0'),
        'config',
        'demo_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='spac2_0',
            executable='spac_node',
            name='spac_node',
            output='screen',
            parameters=[spac_params_file],

        ),
    ])