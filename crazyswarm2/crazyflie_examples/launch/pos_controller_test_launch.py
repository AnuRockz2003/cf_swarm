import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    return LaunchDescription([
        Node(
            package='crazyflie',
            # executable='crazyflie_server.py',
            executable ='crazyflie_server_newtry1.py',
            name='crazyflie_server_hehe',
            output='screen',
            parameters=[server_params]
        )
        # ,
        # Node(
        #     package='crazyflie',
        #     executable='position_controller.py',
        #     name='position_controller',
        #     output='screen',
        #     parameters=[{'hover_height': 0.5},
        #                 {'incoming_twist_topic': '/cmd_vel'},
        #                 {'robot_prefix': '/cf3'}
        #                 ]
        # ),
    ])
