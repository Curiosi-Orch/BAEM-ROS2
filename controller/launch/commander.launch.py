from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  parameter = os.path.join(get_package_share_directory('controller'), 'configs', 'commander.yaml')
  commander = Node(package='controller',
                   node_namespace=None,
                   node_executable='commander',
                   node_name='commander',
                   output='screen',
                   prefix=['stdbuf -o L'],
                   parameters=[parameter],
                   remappings=None,
                   arguments=None,)
  predictor = Node(package='controller',
                   node_namespace=None,
                   node_executable='trajectory_predictor.py',
                   node_name='trajectory_predictor',
                   output='screen',
                   prefix=['stdbuf -o L'],
                   parameters=[parameter],
                   remappings=None,
                   arguments=None,)
  display   = Node(package='controller',
                   node_namespace=None,
                   node_executable='displayer.py',
                   node_name='master_displayer',
                   output='screen',
                   prefix=['stdbuf -o L'],
                   parameters=[parameter],
                   remappings=None,
                   arguments=None,)
  return LaunchDescription([commander,predictor,display])