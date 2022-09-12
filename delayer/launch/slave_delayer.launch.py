from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  parameter_path = os.path.join(get_package_share_directory('delayer'), 'configs', 'slave_delayer.yaml')
  
  return LaunchDescription([
    Node(package='delayer',
         node_namespace=None,
         node_executable='delayer',
         node_name='slave_delayer',
         output='screen',
         prefix=['stdbuf -o L'],
         parameters=[parameter_path],
         remappings=None,
         arguments=None),
    Node(package='delayer',
         node_namespace=None,
         node_executable='delay_predictor',
         node_name='slave_delay_predictor',
         output='screen',
         prefix=['stdbuf -o L'],
         parameters=[parameter_path],
         remappings=None,
         arguments=None),
  ])