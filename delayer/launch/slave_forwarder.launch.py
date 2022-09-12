from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  parameter_path = os.path.join(get_package_share_directory('delayer'), 'configs', 'slave_forwarder.yaml')
  
  return LaunchDescription([
    Node(package='delayer',
         node_namespace=None,
         node_executable='forwarder',
         node_name='slave_forwarder',
         output='screen',
         prefix=['stdbuf -o L'],
         parameters=[parameter_path],
         remappings=None,
         arguments=None),
  ])