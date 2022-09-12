from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  parameter_path = os.path.join(get_package_share_directory('controller'), 'configs', 'slave_position_recorder_2.yaml')
  
  return LaunchDescription([
    Node(package='controller',
         node_namespace=None,
         node_executable='position_recorder.py',
         node_name='slave_position_recorder_2',
         output='screen',
         prefix=['stdbuf -o L'],
         parameters=[parameter_path],
         remappings=None,
         arguments=None),
  ])