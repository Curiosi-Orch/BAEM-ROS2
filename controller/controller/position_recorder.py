#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from messages.msg import PoseState
from messages.msg import PoseCommand
from ament_index_python.packages import get_package_prefix
import threading
import time
import csv
from controller.clock import Clock

class PositionData:
  def __init__(self, timestamp: float, 
                     position_state: list,
                     position_command: list,
                     position_raw: list) -> None:
    self.timestamp = timestamp
    self.position_state = position_state
    self.position_command = position_command
    self.position_raw = position_raw

class PositionRecorder(Node):
  def __init__(self) -> None:
    super().__init__("position_recorder")
    
    workspace_path_prefix = get_package_prefix("controller")
    prefix_path = workspace_path_prefix+"/../../src/"
    
    self.declare_parameter('sub_position_state_topic_name')
    self.declare_parameter('sub_position_command_topic_name')
    self.declare_parameter('sub_position_raw_topic_name')
    self.declare_parameter('save_path')
    self.declare_parameter('frequency')
    self.declare_parameter('duration')
    
    self.sub_position_state_topic_name = self.get_parameter('sub_position_state_topic_name').value
    self.sub_position_command_topic_name = self.get_parameter('sub_position_command_topic_name').value
    self.sub_position_raw_topic_name = self.get_parameter('sub_position_raw_topic_name').value
    self.save_path = prefix_path+self.get_parameter('save_path').value
    self.duration = self.get_parameter('duration').value * 1000.
    self.frequency = self.get_parameter('frequency').value
    
    self.sub_position_state = self.create_subscription(PoseState, self.sub_position_state_topic_name, 
                                                       self.position_state_callback, 1)
    self.sub_position_command = self.create_subscription(PoseCommand, self.sub_position_command_topic_name, 
                                                         self.position_command_callback, 1)
    self.sub_position_raw = self.create_subscription(PoseCommand, self.sub_position_raw_topic_name,
                                                     self.position_raw_callback, 1)
    
    self.clock = Clock()
    self.current_time = self.clock.get_time_ms()
    self.start_time = self.clock.get_time_ms()
    self.current_position_state = [0,0,0]
    self.current_position_command = [0,0,0]
    self.current_position_raw = [0,0,0]
    self.position_data = []
    self.is_started = False
    self.thread_recording = threading.Thread(target=self.task_recording)
    self.thread_update_time = threading.Thread(target=self.task_update_time)
    self.thread_recording.start()
    self.thread_update_time.start()
    
    rclpy.spin(self)
  
  def task_recording(self):
    while rclpy.ok():
      if self.is_started:
        base_time = self.current_time
        end_time = base_time+1000./self.frequency
        temp_data = PositionData(self.current_time,
                                 self.current_position_state,
                                 self.current_position_command,
                                 self.current_position_raw)
        self.position_data.append(temp_data)
        if self.current_time >= self.start_time+self.duration:
          self.save_data()
          self.destroy_node()
          rclpy.shutdown()
          break
        current_time = self.current_time
        sleep_time = (end_time-current_time)/1000. if end_time > current_time else 0
        time.sleep(sleep_time)
    
  def task_update_time(self):
    while rclpy.ok():
      self.current_time = self.clock.get_time_ms()
      time.sleep(0.0001)
      
  def position_state_callback(self, msg):
    self.current_position_state = [msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z]
    
  def position_command_callback(self, msg):
    self._start_recording()
    self.current_position_command = [msg.pose.position.x,
                                     msg.pose.position.y,
                                     msg.pose.position.z]
    
  def position_raw_callback(self, msg):
    self._start_recording()
    self.current_position_raw = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]
  
  def _start_recording(self):
    if not self.is_started:
      self.start_time = self.clock.get_time_ms()
      self.is_started = True
      self.get_logger().info(f"Start Recording!\tDuration: {self.duration/1000} s")
      
  def save_data(self):
    with open(self.save_path,"w") as csvfile: 
      writer = csv.writer(csvfile)
      writer.writerow(["time","position_state","position_command","position_raw"])

      for data in self.position_data:
        writer.writerow([data.timestamp,
                         data.position_state,
                         data.position_command,
                         data.position_raw])
    print("Successfully record position data to: ", self.save_path)
  
def main(args=None):
  rclpy.init()
  position_recorder = PositionRecorder()
  
if __name__ == '__main__':
  main()