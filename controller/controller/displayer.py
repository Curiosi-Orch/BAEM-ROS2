#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from messages.msg import PoseCommand, PoseState
import numpy as np
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import threading
import time


class Displayer(Node):
  def __init__(self):
    super().__init__("displayer")
    
    self.declare_parameter('sub_pose_command_topic_name')
    self.declare_parameter('sub_pose_raw_topic_name')
    self.declare_parameter('sub_pose_state_topic_name')
    self.declare_parameter('input_stack_length')
    
    sub_pose_command_topic_name = self.get_parameter('sub_pose_command_topic_name').value
    sub_pose_raw_topic_name = self.get_parameter('sub_pose_raw_topic_name').value
    sub_pose_state_topic_name = self.get_parameter('sub_pose_state_topic_name').value
    input_stack_length = self.get_parameter('input_stack_length').value
    
    self.sub_pose_command = self.create_subscription(PoseCommand, sub_pose_command_topic_name, 
                                                     self.pose_command_callback, 1)
    self.sub_pose_raw = self.create_subscription(PoseCommand, sub_pose_raw_topic_name, 
                                                     self.pose_raw_callback, 1)
    self.sub_pose_state = self.create_subscription(PoseState, sub_pose_state_topic_name, 
                                                     self.pose_state_callback, 1)
    self.pose_command_stack = np.zeros((input_stack_length, 3))
    self.pose_raw_stack = np.zeros((input_stack_length, 3))
    self.pose_state_stack = np.zeros((input_stack_length, 3))
    
    self.thread_plot = threading.Thread(target=self.plot_callback)
    self.thread_plot.start()

    rclpy.spin(self)  
  
  def plot_callback(self):
    figure = pyplot.figure()
    pyplot.ion()
    while rclpy.ok():
      figure.clf()
      axis = figure.gca(projection = '3d')
      axis.set_xlabel('x (mm)')
      axis.set_ylabel('y (mm)')
      axis.set_zlabel('z (mm)')
      axis.set_xlim([175,275])
      axis.set_ylim([-50,50])
      axis.set_zlim([600,700])
      axis.plot(self.pose_command_stack[:,0],
                self.pose_command_stack[:,1],
                self.pose_command_stack[:,2],'bo-',markersize=3,label='command')
      axis.plot(self.pose_raw_stack[:,0],
                self.pose_raw_stack[:,1],
                self.pose_raw_stack[:,2],'go-',markersize=3,label='raw')
      axis.plot(self.pose_state_stack[:,0],
                self.pose_state_stack[:,1],
                self.pose_state_stack[:,2],'ro-',markersize=3,label='state')
      axis.legend(loc='upper left')
      figure.canvas.draw()
      pyplot.pause(0.0001)

  def pose_command_callback(self, msg) -> None:
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    position = np.array([x,y,z])
    self.pose_command_stack = np.concatenate([self.pose_command_stack[1:,:],position.reshape((1,3))],axis=0)
  
  def pose_raw_callback(self, msg) -> None:
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    position = np.array([x,y,z])
    self.pose_raw_stack = np.concatenate([self.pose_raw_stack[1:,:],position.reshape((1,3))],axis=0)
    
  def pose_state_callback(self, msg) -> None:
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    position = np.array([x,y,z])
    self.pose_state_stack = np.concatenate([self.pose_state_stack[1:,:],position.reshape((1,3))],axis=0)
    
def main(args = None):
  rclpy.init()
  displayer = Displayer()
  pyplot.ioff()
  pyplot.show()
  displayer.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()