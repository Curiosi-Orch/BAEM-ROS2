import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from messages.msg import MultiDoubleArray, PoseCommand, PoseState, JointCommand, JointState
import threading
from controller.clock import Clock
import math
import random

class MessageInfo:
  def __init__(self, publisher, time_pub, msg) -> None:
    self.publisher = publisher
    self.time_pub = time_pub
    self.msg = msg

class Forwarder(Node):
  def __init__(self):
    super().__init__("forwarder")
    
    self.declare_parameter('sub_timestamp_topic_name')
    self.declare_parameter('pub_timestamp_topic_name')
    self.declare_parameter('sub_delay_truth_topic_name')
    self.declare_parameter('pub_delay_truth_topic_name')
    self.declare_parameter('sub_delay_remote_estimation_list_topic_name')
    self.declare_parameter('pub_delay_remote_estimation_list_topic_name')
    self.declare_parameter('sub_joint_state_topic_name')
    self.declare_parameter('pub_joint_state_topic_name')
    self.declare_parameter('sub_pose_state_topic_name')
    self.declare_parameter('pub_pose_state_topic_name')
    self.declare_parameter('sub_joint_command_topic_name')
    self.declare_parameter('pub_joint_command_topic_name')
    self.declare_parameter('sub_pose_command_topic_name')
    self.declare_parameter('pub_pose_command_topic_name')
    self.declare_parameter('sub_pose_raw_topic_name')
    self.declare_parameter('pub_pose_raw_topic_name')
    
    self.declare_parameter('A_0')
    self.declare_parameter('A_1')
    self.declare_parameter('A_2')
    self.declare_parameter('A_3')
    self.declare_parameter('T_1')
    self.declare_parameter('T_2')
    self.declare_parameter('Phi_1')
    self.declare_parameter('Phi_2')
    
    sub_timestamp_topic_name = self.get_parameter('sub_timestamp_topic_name').value
    pub_timestamp_topic_name = self.get_parameter('pub_timestamp_topic_name').value
    sub_delay_truth_topic_name = self.get_parameter('sub_delay_truth_topic_name').value
    pub_delay_truth_topic_name = self.get_parameter('pub_delay_truth_topic_name').value
    sub_delay_remote_estimation_list_topic_name = self.get_parameter('sub_delay_remote_estimation_list_topic_name').value
    pub_delay_remote_estimation_list_topic_name = self.get_parameter('pub_delay_remote_estimation_list_topic_name').value
    
    sub_joint_state_topic_name = self.get_parameter('sub_joint_state_topic_name').value
    pub_joint_state_topic_name = self.get_parameter('pub_joint_state_topic_name').value
    sub_pose_state_topic_name = self.get_parameter('sub_pose_state_topic_name').value
    pub_pose_state_topic_name = self.get_parameter('pub_pose_state_topic_name').value
    sub_joint_command_topic_name = self.get_parameter('sub_joint_command_topic_name').value
    pub_joint_command_topic_name = self.get_parameter('pub_joint_command_topic_name').value
    sub_pose_command_topic_name = self.get_parameter('sub_pose_command_topic_name').value
    pub_pose_command_topic_name = self.get_parameter('pub_pose_command_topic_name').value
    sub_pose_raw_topic_name = self.get_parameter('sub_pose_raw_topic_name').value
    pub_pose_raw_topic_name = self.get_parameter('pub_pose_raw_topic_name').value
    
    self.A_0 = self.get_parameter('A_0').value
    self.A_1 = self.get_parameter('A_1').value
    self.A_2 = self.get_parameter('A_2').value
    self.A_3 = self.get_parameter('A_3').value
    self.T_1 = self.get_parameter('T_1').value
    self.T_2 = self.get_parameter('T_2').value
    self.Phi_1 = self.get_parameter('Phi_1').value
    self.Phi_2 = self.get_parameter('Phi_2').value

    self.sub_timestamp = self.create_subscription(Float64, sub_timestamp_topic_name,
                                                  self.timestamp_callback, 1)
    self.pub_timestamp = self.create_publisher(Float64, pub_timestamp_topic_name, 1)
    self.sub_delay_truth = self.create_subscription(Float64, sub_delay_truth_topic_name,
                                                  self.delay_truth_callback, 1)
    self.pub_delay_truth = self.create_publisher(Float64, pub_delay_truth_topic_name, 1)
    self.sub_delay_remote_estimation_list = self.create_subscription(MultiDoubleArray, 
                                                                     sub_delay_remote_estimation_list_topic_name,
                                                                     self.delay_remote_estimation_list_callback, 1)
    self.pub_delay_remote_estimation_list = self.create_publisher(MultiDoubleArray,
                                                                  pub_delay_remote_estimation_list_topic_name, 1)
    # Slave site
    if sub_joint_state_topic_name != '' and pub_joint_state_topic_name != '' and \
       sub_pose_state_topic_name != '' and pub_pose_state_topic_name != '':
      self.sub_joint_state = self.create_subscription(JointState, sub_joint_state_topic_name,
                                                     self.joint_state_callback, 1)
      self.pub_joint_state = self.create_publisher(JointState, pub_joint_state_topic_name, 1)
      self.sub_pose_state = self.create_subscription(PoseState, sub_pose_state_topic_name,
                                                     self.pose_state_callback, 1)
      self.pub_pose_state = self.create_publisher(PoseState, pub_pose_state_topic_name, 1)
    # Master site
    if sub_pose_command_topic_name != '' and pub_pose_command_topic_name != '' and \
       sub_pose_raw_topic_name != '' and pub_pose_raw_topic_name != '' and \
       sub_joint_command_topic_name != '' and pub_joint_command_topic_name != '':
      self.sub_joint_command = self.create_subscription(JointCommand, sub_joint_command_topic_name,
                                                     self.joint_command_callback, 1)
      self.pub_joint_command = self.create_publisher(JointCommand, pub_joint_command_topic_name, 1)
      self.sub_pose_raw = self.create_subscription(PoseCommand, sub_pose_raw_topic_name,
                                                     self.pose_raw_callback, 1)
      self.pub_pose_raw = self.create_publisher(PoseCommand, pub_pose_raw_topic_name, 1)
      self.sub_pose_command = self.create_subscription(PoseCommand, sub_pose_command_topic_name,
                                                     self.pose_command_callback, 1)
      self.pub_pose_command = self.create_publisher(PoseCommand, pub_pose_command_topic_name, 1)
      
    self.delay_message_stack = list()
    self.robot_message_stack = list()
    
    self.clock = Clock()
    self.current_time = self.clock.get_time_ms()
    self.current_delay = 0.
    
    self.thread_publish_delay_message = threading.Thread(target=self.publish_delay_loop)
    self.thread_publish_robot_message = threading.Thread(target=self.publish_robot_loop)
    self.thread_update_time = threading.Thread(target=self.update_time_loop)
    self.thread_delay_generation = threading.Thread(target=self.delay_generation_loop)
    self.thread_publish_delay_message.start()
    self.thread_publish_robot_message.start()
    self.thread_update_time.start()
    self.thread_delay_generation.start()
    
    rclpy.spin(self)
    
  def publish_delay_loop(self):
    while rclpy.ok():
      for info in self.delay_message_stack:
        if self.current_time >= info.time_pub:
          info.publisher.publish(info.msg)
          self.delay_message_stack.remove(info)
          
  def publish_robot_loop(self):
    while rclpy.ok():
      for info in self.robot_message_stack:
        if self.current_time >= info.time_pub:
          info.publisher.publish(info.msg)
          self.robot_message_stack.remove(info)
    
  def update_time_loop(self):
    while rclpy.ok():
      self.current_time = self.clock.get_time_ms()

  def delay_generation_loop(self):
    while rclpy.ok():
      func_1 = math.sin(self.current_time*2*math.pi/self.T_1+self.Phi_1) if self.A_1 != 0 else 0
      func_2 = math.sin(self.current_time*2*math.pi/self.T_2+self.Phi_2) if self.A_2 != 0 else 0
      self.current_delay = self.A_0 + self.A_1*func_1 + self.A_2*func_2 + self.A_3 * random.gauss(0,1)
  
  def timestamp_callback(self, msg):
    self.delay_message_stack.append(MessageInfo(self.pub_timestamp,self.current_time+self.current_delay,msg))

  def delay_truth_callback(self, msg):
    self.delay_message_stack.append(MessageInfo(self.pub_delay_truth,self.current_time+self.current_delay,msg))
    
  def delay_remote_estimation_list_callback(self, msg):
    self.delay_message_stack.append(MessageInfo(self.pub_delay_remote_estimation_list,self.current_time+self.current_delay,msg))
  
  def joint_state_callback(self, msg):
    self.robot_message_stack.append(MessageInfo(self.pub_joint_state,self.current_time+self.current_delay,msg))
    
  def pose_state_callback(self, msg):
    self.robot_message_stack.append(MessageInfo(self.pub_pose_state,self.current_time+self.current_delay,msg))
  
  def joint_command_callback(self, msg):
    self.robot_message_stack.append(MessageInfo(self.pub_joint_command,self.current_time+self.current_delay,msg))
  
  def pose_raw_callback(self, msg):
    self.robot_message_stack.append(MessageInfo(self.pub_pose_raw,self.current_time+self.current_delay,msg))
    
  def pose_command_callback(self, msg):
    self.robot_message_stack.append(MessageInfo(self.pub_pose_command,self.current_time+self.current_delay,msg))
  
def main(args=None):
  rclpy.init()
  forwarder = Forwarder()
  forwarder.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()