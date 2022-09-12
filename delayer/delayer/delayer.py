import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16
from messages.msg import MultiDoubleArrayStamped, MultiDoubleArray, DoubleArray
import time
import numpy as np
import threading
from controller.clock import Clock

class Delayer(Node):
  def __init__(self):
    super().__init__("delayer")

    self.declare_parameter('sub_timestamp_topic_name')
    self.declare_parameter('sub_delay_remote_estimation_list_topic_name')
    self.declare_parameter('sub_delay_local_prediction_list_topic_name')
    self.declare_parameter('pub_timestamp_topic_name')
    self.declare_parameter('pub_delay_truth_topic_name')
    self.declare_parameter('pub_delay_local_prediction_list_topic_name')
    self.declare_parameter('pub_delay_remote_estimation_list_topic_name')
    self.declare_parameter('pub_delay_computation_topic_name')
    self.declare_parameter('pub_advance_q_topic_name')
    self.declare_parameter('frequency')
    self.declare_parameter('input_stack_length')
    
    sub_timestamp_topic_name = self.get_parameter('sub_timestamp_topic_name').value
    sub_delay_remote_estimation_list_topic_name = self.get_parameter('sub_delay_remote_estimation_list_topic_name').value
    sub_delay_local_prediction_list_topic_name = self.get_parameter('sub_delay_local_prediction_list_topic_name').value
    pub_timestamp_topic_name = self.get_parameter('pub_timestamp_topic_name').value
    pub_delay_truth_topic_name = self.get_parameter('pub_delay_truth_topic_name').value
    pub_delay_local_prediction_list_topic_name = self.get_parameter('pub_delay_local_prediction_list_topic_name').value
    pub_delay_remote_estimation_list_topic_name = self.get_parameter('pub_delay_remote_estimation_list_topic_name').value
    pub_delay_computation_topic_name = self.get_parameter('pub_delay_computation_topic_name').value
    pub_advance_q_topic_name = self.get_parameter('pub_advance_q_topic_name').value
    self.frequency = self.get_parameter('frequency').value
    self.input_stack_length = self.get_parameter('input_stack_length').value
    
    self.sub_timestamp = self.create_subscription(Float64, sub_timestamp_topic_name, 
                                                  self.timestamp_callback, 1)
    self.sub_delay_remote_estimation_list = self.create_subscription(MultiDoubleArray, 
                                                  sub_delay_remote_estimation_list_topic_name, 
                                                  self.delay_remote_estimation_list_callback, 1)
    self.sub_delay_local_prediction_list = self.create_subscription(MultiDoubleArrayStamped, 
                                                  sub_delay_local_prediction_list_topic_name, 
                                                  self.delay_local_prediction_list_callback, 1)
    self.pub_timestamp = self.create_publisher(Float64, pub_timestamp_topic_name, 1)
    self.pub_delay_truth = self.create_publisher(Float64, pub_delay_truth_topic_name, 1)
    self.pub_delay_local_prediction_list = self.create_publisher(MultiDoubleArrayStamped, 
                                                  pub_delay_local_prediction_list_topic_name, 1)
    self.pub_delay_remote_estimation_list = self.create_publisher(MultiDoubleArray, 
                                                  pub_delay_remote_estimation_list_topic_name, 1)
    self.pub_delay_computation = self.create_publisher(Float64, pub_delay_computation_topic_name, 1)
    self.pub_advance_q = self.create_publisher(Int16, pub_advance_q_topic_name, 1)

    self.clock = Clock()
    self.current_time = self.clock.get_time_ms()
    self.thread_pub_timestamp = threading.Thread(target=self.task_pub_timestamp)
    self.thread_update_time = threading.Thread(target=self.task_update_time)
    self.thread_pub_timestamp.start()
    self.thread_update_time.start()
    self.input_stack = list()
    self.is_input_stack_initialized = False
    self.current_advance_q = 0.
    self.count_local_prediction = 0
    self.average_delay_computation = 0.
    
    rclpy.spin(self)
    
  def task_pub_timestamp(self):
    while rclpy.ok():
      base_time = self.current_time
      end_time = base_time+1000./self.frequency
      msg = Float64()
      msg.data = self.current_time
      self.pub_timestamp.publish(msg)
      current_time = self.current_time
      sleep_time = (end_time-current_time)/1000. if end_time > current_time else 0
      time.sleep(sleep_time)
      # time.sleep((end_time-current_time)/1000.)
    
  def task_update_time(self):
    while rclpy.ok():
      self.current_time = self.clock.get_time_ms()
      time.sleep(0.0001)

  def timestamp_callback(self, msg):
    remote_timestamp = msg.data
    delay_remote2local_truth = self.current_time - remote_timestamp
    # publish the ground truth of delay from remote to local
    msg_delay_truth = Float64()
    msg_delay_truth.data = delay_remote2local_truth
    self.pub_delay_truth.publish(msg_delay_truth)
    # self.get_logger().info(f"time:{self.current_time}, delay: {delay_remote2local_truth}")
    self._update_input_stack(delay_remote2local_truth)
    # publish local prediction input to prediction node
    msg_delay_input = MultiDoubleArrayStamped()
    msg_temp = DoubleArray()
    msg_temp.data = self.input_stack
    msg_delay_input.timestamp = self.current_time
    msg_delay_input.data = [msg_temp]
    self.pub_delay_local_prediction_list.publish(msg_delay_input)
    
  def delay_remote_estimation_list_callback(self, msg):
    delay_estimation_list = msg.data[0].data
    # self.get_logger().info(f"length of estimation: {len(delay_estimation_list)}")
    self.current_advance_q = int(self._compute_advance(delay_estimation_list))
    self.get_logger().info(f"advance q: {self.current_advance_q}")
    # publish advance q for transmission delay 
    msg_q = Int16()
    msg_q.data = self.current_advance_q
    self.pub_advance_q.publish(msg_q)
  
  def delay_local_prediction_list_callback(self, msg):
    self.count_local_prediction += 1
    time_prediction_start = msg.timestamp
    time_prediction_end = self.current_time
    temp_delay_computation = time_prediction_end - time_prediction_start
    # self.get_logger().info(f"delay computation: {temp_delay_computation}")
    self.average_delay_computation = (self.average_delay_computation*(self.count_local_prediction-1)+
                                      temp_delay_computation)/self.count_local_prediction
    prediction = msg.data[0].data
    # self.get_logger().info(f"length of prediction: {len(prediction)}")
    advance_u = int(self.current_advance_q)+int(self.average_delay_computation*self.frequency/1000.)
    if advance_u < len(prediction):
      estimation = [prediction[i] for i in range(advance_u,len(prediction))]
    else:
      estimation = [prediction[-1]]
    
    # publish computational delay
    msg_delay_computation = Float64()
    msg_delay_computation.data = temp_delay_computation
    self.pub_delay_computation.publish(msg_delay_computation)
    # publish processed prediction of local-remote delay to remote site
    msg_estimation_list = MultiDoubleArray()
    msg_temp = DoubleArray()
    msg_temp.data = estimation
    msg_estimation_list.data = [msg_temp]
    self.pub_delay_remote_estimation_list.publish(msg_estimation_list)
  
  
  def _update_input_stack(self, new_value):
    if not self.is_input_stack_initialized:
      self.input_stack = [new_value for i in range(self.input_stack_length)]
      self.is_input_stack_initialized = True
    else:
      self.input_stack.pop(0)
      self.input_stack.append(new_value)
  
  def _compute_advance(self, predictions):
    max_step = len(predictions)
    u = np.array(range(max_step))*1000/self.frequency
    error = np.array(predictions)-u
    min_index = np.argmin(np.abs(error))
    return min_index
      
    
def main(args=None):
  rclpy.init()
  delayer = Delayer()
  delayer.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()