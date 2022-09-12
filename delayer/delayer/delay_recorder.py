import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16
from messages.msg import MultiDoubleArray
from ament_index_python.packages import get_package_prefix
import csv
import threading
from controller.clock import Clock
import time

class DelayData:
  def __init__(self, timestamp: float, 
                     tau_raw: float,
                     tau_truth: float, 
                     tau_estimation: float,
                     tau_computation: float, 
                     advance: float) -> None:
    self.timestamp = timestamp
    self.tau_raw = tau_raw
    self.tau_truth = tau_truth
    self.tau_estimation = tau_estimation
    self.tau_computation = tau_computation
    self.advance = advance

class DelayRecorder(Node):
  def __init__(self) -> None:
    super().__init__("delay_recorder")

    workspace_path_prefix = get_package_prefix("delayer")
    prefix_path = workspace_path_prefix+"/../../src/"
    self.declare_parameter('sub_delay_truth_topic_name')
    self.declare_parameter('sub_delay_raw_topic_name')
    self.declare_parameter('sub_delay_estimation_list_topic_name')
    self.declare_parameter('sub_delay_computation_topic_name')
    self.declare_parameter('sub_advance_q_topic_name')
    self.declare_parameter('save_path')
    self.declare_parameter('frequency')
    self.declare_parameter('duration')
    
    self.sub_delay_truth_topic_name = self.get_parameter('sub_delay_truth_topic_name').value
    self.sub_delay_raw_topic_name = self.get_parameter('sub_delay_raw_topic_name').value
    self.sub_delay_estimation_list_topic_name = self.get_parameter('sub_delay_estimation_list_topic_name').value
    self.sub_delay_computation_topic_name = self.get_parameter('sub_delay_computation_topic_name').value
    self.sub_advance_q_topic_name = self.get_parameter('sub_advance_q_topic_name').value
    self.save_path = prefix_path+self.get_parameter('save_path').value
    self.duration = self.get_parameter('duration').value * 1000.
    self.frequency = self.get_parameter('frequency').value
    
    self.sub_delay_truth = self.create_subscription(Float64, self.sub_delay_truth_topic_name, 
                                                  self.delay_truth_callback, 1)
    self.sub_delay_raw = self.create_subscription(Float64, self.sub_delay_raw_topic_name, 
                                                  self.delay_raw_callback, 1)
    self.sub_delay_estimation_list = self.create_subscription(MultiDoubleArray, self.sub_delay_estimation_list_topic_name,
                                                              self.delay_estimation_list_callback, 1)
    self.sub_delay_computation = self.create_subscription(Float64, self.sub_delay_computation_topic_name,
                                                          self.delay_computation_callback, 1)
    self.sub_advance_q = self.create_subscription(Int16, self.sub_advance_q_topic_name,
                                                  self.advance_q_callback, 1)
    
    self.clock = Clock()
    self.current_time = self.clock.get_time_ms()
    self.start_time = self.clock.get_time_ms()
    self.current_tau_truth = 0
    self.current_tau_raw = 0
    self.current_tau_estimation = 0
    self.current_tau_computation = 0
    self.current_advance = 0
    self.delay_data = []
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
        temp_data = DelayData(self.current_time,
                              self.current_tau_raw,
                              self.current_tau_truth,
                              self.current_tau_estimation,
                              self.current_tau_computation,
                              self.current_advance)
        self.delay_data.append(temp_data)
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

  def delay_truth_callback(self, msg):
    self._start_recording()
    self.current_tau_truth = msg.data
    
  def delay_raw_callback(self, msg):
    self._start_recording()
    self.current_tau_raw = msg.data
    
  def delay_estimation_list_callback(self, msg):
    self._start_recording()
    self.current_tau_estimation = msg.data[0].data[0]
    
  def delay_computation_callback(self, msg):
    self._start_recording()
    self.current_tau_computation = msg.data
    
  def advance_q_callback(self, msg):
    self._start_recording()
    self.current_advance = msg.data
    
  def _start_recording(self):
    if not self.is_started:
      self.start_time = self.clock.get_time_ms()
      self.is_started = True
      self.get_logger().info(f"Start Recording!\tDuration: {self.duration/1000} s")
  
  def save_data(self):
    with open(self.save_path,"w") as csvfile: 
      writer = csv.writer(csvfile)
      delay_raw_name = "tau_ms_raw" if self.sub_delay_raw_topic_name=="tau_ms_truth_d" else "tau_sm_raw"
      delay_estimation_name = "tau_ms_estimation" if self.sub_delay_estimation_list_topic_name=="tau_ms_estimation_list_d" else "tau_sm_estimation"
      writer.writerow(["time",
                       delay_raw_name,
                       self.sub_delay_truth_topic_name,
                       delay_estimation_name,
                       self.sub_delay_computation_topic_name,
                       self.sub_advance_q_topic_name])

      for data in self.delay_data:
        writer.writerow([data.timestamp,
                         data.tau_raw,
                         data.tau_truth,
                         data.tau_estimation,
                         data.tau_computation,
                         data.advance])
    self.get_logger().info(f"Successfully record delay data to: {self.save_path}")
     
def main(args=None):
  rclpy.init()
  delay_recorder = DelayRecorder()

if __name__ == '__main__':
  main()