#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from messages.msg import MultiDoubleArrayStamped, DoubleArray
from ament_index_python.packages import get_package_prefix
import sys
import numpy as np

import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryPredictor(Node):
  def __init__(self) -> None:
    super().__init__("trajectory_predictor")
    workspace_path_prefix = get_package_prefix("controller")+"/../../src/"
    
    self.declare_parameter('sub_trajectory_prediction_list_topic_name')
    self.declare_parameter('pub_trajectory_prediction_list_topic_name')
    self.declare_parameter('prediction_project_path')
    self.declare_parameter('prediction_model_dirname')
    
    sub_trajectory_prediction_list_topic_name = self.get_parameter('sub_trajectory_prediction_list_topic_name').value
    pub_trajectory_prediction_list_topic_name = self.get_parameter('pub_trajectory_prediction_list_topic_name').value
    prediction_project_path = workspace_path_prefix+self.get_parameter('prediction_project_path').value
    prediction_model_dirname = self.get_parameter('prediction_model_dirname').value
    
    sys.path.append(prediction_project_path)
    from tools.predict import predict, get_prediction_model
    self.predict = predict
    
    model_path = prediction_project_path+"/results/"+prediction_model_dirname
    self.model, self.scaler, self.params, self.device = get_prediction_model(model_path)
    
    self.sub_trajectory_prediction_list = self.create_subscription(MultiDoubleArrayStamped, sub_trajectory_prediction_list_topic_name,
                                                              self.trajectory_prediction_list_callback, 1)
    self.pub_trajectory_prediction_list = self.create_publisher(MultiDoubleArrayStamped, pub_trajectory_prediction_list_topic_name,1)
    rclpy.spin(self)
    
  def trajectory_prediction_list_callback(self, msg):
    data = []
    data.append(msg.data[0].data)
    data.append(msg.data[1].data)
    data.append(msg.data[2].data)
    prediction = self.predict(self.model, self.scaler, self.params, data, self.device)
    
    # figure = pyplot.figure()
    # axis = figure.gca(projection = '3d') # projection = '3d'
    # axis.set_xlim(-100,100)
    # axis.set_ylim(-50,100)
    # axis.set_zlim(-50,10)
    # axis.set_xlabel('x (mm)')
    # axis.set_ylabel('y (mm)')
    # axis.set_zlabel('z (mm)')
    
    # publish the prediction list with the timestamp of input
    msg_x = DoubleArray()
    msg_y = DoubleArray()
    msg_z = DoubleArray()
    data_msg = []
    msg_x.data = prediction[:,0].tolist()
    data_msg.append(msg_x)
    msg_y.data = prediction[:,1].tolist()
    data_msg.append(msg_y)
    msg_z.data = prediction[:,2].tolist()
    data_msg.append(msg_z)
    
    msg_output = MultiDoubleArrayStamped()
    msg_output.timestamp = msg.timestamp
    msg_output.data = data_msg
    
    # axis.plot(np.array(msg.data[0].data),
    #           np.array(msg.data[1].data),
    #           np.array(msg.data[2].data),'bo-',markersize=3)
    # axis.plot(np.array(msg_output.data[0].data),
    #           np.array(msg_output.data[1].data),
    #           np.array(msg_output.data[2].data),'ro-',markersize=3)
    # axis.plot(prediction[:,0],prediction[:,1],prediction[:,2],'bo-',markersize=3)
    # pyplot.show()
    
    self.pub_trajectory_prediction_list.publish(msg_output)
    
    
def main(args=None):
  rclpy.init()
  trajectory_predictor = TrajectoryPredictor()
  trajectory_predictor.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()