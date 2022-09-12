import rclpy
from rclpy.node import Node
from messages.msg import MultiDoubleArrayStamped, DoubleArray
from ament_index_python.packages import get_package_prefix
import sys

class DelayPredictor(Node):
  def __init__(self) -> None:
    super().__init__("delay_predictor")
    workspace_path_prefix = get_package_prefix("delayer")+"/../../src/"
    
    self.declare_parameter('sub_local_prediction_list_topic_name')
    self.declare_parameter('pub_local_prediction_list_topic_name')
    self.declare_parameter('prediction_project_path')
    self.declare_parameter('prediction_model_dirname')
    
    sub_local_prediction_list_topic_name = self.get_parameter('sub_local_prediction_list_topic_name').value
    pub_local_prediction_list_topic_name = self.get_parameter('pub_local_prediction_list_topic_name').value
    prediction_project_path = workspace_path_prefix+self.get_parameter('prediction_project_path').value
    prediction_model_dirname = self.get_parameter('prediction_model_dirname').value
    
    sys.path.append(prediction_project_path)
    from tools.predict import predict, get_prediction_model
    self.predict = predict
    
    model_path = prediction_project_path+"/results/"+prediction_model_dirname
    self.model, self.scaler, self.params, self.device = get_prediction_model(model_path)
    
    self.sub_local_prediction_list = self.create_subscription(MultiDoubleArrayStamped, sub_local_prediction_list_topic_name,
                                                              self.local_prediction_list_callback, 1)
    self.pub_local_prediction_list = self.create_publisher(MultiDoubleArrayStamped, pub_local_prediction_list_topic_name,1)
    
    rclpy.spin(self)
    
  def local_prediction_list_callback(self, msg):
    data = [msg.data[0].data]
    prediction = self.predict(self.model, self.scaler, self.params, data, self.device)
    
    # publish the prediction list with the timestamp of input
    msg_output = MultiDoubleArrayStamped()
    msg_output.timestamp = msg.timestamp
    msg_temp = DoubleArray()
    msg_temp.data = prediction[:,0].tolist()
    msg_output.data = [msg_temp]
    self.pub_local_prediction_list.publish(msg_output)
    
    
def main(args=None):
  rclpy.init()
  delay_predictor = DelayPredictor()
  delay_predictor.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()