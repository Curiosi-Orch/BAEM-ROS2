#!/usr/bin/env python3

import time
import yaml
from ament_index_python.packages import get_package_share_directory

class Clock:
  def __init__(self, config_path: str = "clock.yaml") -> None:
    '''
      config_path: path relative to controller/configs/
    '''
    self.offset_time = 0.
    self.config_path = get_package_share_directory("controller")+"/configs/"+config_path
  
  def read_configs(self) -> None:
    with open(self.config_path, 'r') as stream:
      data = yaml.safe_load(stream)
      self.offset_time = data["offset_time"]
      
  def get_time_ms(self) -> float:
    return time.time()*1000. + self.offset_time
  
  def get_time_ms_without_offset(self) -> float:
    return time.time()*1000.