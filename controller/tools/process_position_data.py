from turtle import xcor
import matplotlib.pyplot as pyplot
import pandas as pd
import os
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.append(".")

def parse_data_from_string_list(str_list: list):
  data = np.array([]).reshape(-1,3)
  for row in str_list:
    str = row.replace('[','').replace(']','').split(',')
    n = list(map(np.float32, str))
    data = np.append(data, np.array(n).reshape(1,3), axis=0)
  return data

# merge and plot data
if __name__ == '__main__':
  # load data
  file_prefix = os.path.dirname(__file__)+ "/../data/data_thesis/"
  file_label = "D5/task/lissajous/position_raw.csv"
  file_name = file_prefix + "samples/" + file_label
  data_raw = pd.read_csv(file_name,header=0, parse_dates=False, index_col=False)

  time = data_raw['time'].to_numpy()
  time = time-time[0]
  position_raw = parse_data_from_string_list(data_raw['position_raw'].to_list())
  position_state = parse_data_from_string_list(data_raw['position_state'].to_list())
  position_command = parse_data_from_string_list(data_raw['position_command'].to_list())
  x_raw = position_raw[:,0]
  y_raw = position_raw[:,1]
  z_raw = position_raw[:,2]
  x_state = position_state[:,0]
  y_state = position_state[:,1]
  z_state = position_state[:,2]
  x_command = position_command[:,0]
  y_command = position_command[:,1]
  z_command = position_command[:,2]

  figure = pyplot.figure()
  axis = figure.gca(projection = '3d')
  axis.plot(x_raw,y_raw,z_raw,'o-',markersize=1)
  axis.plot(x_state,y_state,z_state,'o-',markersize=1)
  axis.plot(x_command,y_command,z_command,'o-',markersize=1)
  
  # axis_2d = figure.gca()
  # axis_2d.plot(data_master["time"],x)
  # axis_2d.plot(data_master["time"],x_p)
  pyplot.show()
  
  data =  pd.DataFrame({"time": time,
                        "x_raw": x_raw,"y_raw": y_raw,"z_raw": z_raw,
                        "x_state": x_state,"y_state": y_state,"z_state": z_state,
                        "x_command":x_command,"y_command":y_command,"z_command":z_command})
  
  data.to_csv(file_prefix+"processed/"+file_label,index=False)
  