import numpy as np
import matplotlib.pyplot as pyplot
import matplotlib.animation as animation
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import os

class TrajectoryAnimation:
  def __init__(self, path: str) -> None:
    super(TrajectoryAnimation, self).__init__()
    self.position = []
    self.time = []
    self.names = []
    self.colors = []
    self.save_path = path
    
  def add_trajectory(self, time: np.ndarray ,trajectory: np.ndarray, name: str, color: str)->None:
    self.position.append(trajectory)
    self.time.append(time)
    self.colors.append(color)
    self.names.append(name)
    
  def plot(self):
    figure = pyplot.figure()
    axis = figure.gca()
    data_length = np.size(self.position[0],0)
    labels = ['x','y','z']
    colors = ['r','g','b','grey']
    
    for i in range(len(self.names)):
      for j in range(3):
        axis.plot(self.time[i], 
                  self.position[i][:,j],color=colors[i], label=labels[j]+"_"+self.names[i]) 
    axis.legend()
    figure.savefig(self.save_path+".png")
    pyplot.show()
    
  def visualize(self):
    figure = pyplot.figure()
    # axis = figure.add_subplot(projection = '3d') # projection = '3d'
    axis = figure.gca(projection = '3d')
    axis.set_xlim(100,400)
    axis.set_ylim(-250,50)
    axis.set_zlim(900,1200)
    axis.set_xlabel('x (mm)')
    axis.set_ylabel('y (mm)')
    axis.set_zlabel('z (mm)')
    
    images = []
    for i in range(np.size(self.position[0],0)):
      images.append([axis.scatter(self.position[j][:i+1,0] if i<100 else self.position[j][i-100:i+1,0],
                                  self.position[j][:i+1,1] if i<100 else self.position[j][i-100:i+1,1],
                                  self.position[j][:i+1,2] if i<100 else self.position[j][i-100:i+1,2],
                                  s=1,c=self.colors[j],
                                  # alpha=np.linspace(0,1,i+1) if i<100 else np.linspace(0,1,101),
                                  animated=True,
                                  label=self.names[j]) for j in range(np.size(self.names))])
    
    ani = animation.ArtistAnimation(figure, images, interval=10, repeat=False)
    pyplot.show()
    ani.save(self.save_path+".gif", writer='pillow', fps=100)

def parse_data_from_string_list(str_list: list):
  data = np.array([]).reshape(-1,3)
  for row in str_list:
    str = row.replace('[','').replace(']','').split(',')
    n = list(map(np.float32, str))
    data = np.append(data, np.array(n).reshape(1,3), axis=0)
  return data
    

if __name__ == '__main__':
  path = os.path.dirname(__file__)
  dir_name = "data_20220816/D2/tele/lissajous/"
  prefix = path+"/../data/"+dir_name
  prefix = path+"/../data/"
  df_master = pd.read_csv(prefix+"/position_master.csv")
  df_slave = pd.read_csv(prefix+"/position_slave.csv")
  
  time_master = df_master['time'].to_numpy() - df_master['time'].to_numpy()[0]
  time_slave = df_slave['time'].to_numpy() - df_master['time'].to_numpy()[0]

  position_truth = df_master["position_command"].to_list()
  position_raw = df_master['position_raw'].to_list()
  # position_command = df_slave["position_command"].to_list()
  # positon_state = df_slave["position_state"].to_list()
  # position_raw = df_slave["position_raw"].to_list()
  
  position_truth = parse_data_from_string_list(position_truth)
  # position_command = parse_data_from_string_list(position_command)
  # positon_state = parse_data_from_string_list(positon_state)
  position_raw = parse_data_from_string_list(position_raw)
  
  # position_command = np.concatenate([np.zeros([8,3]),position_command[:-8,:]],axis=0)
  trajectory = TrajectoryAnimation(prefix+"/traj1")
  trajectory.add_trajectory(time_master,position_truth,"truth",'g')
  # trajectory.add_trajectory(time_slave,position_command,"command",'r')
  # trajectory.add_trajectory(time_slave,positon_state,"state",'b')
  # trajectory.add_trajectory(time_slave,position_raw[:,:],"raw",'b')
  trajectory.add_trajectory(time_master,position_raw[:,:],"raw",'b')
  
  # df_slave2 = pd.read_csv(prefix+"/position_slave_d.csv")
  # time_slave2 = df_slave2['time'].to_numpy() - df_master['time'].to_numpy()[0]
  # position_raw2 = df_slave2["position_raw"].to_list()
  # position_raw2 = parse_data_from_string_list(position_raw2)
  # trajectory.add_trajectory(time_slave2,position_raw2[:,:],"raw2",'y')
  
  # trajectory.visualize()
  trajectory.plot()
  
  # time = (df_master['time']-df_master['time'][0])/1000.
  # pyplot.plot(np.array(time),np.array(positon_state)[:,0],label='state')
  # pyplot.plot(np.array(time+0.5),np.array(position_command)[:,0],label='command')
  # pyplot.legend()
  # pyplot.show()
  