
import numpy as np
import pandas as pd
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import os

def generate_circle_trajectory(center: list, 
                               normal: list, 
                               radius: float,
                               duration: float, 
                               angular_velocity: float, 
                               sample_frequency: float = 100.0,
                               noise: float = 0.):
  """
    center (list): [x,y,z], unit mm
    normal (list): [x,y,z], need not to be normalized
    radius (float): unit mm
    duration (float): unit s
    angular_velocity (float): unit rad/s
    sample_frequency (float, optional): unit Hz. Defaults to 100.0.
    noise (float, optional): Defaults to 0.
  """
  # radius = math.sqrt(math.pow((start_pos[0]-center[0]),2) + \
  #                    math.pow((start_pos[1]-center[1]),2) + \
  #                    math.pow((start_pos[2]-center[2]),2))
  num_samples = int(duration*sample_frequency)
  sample = np.linspace(0, duration*angular_velocity, num_samples)
  
  a = np.cross(normal, [1, 0, 0])
  if np.all(a == 0):
    a = np.cross(normal, [0, 1, 0])
  b = np.cross(normal, a) 
  
  a = a / np.linalg.norm(a)
  b = b / np.linalg.norm(b)

  p_x = center[0] * np.ones(num_samples) \
      + radius*a[0]*np.cos(sample) \
      + radius*b[0]*np.sin(sample)
  p_y = center[1] * np.ones(num_samples) \
      + radius*a[1]*np.cos(sample) \
      + radius*b[1]*np.sin(sample)
  p_z = center[2] * np.ones(num_samples) \
      + radius*a[2]*np.cos(sample) \
      + radius*b[2]*np.sin(sample)
  noise_x = noise * np.random.normal(0., 1, num_samples)
  noise_y = noise * np.random.normal(0., 1, num_samples)
  noise_z = noise * np.random.normal(0., 1, num_samples)
  p_x += noise_x
  p_y += noise_y
  p_z += noise_z
  return p_x, p_y, p_z

def generate_lissajous_trajectory(center: list, 
                                  A: float,
                                  B: float,
                                  a: float,
                                  b: float,
                                  delta: float,
                                  duration: float, 
                                  angular_velocity: float, 
                                  sample_frequency: float = 100.0,
                                  noise: float = 0.):
  """
    x = x0+A*sin(a(t+delta))
    y = y0+B*sin(bt)
    z = z0
    center (list): [x,y,z], unit mm
    A (float): unit mm
    B (float): unit mm
    a (float): 
    b (float): 
    delta (float): 
    duration (float): unit s
    angular_velocity (float): unit rad/s
    sample_frequency (float, optional): unit Hz. Defaults to 100.0.
    noise (float, optional): Defaults to 0.
  """
  num_samples = int(duration*sample_frequency)
  sample = np.linspace(0, duration*angular_velocity, num_samples)

  p_x = center[0] * np.ones(num_samples) \
      + A*np.cos(a*sample+delta)
  p_y = center[1] * np.ones(num_samples) \
      + B*np.cos(b*sample) 
  p_z = center[2] * np.ones(num_samples)
  noise_x = noise * np.random.normal(0., 1, num_samples)
  noise_y = noise * np.random.normal(0., 1, num_samples)
  noise_z = noise * np.random.normal(0., 1, num_samples)
  p_x += noise_x
  p_y += noise_y
  p_z += noise_z
  return p_x, p_y, p_z

if __name__ == '__main__':
  duration = 100.
  sample_frequency = 100.
  p_x, p_y, p_z = generate_circle_trajectory(center=[240,0,710],
                                            normal=[0,0,1],
                                            radius=50,
                                            duration=duration,
                                            angular_velocity=np.pi/2,
                                            noise=0,
                                            sample_frequency=sample_frequency)

  # p_x, p_y, p_z = generate_lissajous_trajectory(center=[240,0,710],
  #                                               A = 50,
  #                                               B = 50,
  #                                               a = 2,
  #                                               b = 3,
  #                                               delta=np.pi/2,
  #                                               duration=duration,
  #                                               angular_velocity=np.pi/2,
  #                                               noise=0,
  #                                               sample_frequency=sample_frequency)
  figure = pyplot.figure()
  axis = figure.gca(projection = '3d')
  axis.scatter(p_x,p_y,p_z,s=1)
  axis.set_xlabel("x(mm)")
  axis.set_ylabel("y(mm)")
  axis.set_zlabel("z(mm)")
  # axis.set_xlim([-100,100])
  pyplot.show()

  current_path = os.path.dirname(__file__)
  time = np.linspace(0,duration,np.size(p_x))
  data = pd.DataFrame({'time':time,'x':p_x,'y':p_y,'z':p_z})
  file_label = "20220816"
  data.to_csv(current_path+'/../data/trajectory_'+file_label+'.csv',index=False)