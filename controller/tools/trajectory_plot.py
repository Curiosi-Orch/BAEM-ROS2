import numpy as np
import pandas as pd
import matplotlib.pyplot as pyplot
import os

def parse_data_from_string_list(str_list: list):
  data = np.array([]).reshape(-1,3)
  for row in str_list:
    str = row.replace('[','').replace(']','').split(',')
    n = list(map(np.float32, str))
    data = np.append(data, np.array(n).reshape(1,3), axis=0)
  return data

if __name__ == "__main__":
  path = os.path.dirname(__file__)
  dir_name = "data_20220811/120_100/tele"
  prefix = path+"/../data/"+dir_name
  df_master_N = pd.read_csv(prefix+"/position_master_N.csv")
  df_slave_N = pd.read_csv(prefix+"/position_slave_N.csv")
  df_master_T = pd.read_csv(prefix+"/position_master_T.csv")
  df_slave_T = pd.read_csv(prefix+"/position_slave_T.csv")
  df_master_TC = pd.read_csv(prefix+"/position_master_TC.csv")
  df_slave_TC = pd.read_csv(prefix+"/position_slave_TC.csv")
  df_master_TCM = pd.read_csv(prefix+"/position_master_TCM.csv")
  df_slave_TCM = pd.read_csv(prefix+"/position_slave_TCM.csv")
  
  time_master_N = df_master_N['time'].to_numpy() - df_master_N['time'].to_numpy()[0]
  time_master_T = df_master_T['time'].to_numpy() - df_master_T['time'].to_numpy()[0]
  time_master_TC = df_master_TC['time'].to_numpy() - df_master_TC['time'].to_numpy()[0]
  time_master_TCM = df_master_TCM['time'].to_numpy() - df_master_TCM['time'].to_numpy()[0]
  
  time_slave_N = df_slave_N['time'].to_numpy() - df_master_N['time'].to_numpy()[0]
  time_slave_T = df_slave_T['time'].to_numpy() - df_master_T['time'].to_numpy()[0]
  time_slave_TC = df_slave_TC['time'].to_numpy() - df_master_TC['time'].to_numpy()[0]
  time_slave_TCM = df_slave_TCM['time'].to_numpy() - df_master_TCM['time'].to_numpy()[0]

  truth_N = parse_data_from_string_list(df_master_N['position_raw'].to_list())
  truth_T = parse_data_from_string_list(df_master_T['position_raw'].to_list())
  truth_TC = parse_data_from_string_list(df_master_TC['position_raw'].to_list())
  truth_TCM = parse_data_from_string_list(df_master_TCM['position_raw'].to_list())
  # truth_N = np.concatenate([np.zeros([10,3]),truth_N[:-10,:]],axis=0)
  # truth_T = np.concatenate([np.zeros([9,3]),truth_T[:-9,:]],axis=0)
  # truth_TC = np.concatenate([np.zeros([9,3]),truth_TC[:-9,:]],axis=0)
  # truth_TCM = np.concatenate([np.zeros([17,3]),truth_TCM[:-17,:]],axis=0)
  
  state_N = parse_data_from_string_list(df_slave_N['position_state'].to_list())
  state_T = parse_data_from_string_list(df_slave_T['position_state'].to_list())
  state_TC = parse_data_from_string_list(df_slave_TC['position_state'].to_list())
  state_TCM = parse_data_from_string_list(df_slave_TCM['position_state'].to_list())
  # state_N = np.concatenate([np.zeros([10,3]),state_N[:-10,:]],axis=0)
  # state_T = np.concatenate([np.zeros([9,3]),state_T[:-9,:]],axis=0)
  # state_TC = np.concatenate([np.zeros([9,3]),state_TC[:-9,:]],axis=0)
  # state_TCM = np.concatenate([np.zeros([17,3]),state_TCM[:-17,:]],axis=0)
  
  raw_N = parse_data_from_string_list(df_slave_N['position_raw'].to_list())
  raw_T = parse_data_from_string_list(df_slave_T['position_raw'].to_list())
  raw_TC = parse_data_from_string_list(df_slave_TC['position_raw'].to_list())
  raw_TCM = parse_data_from_string_list(df_slave_TCM['position_raw'].to_list())
  # raw_N = np.concatenate([np.zeros([10,3]),raw_N[:-10,:]],axis=0)
  # raw_T = np.concatenate([np.zeros([9,3]),raw_T[:-9,:]],axis=0)
  # raw_TC = np.concatenate([np.zeros([9,3]),raw_TC[:-9,:]],axis=0)
  # raw_TCM = np.concatenate([np.zeros([17,3]),raw_TCM[:-17,:]],axis=0)
  
  command_N = parse_data_from_string_list(df_slave_N['position_command'].to_list())
  command_T = parse_data_from_string_list(df_slave_T['position_command'].to_list())
  command_TC = parse_data_from_string_list(df_slave_TC['position_command'].to_list())
  command_TCM = parse_data_from_string_list(df_slave_TCM['position_command'].to_list())
  # command_N = np.concatenate([np.zeros([10,3]),command_N[:-10,:]],axis=0)
  # command_T = np.concatenate([np.zeros([9,3]),command_T[:-9,:]],axis=0)
  # command_TC = np.concatenate([np.zeros([9,3]),command_TC[:-9,:]],axis=0)
  # command_TCM = np.concatenate([np.zeros([17,3]),command_TCM[:-17,:]],axis=0)
  
  
  # # task 
  # pyplot.plot(time_master_N/1000,truth_N[:,0],label='truth_None')
  # # pyplot.plot(time_master_T/1000,truth_T[:,0],label='truth_T')
  # # pyplot.plot(time_master_TC/1000,truth_TC[:,0],label='truth_TC')
  # # pyplot.plot(time_master_TCM/1000,truth_TCM[:,0],label='truth_TCM')
  
  # # pyplot.plot(time_slave_N/1000,command_N[:,0],label='command_None')
  # # pyplot.plot(time_slave_T/1000,command_T[:,0],label='command_T')
  # # pyplot.plot(time_slave_TC/1000,command_TC[:,0],label='command_TC')
  # # pyplot.plot(time_slave_TCM/1000,command_TCM[:,0],label='command_TCM')
  
  # # pyplot.plot(time_slave_N/1000,raw_N[:,0],label='raw_None')
  # # pyplot.plot(time_slave_T/1000,raw_T[:,0],label='raw_T')
  # # pyplot.plot(time_slave_TC/1000,raw_TC[:,0],label='raw_TC')
  # # pyplot.plot(time_slave_TCM/1000,raw_TCM[:,0],label='raw_TCM')
  
  # pyplot.plot(time_slave_N/1000,state_N[:,0],'--',label='state_None')
  # pyplot.plot(time_slave_T/1000,state_T[:,0],'--',label='state_T')
  # pyplot.plot(time_slave_TC/1000,state_TC[:,0],'--',label='state_TC')
  # pyplot.plot(time_slave_TCM/1000,state_TCM[:,0],'--',label='state_TCM')
  # pyplot.ylabel('position: x (mm)')
  # pyplot.xlabel('time (s)')
  # pyplot.legend()
  
  
  # tele
  pyplot.subplot(221)
  pyplot.plot(time_master_N/1000,truth_N[:,0],label='truth_None')
  pyplot.plot(time_slave_N/1000,state_N[:,0],label='state_None')
  pyplot.legend()
  pyplot.ylabel('position: x (mm)')
  pyplot.xlabel('time (s)')
  
  pyplot.subplot(222)
  pyplot.plot(time_master_T/1000,truth_T[:,0],label='truth_T')
  pyplot.plot(time_slave_T/1000,state_T[:,0],label='state_T')
  pyplot.legend()
  pyplot.ylabel('position: x (mm)')
  pyplot.xlabel('time (s)')
  
  pyplot.subplot(223)
  pyplot.plot(time_master_TC/1000,truth_TC[:,0],label='truth_TC')
  pyplot.plot(time_slave_TC/1000,state_TC[:,0],label='state_TC')
  pyplot.legend()
  pyplot.ylabel('position: x (mm)')
  pyplot.xlabel('time (s)')

  pyplot.subplot(224)
  pyplot.plot(time_master_TCM/1000,truth_TCM[:,0],label='truth_TCM')
  pyplot.plot(time_slave_TCM/1000,state_TCM[:,0],label='state_TCM')
  pyplot.legend()
  pyplot.ylabel('position: x (mm)')
  pyplot.xlabel('time (s)')
  
  
  pyplot.show()