from matplotlib import pyplot
from pandas import read_csv
import numpy as np
import os

def ResampleData(data, time, num):
  step = (time[-1] - time[0])/num
  time_resampled = np.arange(time[0],time[-1],step)
  data_resampled = np.zeros([np.size(time_resampled,0),np.size(data,1)])
  for i in range(len(time_resampled)):
    index = np.where(time_resampled[i] <= time)[0][0]
    index = 1 if index <= 0 else index
    rate = (time_resampled[i]-time[index-1])/(time[index]-time[index-1])
    data_resampled[i,:] = (1-rate)*data[index-1,:]+rate*data[index,:]
  return time_resampled, data_resampled

if __name__ == '__main__':
  file_path = os.path.dirname(__file__)
  prefix = file_path+"/../data/"
  dataset_m = read_csv(prefix+"delay_master.csv", header=0, parse_dates=False, index_col=False, dtype='float64')
  # dataset_s = read_csv(prefix+"delay_slave.csv", header=0, parse_dates=False, index_col=False, dtype='float64')
  
  prune_length = 100
  
  # time_master = dataset_m['time'].to_numpy()
  # time_slave = dataset_s['time'].to_numpy()
  # min_time = max([min(time_master),min(time_slave)])
  # max_time = min([max(time_master),max(time_slave)])

  # dataset_m = dataset_m[dataset_m['time']>=min_time]
  # dataset_m = dataset_m[dataset_m['time']<=max_time]
  # dataset_s = dataset_s[dataset_s['time']>=min_time]
  # dataset_s = dataset_s[dataset_s['time']<=max_time]
  dataset_m = dataset_m[prune_length:]
  # dataset_s = dataset_s[prune_length:]
  
  dataset_m.reset_index(drop=True, inplace=True)
  # dataset_s.reset_index(drop=True, inplace=True)
  
  time_master = dataset_m["time"].to_numpy()
  # time_slave = dataset_s["time"].to_numpy()
  time_master = (time_master-time_master[0])/1000.
  # time_slave = (time_slave-time_slave[0])/1000.
  n = (time_master[-1] - time_master[0])/0.01
  
  tau_ms_estimation = dataset_m["tau_ms_estimation"].to_numpy().reshape(-1,1)
  tau_sm_truth = dataset_m["tau_sm_truth"].to_numpy().reshape(-1,1)
  tau_ms_raw = dataset_m["tau_ms_raw"].to_numpy().reshape(-1,1)
  # tau_sm_estimation = dataset_s["tau_sm_estimation"].to_numpy().reshape(-1,1)
  # tau_sm_raw = dataset_s["tau_sm_raw"].to_numpy().reshape(-1,1)
  # tau_ms_truth = dataset_s["tau_ms_truth"].to_numpy().reshape(-1,1)
  
  
  # time, tau_ms_estimation = ResampleData(tau_ms_estimation,time_master,n)
  # _, tau_sm_truth = ResampleData(tau_sm_truth,time_master,n)
  # _, tau_ms_raw = ResampleData(tau_ms_raw,time_master,n)
  
  # _, tau_sm_estimation = ResampleData(tau_sm_estimation,time_slave,n)
  # _, tau_sm_raw = ResampleData(tau_sm_raw,time_slave,n)
  # _, tau_ms_truth = ResampleData(tau_ms_truth,time_slave,n)
  
  # mse_ms_estimation = np.mean((tau_ms_estimation-tau_ms_truth)**2)
  # mse_sm_estimation = np.mean((tau_sm_estimation-tau_sm_truth)**2)
  # mse_ms_raw = np.mean((tau_ms_raw-tau_ms_truth)**2)
  # mse_sm_raw = np.mean((tau_sm_raw-tau_sm_truth)**2)
  
  # print("ms_estimation:",mse_ms_estimation,"sm_estimation:",mse_sm_estimation,
  #       "ms_raw:",mse_ms_raw,"sm_raw:",mse_sm_raw)
  
  # pyplot.plot(time,tau_ms_estimation,label='estimation_m2s',alpha=0.8)
  # pyplot.plot(time,tau_ms_truth,label='truth_m2s',alpha=0.8)
  # pyplot.plot(time,tau_ms_raw,label='raw_m2s',alpha=0.8)
  
  # pyplot.plot(time_slave,tau_sm_estimation,label='estimation_s2m',alpha=0.8)
  pyplot.plot(time_master,tau_sm_truth,label='truth_s2m',alpha=0.8)
  # pyplot.plot(time_slave,tau_sm_raw,label='raw_s2m',alpha=0.8)
  
  pyplot.legend()
  pyplot.show()