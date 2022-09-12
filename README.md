# BAEM-Delayer-ROS2
## Get started
This repository includes ROS2 packages for Bilateral Active Estimation Model Delayer, 
where 'messages' folder generates the customized messages, 
and 'delayer' folder has the main nodes:  
1. delayer        : communication node 
2. delay_recoder  : recording node
3. delay_predictor: prediction node


## Prerequisites
1. Python 3.6
2. Ubuntu 18.04
3. ROS2 dashing
4. Pytorch 10.2


## System time synchronization for both sites
1. Install ntp & ntpdate & nptstat
2. In server:
Edit file /etc/ntp.conf:  
  comment out [restrict -4 default kod notrap nomodify nopeer noquery limited]  
  comment out [restrict -6 default kod notrap nomodify nopeer noquery limited]  
  add [restrict -4 default kod notrap nomodify]  
  add [restrict -6 default kod notrap nomodify]  
  sudo service ntp restart  
3. In client:
Edit file /etc/ntp.conf:  
  comment out [pool 0.ubuntu.pool.ntp.org iburst]  
  comment out [pool 1.ubuntu.pool.ntp.org iburst]  
  comment out [pool 2.ubuntu.pool.ntp.org iburst]  
  comment out [pool 3.ubuntu.pool.ntp.org iburst]  
  comment out [pool ntp.ubuntu.com]  
  add [server <server_ip>]  
  sudo service ntp restart  
4. The synchronization process will last for around 10 minutes, you can use 
[watch ntpq -p] in client to monitor the process where:  
  poll - the time interval between two synchronization, would get larger  
  reach - count of successful connection with server  
  delay - the time for a request to travel to and back from server  
  offset - adjustment to client time (the smaller the better)  
  jitter - a factor judging the precision of synchronization (the smaller the better)  
5. REMEMBER TO STOP NTP SERVICE WHEN CONTROL START!


## ROS connection between two sites
1. add export ROS_DOMAIN_ID=10 in ~/.bashrc of both PCs
2. source ~/.bashrc


## Time delay generation - TC(traffic control):
1. Use [sudo tc qdisc add dev ___ root netem delay 5ms 1ms 30% distribution normal]
to generate delay of 5±1ms with the next random element depending 30% on the last one  
in normal distribution 
2. Use [sudo tc qdisc del dev eth0 root] to delete the delay


## Environment settings
### Create a new anaconda environment
conda create -n BAEM python=3.6  
conda activate BAEM

### Python dependencies
1. Reinstall setuptools version
pip uninstall setuptools
pip install setuptools==44.1.1
2. Install Pytorch
CPU version:  
conda install pytorch torchvision torchaudio cpuonly -c pytorch  
CUDA version:  
conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch  
3. Other installations
pip install toml  
pip install joblib  
pip install sklearn  
pip install pandas  
pip install matplotlib==2.1.1
pip install tqdm

### Create ROS2 workspace
1. mkdir BAEM_ROS2_ws  
2. cd BAEM_ROS2_ws  
3. git clone [this repository]  
4. rename the folder as "src"

### Environment setup
source src/setup.bash

### Compilement
colcon build

## Run
source install/setup.bash
1. In master site
ros2 launch delayer master_delayer.launch.py
2. In slave site
ros2 launch delayer slave_delayer.launch.py
3. (a) In master site
ros2 launch delayer master_delay_recorder.py
3. (b) In slave site
ros2 launch delayer slave_delay_recorder.py
