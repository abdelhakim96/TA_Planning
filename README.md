# TA_Planning


## Run ROS-Bridge
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```


## Run neptune
```bash
cd ~/catkin_ws_neptune && bash src/neptune/neptune_single_benchmark.sh
```



## Kill nodes
```bash
rosnode kill -a
tmux kill-server
killall xterm
```
## Build with Debug
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```


## Run eampc
```bash
cd ~/catkin_ws && bash src/eampc/run_eampc.sh
```
