#!/bin/bash
source /opt/ros/hydro/setup.zsh
#source /opt/ros/fuerte/setup.zsh
#export ROS_PACKAGE_PATH=~/Projects:~/ros_packages:$ROS_PACKAGE_PATH
#source ~/moveit/devel/setup.zsh
source ~/catkin_ws/devel/setup.zsh
#export ROBOT=B2
export ROBOT=C2
#export ROBOT=D2
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_MASTER_URI=http://192.168.0.200:11311

#ROS_MASTER_URI=http://192.168.0.23:11311
#export ROS_MASTER_URI=http://192.168.66.2:11311
#export ROS_MASTER_URI=http://192.168.1.9:11311
#export ROS_IP=192.168.1.9
export ROS_MASTER_URI=http://192.168.56.1:11311
export ROS_IP=192.168.56.1
#export ROS_IP=192.168.66.10
#export ROS_IP=127.0.0.1
#export ROS_IP=192.168.0.200
#export ROS_IP=192.168.0.23
#export ROS_IP=192.168.0.1
#export ROS_MASTER_URI=http://128.237.173.51:11311
ulimit -c unlimited

