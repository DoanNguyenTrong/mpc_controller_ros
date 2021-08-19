# Model Predictive Control implementation about ROS 

**Credit to**: \
https://github.com/Geonhee-LEE/mpc_ros.git 
https://github.com/DoanNguyenTrong/multiple-mpc-autonomous-cars/tree/main/udacity-CarND-MPC-Project

## Abstract

This repository is implemented with mobile robot model from bycycle to the unicycle which means differential drive wheeled mobile robot for promising tracking performance. For running this NMPC algorithm, you can use the GAZEBO simulatior or customized mobile robot and compare with DWA algorithm given defalut local planner in ROS. 


## Features
* Nonlinear Unicycle Model Based MPC (through ipopt solver)  
* AMCL localization, fake localization (encoder-odometry based)  
* GAZEBO simulation, Warthog bot

### Installation
1. Ubuntu 18.04
2. Install ROS Melodic 
3. Install ROS dependencies: 
```
sudo apt install ros-melodic-costmap-2d  ros-melodic-move-base ros-melodic-global-planner ros-melodic-amcl
```
4. Install Ipopt: Please refer the tutorial in "document/ipopt_install".  
5. create your own catkin_ws and clone the repositories. 
```
https://github.com/DoanNguyenTrong/warthog_mpc_ros.git 
```
For warthog repo, follow [this](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/WarthogInstallation.html)

## Launch

### Run Navigation algorithm with MPC in simulation: 
```
roslaunch warthog_gazebo warthog_world_slam.launch

roslaunch mpc_ros warthog_mpc_gazebo.launch 
```
It can be selected with DWA, MPC, Pure persuit according to 'controller' argument.

