# Model Predictive Control ROS 

## Abstract
- MPC Cotroller node
- Remove intermediate `local_planner` to enforce follow global trajectory behavior


## Features
* Nonlinear Unicycle Model Based MPC w/ `ipopt` solver 


### Installation

- Install Ipopt: Refer to [this tutorial](docs/ipopt_install/).

## Subscribe
- `/global_planner`
- `/odom`

## Publish

- `cmd_vel`
- `cmd_vel_stmpd`
- `/mpc_trajectory`
- `/mpc_reference`


## Launch

```
roslaunch mpc_controller_ros mpc_cotroller_node.launch 
```
