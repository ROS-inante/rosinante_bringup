# rosinante_bringup
This repository contains the launch and configuration files for ROSinante.

# Overview
Two launch files are provided in the [launch](/launch) directory. One for use with real (or dummy) hardware and one for simulation in gazebo.
The [config](/config) directory contains config files for individual parts of the setup.

# Launch Arguments
| Parameter |  Type  | Default value | Description |
|:-----|:--------:|:---:|:---|
| `use_fake_hardware` | bool | false | Start robot with fake hardware mirroring command to its states. |
| `prefix` | string | "" | Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated. |
| `use_sim` | bool | false | Start robot in Gazebo simulation. |
| `start_rviz` | bool | false | Start RViz2 automatically with this launch file. |
| `teleop_joy` | bool | false | Configure robot for joystick teleop. |