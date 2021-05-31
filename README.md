# imu_lidar_calibration
## Target-free Extrinsic Calibration of a 3D Lidar and an IMU

## Overview

This repository is a toolkit for calibrating the 6-DoF rigid transformation between a 3D LiDAR and an IMU. It's based on an Extended Kalman Filter based algorithm which exploits the motion based calibration constraint for state update.

## Prerequisites 
This code base was tested an implemented in a Ubuntu 16.04 system.
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Kinetic)
- [GTSAM](https://gtsam.org/build/) (This will be soon replaced by [Ceres](http://ceres-solver.org/installation.html))
- [ndt_omp](https://github.com/APRIL-ZJU/ndt_omp) 

## Install

- Clone the source code for [ndt_omp](https://github.com/APRIL-ZJU/ndt_omp) and build it in your catkin workspace
- Clone this code-base and build it in your catkin workspace

## Example

Currently this codebase supports `Ouster-128` but it is easy to expand for other LiDARs. 

