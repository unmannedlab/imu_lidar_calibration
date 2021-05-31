# imu_lidar_calibration
## Target-free Extrinsic Calibration of a 3D Lidar and an IMU

## Overview

This repository is a toolkit for calibrating the 6-DoF rigid transformation between a 3D LiDAR and an IMU. It's based on an Extended Kalman Filter based algorithm which exploits the motion based calibration constraint for state update. This algorithm does not depend on any calibration target or special environmental features, like planes.

## Prerequisites 
This code base was tested an implemented in a Ubuntu 16.04 system.
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Kinetic)
- [GTSAM](https://gtsam.org/build/) (This will be soon replaced by [Ceres](http://ceres-solver.org/installation.html))
- [ndt_omp](https://github.com/APRIL-ZJU/ndt_omp) 

## Install

- Clone the source code for [ndt_omp](https://github.com/APRIL-ZJU/ndt_omp) and build it in your catkin workspace
- Clone this code-base and build it in your catkin workspace

## Usage

Currently this code-base supports `Ouster-128` but it is easy to expand for other 3D Lidars. We have tested this code-base by downsampling our `Ouster 128` lidar to 64, 32, 16 channel modes.

### Inter-sensor rotation estimation
`roslaunch linkalibr ros_calib_init.launch`
Please take care to change the file path names.

### Inter-sensor translation estimation
`roslaunch linkalibr linkalibr_ouster_vectornav.launch`
Please take care to change the file path names.

The results are stored in folder `data` as a homogenous transformation matrix in text file `I_T_L_final.txt`

## Plots
We can plot the results of the EKF based estimation process by using the MATLAB plot files available in folder `linkalibr/data`. 

The plot for calibration parameters is shown below:

#### Inter-sensor translation parameter
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/calibXYZ.jpg?raw=true)

#### Inter-sensor rotation parameter
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/calibEulerXYZ.jpg?raw=true)



