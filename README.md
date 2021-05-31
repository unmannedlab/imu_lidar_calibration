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

As far the 3D Lidar is considered, currently this code-base supports `Ouster-128` but it is easy to expand for other 3D Lidars. We have tested this code-base by downsampling our `Ouster 128` lidar to 64, 32, 16 channel modes. The important pre-requisite is that the points in lidar pointcloud must come with a measurement/firing timestamp. We use a Vectornav VN 300 IMU.

### Data collection for extrinsic calibration
We need to excite all degrees of freedom during collecting data required for extrinsic calibration. An example video can be found here: 

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

#### Estimated inter-sensor translation parameter
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/calibXYZ.jpg?raw=true)

#### Estimated Inter-sensor rotation parameter
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/calibEulerXYZ.jpg?raw=true)

Some more figures...

#### Estimated IMU Trajectory
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/trajectoryXYZ.jpg?raw=true)

#### Result of scan matching using motion compensated scans
![alt text](https://github.com/SubMishMar/imu_lidar_calibration/blob/main/figures/map.png?raw=true)
