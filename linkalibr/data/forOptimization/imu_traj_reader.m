clear all;
clc;

format long g


data_imu_trajectory = csvread('imu_trajectory.csv');
data_imu_trajectory_out = csvread('imu_trajectory_out.csv');

for i = 1:length(data_imu_trajectory)
    
end