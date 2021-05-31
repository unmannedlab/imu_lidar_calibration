clear all;
clc;

format long g

data_plane_params = csvread('plane_params.csv');
data_imu_trajectory = csvread('imu_trajectory.csv');
data_lidar_trajectory = csvread('lidar_trajectory.csv');
data_imu_trajectory_out = csvread('imu_trajectory_out.csv');

%%
data_planar_points_raw_scans = csvread('planar_points_raw_scans.csv');
data_planar_points_deskewed_map = csvread('planar_points_deskewed_map.csv');
data_planar_points_preint_map = csvread('planar_points_deskewed_preint_map.csv');
%%
figure(1)
subplot(311)
plot(data_imu_trajectory(:, 6));
hold on;
plot(data_imu_trajectory_out(:, 6));
hold off;
legend('x in', 'x out');
grid;
subplot(312)
plot(data_imu_trajectory(:, 7));
hold on;
plot(data_imu_trajectory_out(:, 7));
hold off;
legend('y in', 'y out');
grid;
subplot(313)
plot(data_imu_trajectory(:, 8));
hold on;
plot(data_imu_trajectory_out(:, 8));
hold off;
legend('z in', 'z out');
grid;
%%
data_imu_trajectory_qxqyqzqw = data_imu_trajectory(:, 2:5);
data_imu_trajectory_out_qxqwqzqw = data_imu_trajectory_out(:, 2:5);
euler_in = [];
euler_out = [];
for i = 1:length(data_imu_trajectory_qxqyqzqw)
    quat_in = quaternion(data_imu_trajectory_qxqyqzqw(i, 4), data_imu_trajectory_qxqyqzqw(i, 1),... 
                         data_imu_trajectory_qxqyqzqw(i, 2), data_imu_trajectory_qxqyqzqw(i, 3));
    euler_in = [euler_in; eulerd(quat_in, 'XYZ', 'frame')];                
    quat_out = quaternion(data_imu_trajectory_out_qxqwqzqw(i, 4), data_imu_trajectory_out_qxqwqzqw(i, 1),... 
                          data_imu_trajectory_out_qxqwqzqw(i, 2), data_imu_trajectory_out_qxqwqzqw(i, 3));
    euler_out = [euler_out; eulerd(quat_out, 'XYZ', 'frame')];
end

figure(2)
subplot(311)
plot(euler_in(:, 1));
hold on;
plot(euler_out(:, 1));
hold off;
grid;

subplot(312)
plot(euler_in(:, 2));
hold on;
plot(euler_out(:, 2));
hold off;
grid;

subplot(313)
plot(euler_in(:, 3));
hold on;
plot(euler_out(:, 3));
hold off;
grid;

%%
figure(3);
figure(4);
figure(5);
for i = 1:1:length(data_plane_params)
    idx_plane_i = find(data_planar_points_preint_map(:,4)== (i-1));
    plane_param_i = data_plane_params(i,:);
    deskewed_map_points_on_plane_i = data_planar_points_deskewed_map(idx_plane_i,5:7);
    preint_map_points_on_plane_i = data_planar_points_preint_map(idx_plane_i, 5:7);
    data_raw_scan_points_on_plane_i = data_planar_points_raw_scans(idx_plane_i,:);
    raw_scan_points_on_plane_i_projected_on_map = [];
    for j = 1:length(data_raw_scan_points_on_plane_i)
         scan_id = data_raw_scan_points_on_plane_i(j, 3) + 1;
         
         pose_k = data_lidar_trajectory(scan_id, 2:8);
         orientation_k = pose_k(1:4);
         translation_k = pose_k(5:7);
         L1_R_Lk = quat2rotm(quaternion(orientation_k(4), orientation_k(1), orientation_k(2), orientation_k(3)));
         L1_t_Lk = translation_k';
         L1_T_Lk = [L1_R_Lk L1_t_Lk; 0 0 0 1];
         
         point_L1 = L1_R_Lk*data_raw_scan_points_on_plane_i(j, 5:7)'+L1_t_Lk;
         raw_scan_points_on_plane_i_projected_on_map = [raw_scan_points_on_plane_i_projected_on_map; point_L1'];
    end
    
    if(~isempty(preint_map_points_on_plane_i))
        figure(3)
        plot3(raw_scan_points_on_plane_i_projected_on_map(:, 1),... 
              raw_scan_points_on_plane_i_projected_on_map(:, 2),... 
              raw_scan_points_on_plane_i_projected_on_map(:, 3), '.');
        hold on;
        
        figure(4)
        plot3(preint_map_points_on_plane_i(:, 1),...
              preint_map_points_on_plane_i(:, 2),...
              preint_map_points_on_plane_i(:, 3), '.');
        hold on;
        
        figure(5)
        plot3(deskewed_map_points_on_plane_i(:, 1),... 
              deskewed_map_points_on_plane_i(:, 2),... 
              deskewed_map_points_on_plane_i(:, 3), '.');
        hold on;
    end
end
figure(3);
hold off;
grid;
axis equal;

figure(4);
hold off;
grid;
axis equal;

figure(5);
hold off;
grid;
axis equal;

%%
data_surfel_map = csvread('surfel_map.csv');
figure(4)
plot3(data_surfel_map(:, 1), data_surfel_map(:, 2), data_surfel_map(:, 3), '.');
grid;
axis equal;

%%
data_initial_calib = csvread('init_calib_csv.csv');
data_final_calib = csvread('final_calib_csv.csv');

figure(4)
subplot(311)
plot(data_initial_calib(:, 1));
hold on;
plot(data_final_calib(:, 1));
hold off;
grid;

subplot(312)
plot(data_initial_calib(:, 2));
hold on;
plot(data_final_calib(:, 2));
hold off;
grid;

subplot(313)
plot(data_initial_calib(:, 3));
hold on;
plot(data_final_calib(:, 3));
hold off;
grid;

figure(5)
subplot(311)
plot(data_initial_calib(:, 4));
hold on;
plot(data_final_calib(:, 4));
hold off;
grid;

subplot(312)
plot(data_initial_calib(:, 5));
hold on;
plot(data_final_calib(:, 5));
hold off;
grid;

subplot(313)
plot(data_initial_calib(:, 6));
hold on;
plot(data_final_calib(:, 6));
hold off;
grid;

%%
data_velocity = data_imu_trajectory_out(:, 9:11);
figure(6)
subplot(311)
plot(data_velocity(:, 1));
grid;
subplot(312)
plot(data_velocity(:, 2));
grid;
subplot(313)
plot(data_velocity(:, 3));
grid;

