clear all;
clc;

data_ext_calib_raw = csvread('lin_calib_extrinsics_rawcloud.csv');
xyz_raw = data_ext_calib_raw(:, 5:7);
data_ext_calib_undistorted = csvread('lin_calib_extrinsics_undistortedcloud.csv');
xyz_undistorted = data_ext_calib_undistorted(:, 5:7);

data_dt_calib_raw = csvread('lin_calib_dt_rawcloud.csv');
dt_raw = data_dt_calib_raw(:, 1);
data_dt_calib_undistorted = csvread('lin_calib_dt_undistortedcloud.csv');
dt_undistorted = data_dt_calib_undistorted(:, 1);

figure('Name','Calibration XYZ KF','NumberTitle','off');
subplot(311)
plot(xyz_raw(:, 1), '-r', 'LineWidth', 3);
hold on;
plot(xyz_undistorted(:, 1), 'LineWidth', 3);
hold off;
legend('Raw', 'Undistorted', "location", "southeast");
ylabel('X');
grid;
title('X time plot');

subplot(312)
plot(xyz_raw(:,2), '-r' ,'LineWidth', 3);
hold on;
plot(xyz_undistorted(:, 2), 'LineWidth', 3);
hold off;
legend('Raw', 'Undistorted');
ylabel('Y');
grid;
title('Y time plot');

subplot(313)
plot(xyz_raw(:,3), '-r', 'LineWidth', 3);
hold on;
plot(xyz_undistorted(:, 3), 'LineWidth', 3);
hold off;
legend('Raw', 'Undistorted');
ylabel('Z');
grid;
title('Z time plot');

figure('Name','Calibration Time-offset KF','NumberTitle','off');
plot(dt_raw(:, 1), '-r', 'LineWidth', 3);
hold on;
plot(dt_undistorted(:, 1), 'LineWidth', 3);
hold off;
legend('Raw', 'Undistorted');
ylabel('Time Offset');
grid;
title('Time Offset time plot');