clear all;
clc;

data_raw = csvread('lo_trajectory_rawcloud.csv');
rpy_raw = data_raw(:, 1:3);
xyz_raw = data_raw(:, 4:6);

data_undistortedcloud = csvread('lo_trajectory_undistortedcloud.csv');
rpy_undistorted = data_undistortedcloud(:, 1:3);
xyz_undistorted = data_undistortedcloud(:, 4:6);

%%
figure('Name','Raw vs Undistorted LOAM Trajectory XYZ','NumberTitle','off');
subplot(311)
plot(xyz_raw(:,1), '-r', 'LineWidth', 3);
hold on;
plot(xyz_undistorted(:,1), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('X');
grid;
subplot(312)
plot(xyz_raw(:,2), '-r', 'LineWidth', 3);
hold on;
plot(xyz_undistorted(:,2), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('Y');
grid;
subplot(313)
plot(xyz_raw(:,3), '-r', 'LineWidth', 3);
hold on;
plot(xyz_undistorted(:,3), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('Z');
grid;

%%
figure('Name','Raw vs Undistorted LOAM Trajectory RPY','NumberTitle','off');
subplot(311)
plot(rpy_raw(:,1), '-r', 'LineWidth', 3);
hold on;
plot(rpy_undistorted(:,1), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('R');
grid;
subplot(312)
plot(rpy_raw(:,2), '-r', 'LineWidth', 3);
hold on;
plot(rpy_undistorted(:,2), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('P');
grid;
subplot(313)
plot(rpy_raw(:,3), '-r', 'LineWidth', 3);
hold on;
plot(rpy_undistorted(:,3), 'LineWidth', 3);
hold off
legend('Raw', 'Undistorted', "location", 'southwest');
ylabel('Y');
grid;