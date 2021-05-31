clear all;
clc;
data = csvread('lo_trajectory_undistortedcloud.csv');
rpy = data(:, 1:3);
xyz = data(:, 4:6);
%%
figure('Name','Undistorted LOAM Trajectory','NumberTitle','off');
subplot(611)
plot(xyz(:,1), 'LineWidth', 3);
hold off
ylabel('X','fontweight','bold','fontsize',16);
grid;
axis([0, length(xyz), min(xyz(:,1)), max(xyz(:,1))])

subplot(612)
plot(xyz(:,2), 'LineWidth', 3);
hold off
ylabel('Y','fontweight','bold','fontsize',16);
grid;
axis([0, length(xyz), min(xyz(:,2)), max(xyz(:,2))])

subplot(613)
plot(xyz(:,3), 'LineWidth', 3);
hold off
ylabel('Z','fontweight','bold','fontsize',16);
grid;
axis([0, length(xyz), min(xyz(:,3)), max(xyz(:,3))])

subplot(614)
plot(rpy(:,1), 'LineWidth', 3);
hold off
ylabel('ax','fontweight','bold','fontsize',16);
grid;
axis([0, length(rpy), min(rpy(:,1)), max(rpy(:,1))])

subplot(615)
plot(rpy(:,2), 'LineWidth', 3);
hold off
ylabel('ay','fontweight','bold','fontsize',16);
grid;
axis([0, length(rpy), min(rpy(:,2)), max(rpy(:,2))])

subplot(616)
plot(rpy(:,3), 'LineWidth', 3);
hold off
ylabel('az','fontweight','bold','fontsize',16);
grid;
axis([0, length(rpy), min(rpy(:,3)), max(rpy(:,3))])