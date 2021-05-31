clear all;
clc;
data = csvread('lo_trajectory_rawcloud.csv');
a_xyzang = data(:, 1:3);
xyz = data(:, 4:6);
%%
figure('Name','RAW LOAM Trajectory','NumberTitle','off');
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
plot(a_xyzang(:,1), 'LineWidth', 3);
hold off
ylabel('ax','fontweight','bold','fontsize',16);
grid;
axis([0, length(a_xyzang), min(a_xyzang(:,1)), max(a_xyzang(:,1))])

subplot(615)
plot(a_xyzang(:,2), 'LineWidth', 3);
hold off
ylabel('ay','fontweight','bold','fontsize',16);
grid;
axis([0, length(a_xyzang), min(a_xyzang(:,2)), max(a_xyzang(:,2))])

subplot(616)
plot(a_xyzang(:,3), 'LineWidth', 3);
hold off
ylabel('az','fontweight','bold','fontsize',16);
grid;
axis([0, length(a_xyzang), min(a_xyzang(:,3)), max(a_xyzang(:,3))])


