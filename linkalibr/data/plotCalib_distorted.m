clear all;
clc;
data_ext_calib = csvread('lin_calib_extrinsics_rawcloud.csv');
% rpy = data(:, 1:4);
xyz = data_ext_calib(:, 5:7);
sigma_xyz = data_ext_calib(:, 11:13);

data_dt_calib = csvread('lin_calib_dt_rawcloud.csv');
dt = data_dt_calib(:, 1);
sigma_dt = data_dt_calib(:, 2);
%%
figure('Name','Calibration XYZ KF','NumberTitle','off');
subplot(311)
plot(xyz(:,1), 'LineWidth', 3);
hold off;
%hold on;
%plot(xyz(:,1) + 3*sigma_xyz(:,1), '--r', 'LineWidth', 3);
%hold on;
%plot(xyz(:,1) - 3*sigma_xyz(:,1), '--r', 'LineWidth', 3);
%hold off;
ylabel('X');
grid;
title('X time plot');
subplot(312)
plot(xyz(:,2), 'LineWidth', 3);
hold off;
%hold on;
%plot(xyz(:,2) + 3*sigma_xyz(:,2), '--r', 'LineWidth', 3);
%hold on;
%plot(xyz(:,2) - 3*sigma_xyz(:,2), '--r', 'LineWidth', 3);
%hold off;
ylabel('Y');
grid;
title('Y time plot');
subplot(313)
plot(xyz(:,3), 'LineWidth', 3);
hold off;
%hold on;
%plot(xyz(:,3) + 3*sigma_xyz(:,3), '--r', 'LineWidth', 3);
%hold on;
%plot(xyz(:,3) - 3*sigma_xyz(:,3), '--r', 'LineWidth', 3);
%hold off;
ylabel('Z');
grid;
title('Z time plot');

%%
figure('Name','Calibration XYZ KF','NumberTitle','off');

plot(dt(:,1), 'LineWidth', 3);
hold off;
%hold on;
%plot(dt(:,1) + 3*sigma_dt(:,1), '--r', 'LineWidth', 3);
%hold on;
%plot(dt(:,1) - 3*sigma_dt(:,1), '--r', 'LineWidth', 3);
%hold off;
ylabel('time offset');
grid;