clear all;
close all;
clc;
data_ext_calib_far1_undistorted = csvread('lin_calib_extrinsics_far1_undistortedcloud.csv');

quat_undistorted = data_ext_calib_far1_undistorted(:, 1:4);
xyz_undistorted = data_ext_calib_far1_undistorted(:, 5:7);
sigma_xyz_undistorted = data_ext_calib_far1_undistorted(:, 11:13);
sigma_rxryrz_undistorted = data_ext_calib_far1_undistorted(:, 8:10)*180/pi;
%%
figure('Name','Calibration KF','NumberTitle','off');
subplot(611)
plot(xyz_undistorted(:,1), 'LineWidth', 3);
%hold off;
hold on;
% title('Calib X [m]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

subplot(612)
plot(xyz_undistorted(:,2), 'LineWidth', 3);
%hold off;
hold on;
% title('Calib Y [m]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

subplot(613)
plot(xyz_undistorted(:,3), 'LineWidth', 3);
%hold off;
hold on;
% title('Calib Z [m]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

%%
eulerangleDegrees = [];
for i=1:length(quat_undistorted)
    quat_i = quaternion(quat_undistorted(i, 4), quat_undistorted(i, 1), quat_undistorted(i, 2), quat_undistorted(i, 3));
    eulerAngles = eulerd(quat_i, 'XYZ', 'frame');
    euler_x = wrapTo360(eulerAngles(:, 1));
    euler_y = eulerAngles(:, 2);
    euler_z = eulerAngles(:, 3);
    eulerangleDegrees = [eulerangleDegrees; [euler_x, euler_y, euler_z]];
end
eulerangleDegreesMinus = eulerangleDegrees - sigma_rxryrz_undistorted;
eulerangleDegreesPlus = eulerangleDegrees + sigma_rxryrz_undistorted;

%%
% figure('Name','Rotation Calibration KF','NumberTitle','off');
subplot(614)
plot(eulerangleDegrees(:,1), 'LineWidth', 3);
hold on;
% title('Euler X [deg]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([170, 190]);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

subplot(615)
plot(eulerangleDegrees(:,2), 'LineWidth', 3);
hold on;
% title('Euler Y [deg]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([-10, 10]);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

subplot(616)
plot(eulerangleDegrees(:,3), 'LineWidth', 3);
hold on;
% title('Euler Z [deg]','fontweight','bold','fontsize',16);
% xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([-10, 10]);
% legend('Estimate', '1 \sigma bound');
% set(gca,'FontSize', 24);

%%
data_ext_calib_far1_undistorted = csvread('lin_calib_extrinsics_far1_rawcloud.csv');

quat_undistorted = data_ext_calib_far1_undistorted(:, 1:4);
xyz_undistorted = data_ext_calib_far1_undistorted(:, 5:7);
sigma_xyz_undistorted = data_ext_calib_far1_undistorted(:, 11:13);
sigma_rxryrz_undistorted = data_ext_calib_far1_undistorted(:, 8:10)*180/pi;
%%
subplot(611)
plot(xyz_undistorted(:,1), 'LineWidth', 3);
%hold off;
hold off;
ylabel('x [m]','fontsize',36);
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');
title('Impact of deskewing on calibration results','fontweight','bold','fontsize',36);
subplot(612)
plot(xyz_undistorted(:,2), 'LineWidth', 3);
%hold off;
hold off;
ylabel('y [m]');
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');

subplot(613)
plot(xyz_undistorted(:,3), 'LineWidth', 3);
%hold off;
hold off;
ylabel('z [m]');
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');

%%
eulerangleDegrees = [];
for i=1:length(quat_undistorted)
    quat_i = quaternion(quat_undistorted(i, 4), quat_undistorted(i, 1), quat_undistorted(i, 2), quat_undistorted(i, 3));
    eulerAngles = eulerd(quat_i, 'XYZ', 'frame');
    euler_x = wrapTo360(eulerAngles(:, 1));
    euler_y = eulerAngles(:, 2);
    euler_z = eulerAngles(:, 3);
    eulerangleDegrees = [eulerangleDegrees; [euler_x, euler_y, euler_z]];
end
eulerangleDegreesMinus = eulerangleDegrees - sigma_rxryrz_undistorted;
eulerangleDegreesPlus = eulerangleDegrees + sigma_rxryrz_undistorted;

%%
% figure('Name','Rotation Calibration KF','NumberTitle','off');
subplot(614)
plot(eulerangleDegrees(:,1), 'LineWidth', 3);
hold off;
ylabel('R [deg]');
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');

subplot(615)
plot(eulerangleDegrees(:,2), 'LineWidth', 3);
hold off;
ylabel('P [deg]');
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');

subplot(616)
plot(eulerangleDegrees(:,3), 'LineWidth', 3);
hold off;
ylabel('Y [deg]');
grid;
legend('deskewed', 'not deskewed','fontweight','bold');
set(gca,'FontSize', 16,'fontweight','bold');




