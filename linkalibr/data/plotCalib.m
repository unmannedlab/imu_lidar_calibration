clear all;
clc;
data_ext_calib = csvread('lin_calib_extrinsics_undistortedcloud.csv');
quat = data_ext_calib(:, 1:4);
xyz = data_ext_calib(:, 5:7);
sigma_xyz = data_ext_calib(:, 11:13);
sigma_rxryrz = data_ext_calib(:, 8:10)*180/pi;
data_dt_calib = csvread('lin_calib_dt_undistortedcloud.csv');
dt = data_dt_calib(:, 1);
sigma_dt = data_dt_calib(:, 2);
%%
figure('Name','Calibration XYZ KF','NumberTitle','off');
subplot(311)
plot(xyz(:,1), 'LineWidth', 3);
%hold off;
hold on;
plot(xyz(:,1) + sigma_xyz(:,1), '--r', 'LineWidth', 3);
hold on;
plot(xyz(:,1) - sigma_xyz(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib X [m]','fontweight','bold','fontsize',16);
grid;
title('Calib X [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(312)
plot(xyz(:,2), 'LineWidth', 3);
%hold off;
hold on;
plot(xyz(:,2) + sigma_xyz(:,2), '--r', 'LineWidth', 3);
hold on;
plot(xyz(:,2) - sigma_xyz(:,2), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib Y [m]','fontweight','bold','fontsize',16);
grid;
title('Calib Y [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(313)
plot(xyz(:,3), 'LineWidth', 3);
%hold off;
hold on;
plot(xyz(:,3) + sigma_xyz(:,3), '--r', 'LineWidth', 3);
hold on;
plot(xyz(:,3) - sigma_xyz(:,3), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib Z [m]','fontweight','bold','fontsize',16);
grid;
title('Calib Z [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);


%%
eulerangleDegrees = [];
for i=1:length(quat)
    quat_i = quaternion(quat(i, 4), quat(i, 1), quat(i, 2), quat(i, 3));
    eulerAngles = eulerd(quat_i, 'XYZ', 'frame');
    euler_x = wrapTo360(eulerAngles(:, 1));
    euler_y = eulerAngles(:, 2);
    euler_z = eulerAngles(:, 3);
    eulerangleDegrees = [eulerangleDegrees; [euler_x, euler_y, euler_z]];
end
eulerangleDegreesMinus = eulerangleDegrees - sigma_rxryrz;
eulerangleDegreesPlus = eulerangleDegrees + sigma_rxryrz;

%%
figure('Name','Rotation Calibration KF','NumberTitle','off');
subplot(311)
plot(eulerangleDegrees(:,1), 'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,1), '--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('Euler X','fontweight','bold','fontsize',16);
grid;
title('Euler X [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
ylim([170, 190]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(312)
plot(eulerangleDegrees(:,2), 'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,2),'--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,2),'--r',  'LineWidth', 3);
hold off;
ylabel('Euler Y','fontweight','bold','fontsize',16);
grid;
title('Euler Y [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
ylim([-10, 10]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(313)
plot(eulerangleDegrees(:,3), 'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,3),'--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,3),'--r',  'LineWidth', 3);
hold off;
ylabel('Euler Z','fontweight','bold','fontsize',16);
grid;
title('Euler Z [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
ylim([-10, 10]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);


