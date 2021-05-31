clear all;
clc;
data = csvread('lin_trajectory_near1_undistortedcloud.csv');
rpy = data(:, 2:5);
xyz = data(:, 6:8);
% sigma_rpy = data(:, 7:9);
% sigma_xyz = data(:, 10:12);

%%
% figure('Name','Trajectory RPY KF','NumberTitle','off');
% subplot(311)
% plot(rpy(:,1), 'LineWidth', 3);
% hold off;
% hold on;
% plot(rpy(:,1) + 3*sigma_rpy(:,1), '--r', 'LineWidth', 3);
% hold on;
% plot(rpy(:,1) - 3*sigma_rpy(:,1), '--r', 'LineWidth', 3);
% hold off;
% ylabel('R');
% grid;
% title('R time plot');
% subplot(312)
% plot(rpy(:,2), 'LineWidth', 3);
% hold off;
% hold on;
% plot(rpy(:,2) + 3*sigma_rpy(:,2), '--r', 'LineWidth', 3);
% hold on;
% plot(rpy(:,2) - 3*sigma_rpy(:,2), '--r', 'LineWidth', 3);
% hold off;
% ylabel('P');
% grid;
% title('P time plot');
% subplot(313)
% plot(rpy(:,3), 'LineWidth', 3);
% hold off;
% hold on;
% plot(rpy(:,3) + 3*sigma_rpy(:,3), '--r', 'LineWidth', 3);
% hold on;
% plot(rpy(:,3) - 3*sigma_rpy(:,3), '--r', 'LineWidth', 3);
% hold off;
% ylabel('Y');
% grid;
% title('Y time plot');

%%
figure('Name','Trajectory 3D XYZ KF','NumberTitle','off');
plot3(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'LineWidth', 3);
grid;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('XYZ 3D plot');

% %%
% figure('Name','Trajectory XYZ KF','NumberTitle','off');
% subplot(311)
% plot(xyz(:,1), 'LineWidth', 3);
% hold off;
% %hold on;
% %plot(xyz(:,1) + 3*sigma_xyz(:,1), '--r', 'LineWidth', 3);
% %hold on;
% %plot(xyz(:,1) - 3*sigma_xyz(:,1), '--r', 'LineWidth', 3);
% %hold off;
% ylabel('X');
% grid;
% title('X time plot');
% subplot(312)
% plot(xyz(:,2), 'LineWidth', 3);
% hold off;
% %hold on;
% %plot(xyz(:,2) + 3*sigma_xyz(:,2), '--r', 'LineWidth', 3);
% %hold on;
% %plot(xyz(:,2) - 3*sigma_xyz(:,2), '--r', 'LineWidth', 3);
% %hold off;
% ylabel('Y');
% grid;
% title('Y time plot');
% subplot(313)
% plot(xyz(:,3), 'LineWidth', 3);
% hold off;
% %hold on;
% %plot(xyz(:,3) + 3*sigma_xyz(:,3), '--r', 'LineWidth', 3);
% %hold on;
% %plot(xyz(:,3) - 3*sigma_xyz(:,3), '--r', 'LineWidth', 3);
% %hold off;
% ylabel('Z');
% grid;
% title('Z time plot');


