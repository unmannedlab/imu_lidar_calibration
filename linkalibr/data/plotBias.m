clear all;
clc;
data = csvread('lin_bias_undistortedcloud.csv');
ba = data(:, 1:3);
bg = data(:, 4:6);
sigma_ba = data(:, 7:9);
sigma_bg = data(:, 10:12);

%%
figure('Name','Accelerometer Bias KF','NumberTitle','off');
subplot(311)
plot(ba(:,1), 'LineWidth', 3);
hold on;
plot(ba(:,1) + sigma_ba(:,1), '--r', 'LineWidth', 3);
hold on;
plot(ba(:,1) - sigma_ba(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('ba_x');
grid;
title('ba_x time plot');
subplot(312)
plot(ba(:,2), 'LineWidth', 3);
hold on;
plot(ba(:,2) + sigma_ba(:,2), '--r', 'LineWidth', 3);
hold on;
plot(ba(:,2) - sigma_ba(:,2), '--r', 'LineWidth', 3);
hold off;
ylabel('ba_y');
grid;
title('ba_y time plot');
subplot(313)
plot(ba(:,3), 'LineWidth', 3);
hold on;
plot(ba(:,3) + sigma_ba(:,3), '--r', 'LineWidth', 3);
hold on;
plot(ba(:,3) - sigma_ba(:,3), '--r', 'LineWidth', 3);
hold off;
ylabel('ba_z');
grid;
title('ba_z time plot');
%%
figure('Name','Gyroscope Bias KF','NumberTitle','off');
subplot(311)
plot(bg(:,1), 'LineWidth', 3);
hold on;
plot(bg(:,1) + sigma_bg(:,1), '--r', 'LineWidth', 3);
hold on;
plot(bg(:,1) - sigma_bg(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('bg_x');
grid;
title('bg_x time plot');
subplot(312)
plot(bg(:,2), 'LineWidth', 3);
hold on;
plot(bg(:,2) + sigma_bg(:,2), '--r', 'LineWidth', 3);
hold on;
plot(bg(:,2) - sigma_bg(:,2), '--r', 'LineWidth', 3);
hold off;
ylabel('bg_y');
grid;
title('bg_y time plot');
subplot(313)
plot(bg(:,3), 'LineWidth', 3);
hold on;
plot(bg(:,3) + sigma_bg(:,3), '--r', 'LineWidth', 3);
hold on;
plot(bg(:,3) - sigma_bg(:,3), '--r', 'LineWidth', 3);
hold off;
ylabel('bg_z');
title('bg_z time plot');
grid;