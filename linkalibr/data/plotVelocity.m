clear all;
clc;
data = csvread('lin_velocity_undistortedcloud.csv');
velocity = data(:,1:3);
sigma_velocity = data(:, 4:6);

%%

figure('Name','Velocity KF','NumberTitle','off');
subplot(311)
plot(velocity(:,1), 'LineWidth', 3);
hold on;
plot(velocity(:,1) + sigma_velocity(:,1), '--r', 'LineWidth', 3);
hold on;
plot(velocity(:,1) - sigma_velocity(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('v_x');
grid;
title('v_x time plot');
legend('Estimate', '1 \sigma bound');
subplot(312)
plot(velocity(:,2), 'LineWidth', 3);
hold on;
plot(velocity(:,2) + sigma_velocity(:,2), '--r', 'LineWidth', 3);
hold on;
plot(velocity(:,2) - sigma_velocity(:,2), '--r', 'LineWidth', 3);
hold off;
ylabel('v_y');
grid;
title('v_y time plot');
legend('Estimate', '1 \sigma bound');
subplot(313)
plot(velocity(:,3), 'LineWidth', 3);
hold on;
plot(velocity(:,3) + sigma_velocity(:,3), '--r', 'LineWidth', 3);
hold on;
plot(velocity(:,3) - sigma_velocity(:,3), '--r', 'LineWidth', 3);
hold off;
ylabel('v_z');
grid;
title('v_z time plot');
legend('Estimate', '1 \sigma bound');