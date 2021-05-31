clear all;
clc;
data = csvread('lin_velocity_rawcloud.csv');
velocity = data(:,1:3);
sigma_velocity = data(:, 4:6);

%%

figure('Name','Velocity KF','NumberTitle','off');
subplot(311)
plot(velocity(:,1));
hold on;
plot(velocity(:,1) + 3*sigma_velocity(:,1), '--r');
hold on;
plot(velocity(:,1) - 3*sigma_velocity(:,1), '--r');
hold off;
ylabel('v_x');
grid;
title('v_x time plot');
subplot(312)
plot(velocity(:,2));
hold on;
plot(velocity(:,2) + 3*sigma_velocity(:,2), '--r');
hold on;
plot(velocity(:,2) - 3*sigma_velocity(:,2), '--r');
hold off;
ylabel('v_y');
grid;
title('v_y time plot');
subplot(313)
plot(velocity(:,3));
hold on;
plot(velocity(:,3) + 3*sigma_velocity(:,3), '--r');
hold on;
plot(velocity(:,3) - 3*sigma_velocity(:,3), '--r');
hold off;
ylabel('v_z');
grid;
title('v_z time plot');