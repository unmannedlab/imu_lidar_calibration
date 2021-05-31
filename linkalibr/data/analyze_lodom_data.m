clear all;
clc;
lodom_data = csvread('lo_trajectory.csv');

xyz = lodom_data(:, 6:8);
timestamp1 = lodom_data(1, 1)/1e9;
timestamps = lodom_data(:, 1)/1e9 - timestamp1;

tx = xyz(:, 1); ty = xyz(:, 2); tz = xyz(:, 3);

subplot(311)
plot(timestamps, tx, 'LineWidth', 3);
hold off;
grid;
ylabel('X', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 15);
subplot(312)
plot(timestamps, ty, 'LineWidth', 3);
hold off;
grid;
ylabel('Y', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 15);
subplot(313)
plot(timestamps, tz, 'LineWidth', 3);
hold off;
grid;
ylabel('Z', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 15);