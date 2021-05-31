clear all;
clc;
imu_data = csvread('imu_test_data.csv');
expected_freq = length(imu_data)*1e9/(imu_data(end, 1)-imu_data(1, 1));
expected_deltaT = 1000/expected_freq % in ms
% Process IMU data
deltaTs = [];
times = [];
t_first = 1000*imu_data(1, 1)/1e9;
for i=2:length(imu_data)
    t1 = 1000*imu_data(i-1, 1)/1e9;
    t2 = 1000*imu_data(i, 1)/1e9;
    deltaT = (t2-t1);
    deltaTs = [deltaTs deltaT];
    times = [times (t2-t_first)/1000];
end

deltaTsExpected = 1000*(1/expected_freq) * ones(1, length(deltaTs));

figure(1)
p = plot(times, deltaTs, '.');
hold on;
q = plot(times, deltaTsExpected, '--', 'LineWidth', 6);
hold off;
grid;
lgd = legend(q, 'expected \DeltaT');
lgd.FontSize = 20;
ylabel('\DeltaT in ms', 'FontSize', 36);
xlabel('Time [s]', 'FontSize', 36);
title('\DeltaT in ms between IMU measurements', 'FontSize', 36);
no_grtr = length(find(deltaTs > 1000*(1/expected_freq)))
no_lesser =  length(find(deltaTs <= 1000*(1/expected_freq)))
ylim([0, 2*max(deltaTs)]);