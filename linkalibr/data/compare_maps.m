clear all;
close all;
clc;
data_distorted = csvread('map_csv_file_distorted_far1.csv');
data_undistorted_init_calib = csvread('map_csv_file_undistorted_init_calib_far1.csv');
data_undistorted_final_calib = csvread('map_csv_file_undistorted_far1.csv');
data_undistorted_tape_calib = csvread('map_csv_file_undistorted_wrong_calib_far1.csv');

%% 1

total_1 = 0;
for i=1:length(data_distorted)
    distorted = data_distorted(i,:);
    diff = (data_distorted - distorted).^2;
    sqrd_diff_sum_col = sqrt(sum(diff, 2));
    sqrd_diff_sum_col_sorted = sort(sqrd_diff_sum_col);
    total_1 = total_1 + sqrd_diff_sum_col_sorted(2);
end
total_1 = total_1/length(data_distorted)

%% 2
total_2 = 0;
for i=1:length(data_undistorted_init_calib)
    distorted = data_undistorted_init_calib(i,:);
    diff = (data_undistorted_init_calib - distorted).^2;
    sqrd_diff_sum_col = sqrt(sum(diff, 2));
    sqrd_diff_sum_col_sorted = sort(sqrd_diff_sum_col);
    total_2 = total_2 + sqrd_diff_sum_col_sorted(2);
end
total_2 = total_2/length(data_undistorted_init_calib)

%% 3
total_3 = 0;
for i=1:length(data_undistorted_final_calib)
    distorted = data_undistorted_final_calib(i,:);
    diff = (data_undistorted_final_calib - distorted).^2;
    sqrd_diff_sum_col = sqrt(sum(diff, 2));
    sqrd_diff_sum_col_sorted = sort(sqrd_diff_sum_col);
    total_3 = total_3 + sqrd_diff_sum_col_sorted(2);
end
total_3 = total_3/length(data_undistorted_final_calib)

%% 4
total_4 = 0;
for i=1:length(data_undistorted_tape_calib)
    distorted = data_undistorted_tape_calib(i,:);
    diff = (data_undistorted_tape_calib - distorted).^2;
    sqrd_diff_sum_col = sqrt(sum(diff, 2));
    sqrd_diff_sum_col_sorted = sort(sqrd_diff_sum_col);
    total_4 = total_4 + sqrd_diff_sum_col_sorted(2);
end
total_4 = total_4/length(data_undistorted_tape_calib)
%%
step_size = 1;
figure(1)
subplot(121)
plot(-data_distorted(1:step_size:end,2), data_distorted(1:step_size:end,1), '.');
axis equal
xlabel('X Axis', 'fontsize', 24, 'fontweight', 'bold');
ylabel('Y Axis', 'fontsize', 24, 'fontweight', 'bold');
grid;
% axis([-6 15 -15 7])
title('Raw Scan Matching', 'fontsize', 24);
subplot(122)
% plot(-data_undistorted_init_calib(1:step_size:end,2), data_undistorted_init_calib(1:step_size:end,1), '.');
% hold on;
plot(-data_undistorted_final_calib(1:step_size:end,2), data_undistorted_final_calib(1:step_size:end,1),'.');
hold off;
axis equal
xlabel('X Axis', 'fontsize', 24, 'fontweight', 'bold');
ylabel('Y Axis', 'fontsize', 24, 'fontweight', 'bold');
grid;
% axis([-6 15 -15 7])
title('Deskewed Scan Matching', 'fontsize', 24);

%%
step_size = 1;
figure(2)
subplot(131)
plot(-data_undistorted_tape_calib(1:step_size:end,2), data_undistorted_tape_calib(1:step_size:end,1), '.');
hold off;
axis equal
xlabel('X Axis', 'fontsize', 24, 'fontweight', 'bold');
ylabel('Y Axis', 'fontsize', 24, 'fontweight', 'bold');
% axis([-6 15 -15 7])
grid;
title('Deskewed Scan Matching with hand calculated calibration');

subplot(132)
plot(-data_undistorted_init_calib(1:step_size:end,2), data_undistorted_init_calib(1:step_size:end,1), '.');
hold off;
axis equal
xlabel('X Axis', 'fontsize', 24, 'fontweight', 'bold');
ylabel('Y Axis', 'fontsize', 24, 'fontweight', 'bold');
% axis([-6 15 -15 7])
grid;
title('Deskewed Scan Matching with initial calibration');

subplot(133)
plot(-data_undistorted_final_calib(1:step_size:end,2), data_undistorted_final_calib(1:step_size:end,1), '.');
hold off;
axis equal
xlabel('X Axis', 'fontsize', 24, 'fontweight', 'bold');
ylabel('Y Axis', 'fontsize', 24, 'fontweight', 'bold');
% axis([-6 15 -15 7])
grid;
title('Deskewed Scan Matching with final calibration');

