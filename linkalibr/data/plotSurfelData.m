clear all;
clc;
data = csvread('surfel_data.csv');

plot3(data(:, 1), data(:, 2), data(:, 3), '.');
axis equal
grid;