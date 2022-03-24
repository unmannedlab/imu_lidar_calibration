clear all
close all
clc;
data = csvread('map.csv');
n=10;
plot3(data(1:n:end,1), data(1:n:end,2), data(1:n:end,3), '.');
grid;
axis equal;
xlim([-20, 20])
ylim([-15, 15])
zlim([-5, 5])
xlabel('X Axis');
ylabel('Y Axis');