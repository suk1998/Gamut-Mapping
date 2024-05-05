clear; clc; close all

% Define your independent variable
t=0: 2*pi/360 : 2*pi;

% Define values along your x-axis
x=exp(t);

% Define values along your y-axis
y= 50 + exp(1*t);

% Plot your function with a wider line and grid the figure
loglog(x, y, 'LineWidth',2)
grid on

% Use a title for the figure
title('Demonstration of logarithmic plots')

% Label your x-axis with a double line.
% Note the special characters

xlabel([{'e^{t}'}; {'0\leg 2\pi'}])

% Label your y-axis
ylabel('50 + e^(3t)')
