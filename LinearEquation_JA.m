% Matlab program to calculate the linear equation of a line y = mx + c when given 
% two points.
%
% The method solves a system of simultanious equations to find the unknowns
% m and c.
%
% Equations are solved using A*x = B were x = A^-1 * B
%
% A graph of the calculated equation is plotted against a defined range
% of x values
%
% 8/11/13
%
% James Adams

function LinearEquation_JA
clear
clc

dx = 0.01;       % Sets spacing of x values for graph.
n = 18;         % Set no. x values for graph 

x1 = 0.3127;   % Input points
y1 = 0.329;
x2 = 0.3;
y2 = 0.6;

point1 = [x1 y1]       % Displays points as coordinates in command window 
point2 = [x2 y2]      

MatrixX = [x1 1; x2 1];    % Sets up matrices for calculation.
MatrixY = [y1; y2];

solutions = inv(MatrixX)*MatrixY;   % Calculates solutions using A^-1 * B

m = solutions(1,1);            % Extracts m and c values for solution matrix
c = solutions (2,1);

fprintf('\n\ny = %2.6fx + %2.6f \n\n', m, c)   % Displays equation.

xplot = 0: dx : n;

plot(xplot, m*xplot + c , 'b')
title('Graph of calculated equation','Fontsize',14)
xlabel('x','Fontsize', 16)
ylabel('y', 'Fontsize', 16)


end

% g
% y=-21.338583*x + 7.001575