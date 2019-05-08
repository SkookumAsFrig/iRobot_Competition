clc
clear all
close all
%	1 - Intersection (P1 & P2)
%	2 - Union (P1 | P2)
%	3 - Difference (P1 & ~P2)

% theta = linspace(0, 2*pi, 100);
% x1 = cos(theta) - 0.5;
% y1 = -sin(theta);    % -sin(theta) to make a clockwise contour
% x2 = x1 + 0.5;
% y2 = y1;

x1 = [0 0 1 1];
y1 = [0 1 1 0];
% x2 = [0 0 1.5 1.5];
% y2 = [0 0.75 0.75 0];
x2 = [-1 -1 1.5 1.5];
y2 = [0.25 0.75 0.75 0.25];
% x2 = x1+0.5;
% y2 = y1+0.5;

figure
plot(x1, y1, x2, y2, 'Color', 'k')
[xa, ya] = polybool(x1, y1, x2, y2, 2);
[xb, yb] = polybool(x1, y1, x2, y2, 1);
[xd, yd] = polybool(x2, y2, x1, y1, 3);
figure
subplot(2, 2, 1)
patch(xa, ya, 1, 'FaceColor', 'r')
axis equal, axis off, hold on
plot(x1, y1, x2, y2, 'Color', 'g')
title('Union')

subplot(2, 2, 3)
patch(xb, yb, 1, 'FaceColor', 'r')
axis equal, axis off, hold on
plot(x1, y1, x2, y2, 'Color', 'k')
title('Intersection')

subplot(2, 2, 4)
patch(xd, yd, 1, 'FaceColor', 'r')
axis equal, axis off, hold on
plot(x1, y1, x2, y2, 'Color', 'k')
title('Subtraction')

gg2 = [xa ya]