function [Xd, dXd] = circleTracjectory(t, radius, height)
Xd = zeros(1, 3);
dXd = zeros(1, 3);
%% Quy dao hinh tron
Xd(1) = radius*cos(t);
Xd(2) = radius*sin(t);
Xd(3) = height;

dXd(1) = radius*sin(t);
dXd(2) = radius*cos(t);
dXd(3) = 0;
end