function [Xd, dXd] = Quydao(t)
Xd = zeros(1, 3);
dXd = zeros(1, 3);
%% Quy dao hinh tron
Xd(1) = 0.8*cos(t);
Xd(2) = 0.8*sin(t);
Xd(3) = 1.2;

dXd(1) = -0.8*sin(t);
dXd(2) = 0.8*cos(t);
dXd(3) = 0;
end