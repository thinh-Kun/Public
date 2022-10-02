function [Xd, dXd] = squareTracjectory(t, lengthOfEdge, height)
a = mod(t, 8);
if a >= 0  && a < 2
    Xd(1) = lengthOfEdge/2;
    Xd(2) = lengthOfEdge/2*a - lengthOfEdge/2;
    Xd(3) = height;
    
    dXd(1) = 0;
    dXd(2) = lengthOfEdge/2;
    dXd(3) = 0;
elseif a >= 2  && a < 4
    Xd(1) = -lengthOfEdge/2*a + 3*edge/2;
    Xd(2) = lengthOfEdge/2;
    Xd(3) = height;
    
    dXd(1) = -lengthOfEdge/2;
    dXd(2) = 0;
    dXd(3) = 0;
elseif a >=4  && a < 6
    Xd(1) = -lengthOfEdge/2;
    Xd(2) = -lengthOfEdge/2*a + 5*lengthOfEdge/2;
    Xd(3) = height;
    
    dXd(1) = 0;
    dXd(2) = -lengthOfEdge/2;
    dXd(3) = 0;
elseif a >= 6  && a < 8
    Xd(1) = lengthOfEdge/2*t - 7*edge/2;
    Xd(2) = -lengthOfEdge/2;
    Xd(3) = height;
    
    dXd(1) = lengthOfEdge/2;
    dXd(2) = 0;
    dXd(3) = 0;
end
end