function ForwardKinematicUR10(q1, q2, q3, q4, q5, q6)

a = [0, -0.6127, -0.5716, 0, 0, 0];
d = [0.128, 0, 0, 0.1639, 0.1157, 0.0922];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

offset = [0, -pi/2, 0, -pi/2, 0, 0];
for i= 1:6
   L(i) = Link([ 0 d(i) a(i) alpha(i) 0 offset(i)], 'standard'); 
end
G =  SerialLink(L)
G.name = 'UR10';
kineMatrix = G.fkine([q1, q2, q3, q4, q5, q6]);
disp(kineMatrix);
end