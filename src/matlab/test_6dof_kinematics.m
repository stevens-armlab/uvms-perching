clear all
close all
clc

a = [-800; 0; 0];
% Ra = eye(3);
Ra = eul2rotm([0 -0 0],'ZYX');

arm_base_x = 226.37;
arm_base_y = 095.53;
arm_base_z = 113.28;
ab1 = [arm_base_x; -arm_base_y; -arm_base_z];
ab2 = [arm_base_x; arm_base_y; -arm_base_z];
ab3 = [arm_base_x; 0; arm_base_z];

b1 = a + Ra*ab1;
E1_d = [0; -200; 0];
Re1_d = eye(3);

b2 = a + Ra*ab2;
E2_d = [0; 200; 0];
Re2_d = eye(3);

b3 = a + Ra*ab3;
E3_d = [0; 0; 200];
Re3_d = eye(3);

q1 = invkin_6dof(b1, Ra, E1_d, Re1_d, 'XZZXZX');
frames1 = forkin_6dof(q1, b1, Ra);

q2 = invkin_6dof(b2, Ra, E2_d, Re2_d, 'XZZXZX');
frames2 = forkin_6dof(q2, b2, Ra);

q3 = invkin_6dof(b3, Ra, E3_d, Re3_d, 'XZZXZX');
frames3 = forkin_6dof(q3, b3, Ra);

E1 = frames1(1:3,4,6)
E2 = frames2(1:3,4,6)
E3 = frames3(1:3,4,6)

figure(1)
clf
set(gcf,'Position', get(0, 'ScreenSize'));
view(135, 45)
axis equal
axis([-1100 200 -500 500 -500 500])
draw_auv(a, Ra)
draw_arm_6dof(frames1, b1, 'r')
draw_arm_6dof(frames2, b2, 'g')
draw_arm_6dof(frames3, b3, 'b')

%% Stiffness analysis
q1 = [pi/4; pi/3; -pi/3; -pi/4; 0; 0];
q2 = [-pi/4; -pi/3; pi/3; pi/4; 0; 0];
q3 = [pi/2; pi/3; -pi/3; -pi/2; 0; 0];


[J3, A, B] = jacobian_threearm_6dof(Ra, [q1; q2; q3]);
Kt3 = 3.7282*eye(18);
K3 = J3'*Kt3*J3;

Fx = [1; 0; 0; 0; 0; 0];
Fy = [0; 1; 0; 0; 0; 0];
Fz = [0; 0; 1; 0; 0; 0];

X2 = inv(K2)*Fx
Y2 = inv(K2)*Fy
Z2 = inv(K2)*Fz

X3 = inv(K3)*Fx
Y3 = inv(K3)*Fy
Z3 = inv(K3)*Fz

%% Replot deformed UVMS

dX = X3;

dq = J3*dX;
qnew = [q1; q2; q3] + dq;

th = norm(dX(4:6));
m = dX(4:6)/th;
anew = a + dX(1:3)*1e3;
Rnew = Ra*axang2rotm([m', th]);

b1new = anew + Rnew*ab1;
b2new = anew + Rnew*ab2;
b3new = anew + Rnew*ab3;

frames1new = forkin_6dof(qnew(1:6), b1new, Rnew);
frames2new = forkin_6dof(qnew(7:12), b2new, Rnew);
frames3new = forkin_6dof(qnew(13:18), b3new, Rnew);

figure(2)
clf
set(gcf,'Position', get(0, 'ScreenSize'));
view(135, 45)
axis equal
axis([-1100 200 -500 500 -500 500])

draw_auv(a, Ra)
draw_arm_6dof(frames1, b1, [0.6; 0; 0])
draw_arm_6dof(frames2, b2, [0; 0.6; 0])
draw_arm_6dof(frames3, b3, [0; 0; 0.6])

draw_auv(anew, Rnew)
draw_arm_6dof(frames1new, b1new, 'r')
draw_arm_6dof(frames2new, b2new, 'g')
draw_arm_6dof(frames3new, b3new, 'b')
