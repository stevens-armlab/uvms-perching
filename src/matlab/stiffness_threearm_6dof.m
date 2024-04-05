function K = stiffness_threearm_6dof(auv_rot, q, tau)
num_arms = 3;
num_links = 6;
DOF = num_arms*num_links;

[A, B] = jacobian_threearm_6dof(auv_rot, q);
J = pinv(B)*A;
dA_dq  = zeros(6*num_arms, 6, DOF);
dB_dq  = zeros(6*num_arms, DOF, DOF);
dJ_dqi  = zeros(DOF, 6, DOF);
dJi_dq = zeros(DOF, DOF, 6);

for i=1:DOF
    dA_dq(:, :, i) = dA_dqi(auv_rot, q, i, num_arms, num_links);
    dB_dq(:, :, i) = dB_dqi(auv_rot, q, i, num_arms, num_links);

    dBpinv = -pinv(B)*dB_dq(:, :, i)*pinv(B) + pinv(B)*pinv(B)'*dB_dq(:, :, i)'*(eye(6*num_arms) - B*pinv(B)) + (eye(DOF) - pinv(B)*B)*dB_dq(:, :, i)'*pinv(B)'*pinv(B);
    dJ_dqi(:,:,i) = dBpinv*A + pinv(B)*dA_dq(:,:,i);
end

for i=1:6
    Z = [];
    for j = 1:DOF
        Z = [Z, dJ_dqi(:,i,j)];
    end
    dJi_dq(:,:,i) = Z;
end

T = zeros(6, 6*DOF);
for i=1:6
    T(i, (DOF*(i - 1)+1):(DOF*i)) = tau';
end
dtau_dq = eye(DOF)*10;

first = T*[dJi_dq(:,:,1);...
    dJi_dq(:,:,2);...
    dJi_dq(:,:,3);...
    dJi_dq(:,:,4);...
    dJi_dq(:,:,5);...
    dJi_dq(:,:,6)];
second = J'*dtau_dq;

K = first*J + second*J;
end



function X = dA_dqi(auv_rot, Q, idx, num_arms, num_links)
    arm_base_x = 226.37;
    arm_base_y = 095.53;
    arm_base_z = 113.28;
    ab = zeros(3,1,num_arms);
    ab(:, :, 1) = [arm_base_x; -arm_base_y; -arm_base_z];
    ab(:, :, 2) = [arm_base_x; arm_base_y; -arm_base_z];
    ab(:, :, 3) = [arm_base_x; 0; sqrt(arm_base_y^2 + arm_base_z^2)];
    l1 = [140;0;0];
    l2 = [120;0;0];
    l3 = [140;0;0];
    l4 = [115;0;0];
    l5 = [140;0;0];
    l6 = [95;0;0];
%     l4 = [95; 0; 0];
%     l5 = [0; 0; 0];
%     l6 = [0; 0; 0];
    DOF = num_arms*num_links;

    syms q [DOF 1]
    r_a_e = sym(zeros(3, 1, num_arms));
    A = sym(zeros(6*num_arms, 6));
    for i=1:num_arms
        j = num_links*(i-1);
        r_a_e(:,:,i) = ab(:, :, i) + ...
            rotx(q(j+1))*(l1 + ...
            rotz(q(j+2))*(l2 + ...
            rotz(q(j+3))*(l3 + ...
            rotx(q(j+4))*(l4 + ...
            rotz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6))))));
        A((6*(i-1)+1):(6*i), :) = ...
        [eye(3), -skewsym(auv_rot*r_a_e(:, :, i));...
        zeros(3,3), eye(3)];
    end
    
    Y = diff(A, q(idx));
    X = double(subs(Y, q, Q));
end



function X = dB_dqi(auv_rot, Q, idx, num_arms, num_links)
    l1 = [140;0;0];
    l2 = [120;0;0];
    l3 = [140;0;0];
    l4 = [115;0;0];
    l5 = [140;0;0];
    l6 = [95;0;0];
%     l4 = [95; 0; 0];
%     l5 = [0; 0; 0];
%     l6 = [0; 0; 0];
    DOF = num_arms*num_links;

    syms q [DOF 1]

    M = sym(zeros(3, num_links, num_arms));
    S = sym(zeros(3, num_links, num_arms));
    B = sym(zeros(6*num_arms, DOF));
    for i=1:num_arms
        j = num_links*(i-1);
        m1 = -auv_rot*dRx(q(j+1))*(l1 + ...
            rotz(q(j+2))*(l2 + ...
            rotz(q(j+3))*(l3 + ...
            rotx(q(j+4))*(l4 + ...
            rotz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6))))));
        m2 = -auv_rot*rotx(q(j+1))*dRz(q(j+2))*(l2 + ...
            rotz(q(j+3))*(l3 + ...
            rotx(q(j+4))*(l4 + ...
            rotz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6)))));
        m3 = -auv_rot*rotx(q(j+1))*rotz(q(j+2))*dRz(q(j+3))*(l3 + ...
            rotx(q(j+4))*(l4 + ...
            rotz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6))));
        m4 = -auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*dRx(q(j+4))*(l4 + ...
            rotz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6)));
        m5 = -auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*rotx(q(j+4))*dRz(q(j+5))*(l5 + ...
            rotx(q(j+6))*(l6));
        m6 = -auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*rotx(q(j+4))*rotz(q(j+5))*dRx(q(j+6))*(l6);
        M(:, :, i) = [m1 m2 m3 m4 m5 m6];
    
        s1 = auv_rot*rotx(q(j+1))*[1;0;0];
        s2 = auv_rot*rotx(q(j+1))*rotz(q(j+2))*[0;0;1];
        s3 = auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*[0;0;1];
        s4 = auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*rotx(q(j+4))*[1;0;0];
        s5 = auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*rotx(q(j+4))*rotz(q(j+5))*[0;0;1];
        s6 = auv_rot*rotx(q(j+1))*rotz(q(j+2))*rotz(q(j+3))*rotx(q(j+4))*rotz(q(j+5))*rotx(q(j+6))*[1;0;0];
        S(:, :, i) = [s1 s2 s3 s4 s5 s6];  

        B((6*i-5):(6*i), (num_links*(i-1)+1):(num_links*i)) = ...
            [M(:, :, i); ...
            S(:, :, i)];
    end
    Y = diff(B, q(idx));
    X = double(subs(Y, q, Q));
end



function R = dRx(a)
R = [0, 0, 0;...
    0, -sin(a), -cos(a);...
    0, cos(a), -sin(a)];
end



function R = dRz(a)
R = [-sin(a), -cos(a), 0;...
    cos(a), -sin(a), 0;...
    0, 0, 0];
end