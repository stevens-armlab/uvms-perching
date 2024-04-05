function q = invkin_6dof(b, Ra, E_d, Re_d, sequence)

q = [0; pi/100; pi/100; 0; pi/100; 0];

vmax = 1;
vmin = 0;
wmax = 0.01;
wmin = 0;
ep = 0.001;
ew = 0.001;
lambda = 5;

for i=1:1000
    frames = forkin_6dof(q, b, Ra);
    E_c = frames(1:3,4,6);
    Re_c = frames(1:3, 1:3, 6);
    
    Re_e = Re_d*Re_c';
    the = acos((trace(Re_e) - 1)/2);
    me = 1/(2*sin(the)) * [Re_e(3,2) - Re_e(2,3);...
        Re_e(1,3) - Re_e(3,1);...
        Re_e(2,1) - Re_e(1,2)];
    deltap = norm(E_d - E_c);
    deltaw = norm(the*me);
    if deltap<ep && deltaw<ew
       break 
    end
    n = (E_d - E_c)/deltap;

    if deltap/ep > lambda
        vmag = vmax;
    else
        vmag = vmin + (vmax - vmin)*(deltap - ep)/(ep*(lambda - 1));
    end
    
    if deltaw/ew > lambda
        wmag = wmax;
    else
        wmag = wmin + (wmax - wmin)*(deltaw - ew)/(ew*(lambda - 1));
    end
    pdot = vmag*n;
    wdot = wmag*me;

    xdot = [pdot; wdot];
    J = jacobian_6dof(frames, b, Ra, sequence);
    Jinv = J'*inv(J*J' + 1e-6*eye(6));
    qdot = Jinv*xdot;
    q = q + qdot;
end