function frames = forkin_6dof(q, b, Ra)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);

    l1 = 115;
    l2 = 140;
    l3 = 115;
    l4 = 100;
    l5 = 115;
    l6 = 120;

    frames = zeros(4, 4, 6);
    frames(4, 4, :) = 1;
    frames(1:3, 1:3, 1) = Ra*rotx(q1);
    frames(1:3, 4, 1) = b + frames(1:3, 1:3, 1)*[l1;0;0];

    frames(1:3, 1:3, 2) = frames(1:3, 1:3, 1)*rotz(q2);
    frames(1:3, 4, 2) = frames(1:3, 4, 1) + frames(1:3, 1:3, 2)*[l2;0;0];

    frames(1:3, 1:3, 3) = frames(1:3, 1:3, 2)*rotz(q3);
    frames(1:3, 4, 3) = frames(1:3, 4, 2) + frames(1:3, 1:3, 3)*[l3;0;0];

    frames(1:3, 1:3, 4) = frames(1:3, 1:3, 3)*rotx(q4);
    frames(1:3, 4, 4) = frames(1:3, 4, 3) + frames(1:3, 1:3, 4)*[l4;0;0];

    frames(1:3, 1:3, 5) = frames(1:3, 1:3, 4)*rotz(q5);
    frames(1:3, 4, 5) = frames(1:3, 4, 4) + frames(1:3, 1:3, 5)*[l5;0;0];

    frames(1:3, 1:3, 6) = frames(1:3, 1:3, 5)*rotx(q6);
    frames(1:3, 4, 6) = frames(1:3, 4, 5) + frames(1:3, 1:3, 6)*[l6;0;0];
end