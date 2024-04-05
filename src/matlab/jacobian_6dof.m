function J = jacobian_6dof(frames, b, Ra, sequence)
    % ax = [];
    % for i=1:6
    %     if sequence{1}(i) == 'X'
    %         ch = 1;
    %     elseif sequence{1}(i) == 'Y'
    %         ch = 2;
    %     elseif sequence{1}(i) == 'Z'
    %         ch = 3;
    %     end
    %     ax = [ax, ch];
    % end
    J = zeros(6, 6);
    P = frames(1:3, 4, 6);  
    ax = sequence(1);
    z1 = Ra(1:3, readaxis(ax));
    p1 = b;
    Jp1 = cross(z1, (P - p1));
    Jo1 = z1;
    J(:, 1) = [Jp1; Jo1];
    
    for i=2:6
        % if i == 4 || i == 6
        % switch(i)
        %     case 4
        %         zi = frames(1:3, 1, i);
        %     case 5
        %         zi = frames(1:3, 2, i);
        %     otherwise
        %         zi = frames(1:3, 3, i);
        % end
        ax = sequence(i);
        zi = frames(1:3, readaxis(ax), i);
        
        pi = frames(1:3, 4, i-1);
        Jpi = cross(zi, (P - pi));
        Joi = zi;
        J(:, i) = [Jpi; Joi];
    end
end

function X = readaxis(c)
    if c == 'X'
        X = 1;
    elseif c == 'Y'
        X = 2;
    elseif c == 'Z'
        X = 3;
    end
end