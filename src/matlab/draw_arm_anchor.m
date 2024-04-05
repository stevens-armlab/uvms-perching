function [] = draw_arm_anchor(q1, q2, q3, q4, anchor_pos, anchor_rot)
l1 = [115;0;0];
l2 = [140;0;0];
l3 = [115;0;0];
l4 = [120;0;0];

P1 = anchor_pos;
R1 = anchor_rot;
P2 = P1 - R1*l4;
R2 = R1*rotx(-q4);
P3 = P2 - R2*l3;
R3 = R2*rotz(-q3);
P4 = P3 - R3*l2;
R4 = R3*rotz(-q2);
P5 = P4 - R4*l1;
R5 = R4*rotx(-q1);

size = 40;
draw_coordinate_system(size,R1,P1,'rgb','F')
% draw_coordinate_system(size,R5,P5,'rgb')
xyz = [P1, P2, P3, P4, P5];
hold on
% view(45,45)
plot3(xyz(1,:), xyz(2,:), xyz(3,:),'LineWidth',3)
scatter3(xyz(1,2:4), xyz(2,2:4), xyz(3,2:4),'filled','MarkerEdgeColor','k','MarkerFaceColor','k')
scatter3(xyz(1,1), xyz(2,1), xyz(3,1),'filled','MarkerEdgeColor','g','MarkerFaceColor','g')
scatter3(xyz(1,5), xyz(2,5), xyz(3,5),'filled','MarkerEdgeColor','r','MarkerFaceColor','r')
end