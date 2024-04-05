function [] = draw_arm_6dof(frames, b, colorstring)
p1 = b;
p2 = frames(1:3,4,1);
p3 = frames(1:3,4,2);
p4 = frames(1:3,4,3);
p5 = frames(1:3,4,4);
p6 = frames(1:3,4,5);
p7 = frames(1:3,4,6);

% size = 40;
% draw_coordinate_system(size,R1,P1,'rgb','F')
% draw_coordinate_system(size,R5,P5,'rgb')
xyz = [p1 p2 p3 p4 p5 p6 p7];
hold on
% view(45,45)
plot3(xyz(1,:), xyz(2,:), xyz(3,:),'LineWidth',3,'Color',colorstring)
scatter3(xyz(1,2:6), xyz(2,2:6), xyz(3,2:6),'filled','MarkerEdgeColor',colorstring,'MarkerFaceColor',colorstring)
scatter3(xyz(1,1), xyz(2,1), xyz(3,1),'filled','MarkerEdgeColor','k','MarkerFaceColor','k')
scatter3(xyz(1,7), xyz(2,7), xyz(3,7),'filled','MarkerEdgeColor','k','MarkerFaceColor','k')
end