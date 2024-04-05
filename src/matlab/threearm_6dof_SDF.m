%% Computing SDF link poses/inertias as function of joint variables

clear all
close all
clc



% Edit Configuration Name:
configname = 'q8';

% Edit joint variables:
q1 = [-0.784526741	-0.015576509	1.673183898	8.21792E-05	-1.657418641	0.784438751]';
q2 = [-2.602271968	0.585262507	0.965518568	0.000217586	-1.551505268	2.602595743]';
q3 = [-1.138353797	-2.298165419	2.229209618	-3.142293925	-0.068192503	4.280598441]';





auv_length = 457.20;
auv_width  = 338.05;
auv_height = 397.85;

a = [-auv_length/2; 0; 0];
Ra = eye(3);

arm_base_x = auv_length/2;
arm_base_y = 100;
arm_base_z = 100;

ab1 = [arm_base_x; arm_base_y; -arm_base_z];
ab2 = [arm_base_x; -arm_base_y; -arm_base_z];
ab3 = [arm_base_x; 0; sqrt(arm_base_y^2 + arm_base_z^2)];

b1 = a + Ra*ab1;
b2 = a + Ra*ab2;
b3 = a + Ra*ab3;

frames1 = forkin_6dof(q1, b1, Ra);
frames2 = forkin_6dof(q2, b2, Ra);
frames3 = forkin_6dof(q3, b3, Ra);

link1_length = 115;


% arm 1 link midpoint positions (3 x 1)
a1_p1 = (b1 + Ra*[link1_length/2;0;0])/1000;
a1_p2 = (frames1(1:3,4,1) + (frames1(1:3,4,2) - frames1(1:3,4,1))*0.5)/1000;
a1_p3 = (frames1(1:3,4,2) + (frames1(1:3,4,3) - frames1(1:3,4,2))*0.5)/1000;
a1_p4 = (frames1(1:3,4,3) + (frames1(1:3,4,4) - frames1(1:3,4,3))*0.5)/1000;
a1_p5 = (frames1(1:3,4,4) + (frames1(1:3,4,5) - frames1(1:3,4,4))*0.5)/1000;
a1_p6 = (frames1(1:3,4,5) + (frames1(1:3,4,6) - frames1(1:3,4,5))*0.5)/1000;

% arm 1 link orientation vectors (3 x 1)
a1_r1 = flip(rotm2eul(frames1(1:3,1:3,1),'ZYX'));
a1_r2 = flip(rotm2eul(frames1(1:3,1:3,2),'ZYX'));
a1_r3 = flip(rotm2eul(frames1(1:3,1:3,3),'ZYX'));
a1_r4 = flip(rotm2eul(frames1(1:3,1:3,4),'ZYX'));
a1_r5 = flip(rotm2eul(frames1(1:3,1:3,5),'ZYX'));
a1_r6 = flip(rotm2eul(frames1(1:3,1:3,6),'ZYX'));

% arm 1 link poses (6 x 1)
a1_pose1 = [a1_p1', a1_r1];
a1_pose2 = [a1_p2', a1_r2];
a1_pose3 = [a1_p3', a1_r3];
a1_pose4 = [a1_p4', a1_r4];
a1_pose5 = [a1_p5', a1_r5];
a1_pose6 = [a1_p6', a1_r6];


% arm 2 link midpoint positions (3 x 1)
a2_p1 = (b2 + Ra*[link1_length/2;0;0])/1000;
a2_p2 = (frames2(1:3,4,1) + (frames2(1:3,4,2) - frames2(1:3,4,1))*0.5)/1000;
a2_p3 = (frames2(1:3,4,2) + (frames2(1:3,4,3) - frames2(1:3,4,2))*0.5)/1000;
a2_p4 = (frames2(1:3,4,3) + (frames2(1:3,4,4) - frames2(1:3,4,3))*0.5)/1000;
a2_p5 = (frames2(1:3,4,4) + (frames2(1:3,4,5) - frames2(1:3,4,4))*0.5)/1000;
a2_p6 = (frames2(1:3,4,5) + (frames2(1:3,4,6) - frames2(1:3,4,5))*0.5)/1000;

% arm 2 link orientation vectors (3 x 1)
a2_r1 = flip(rotm2eul(frames2(1:3,1:3,1),'ZYX'));
a2_r2 = flip(rotm2eul(frames2(1:3,1:3,2),'ZYX'));
a2_r3 = flip(rotm2eul(frames2(1:3,1:3,3),'ZYX'));
a2_r4 = flip(rotm2eul(frames2(1:3,1:3,4),'ZYX'));
a2_r5 = flip(rotm2eul(frames2(1:3,1:3,5),'ZYX'));
a2_r6 = flip(rotm2eul(frames2(1:3,1:3,6),'ZYX'));

% arm 2 link poses (6 x 1)
a2_pose1 = [a2_p1', a2_r1];
a2_pose2 = [a2_p2', a2_r2];
a2_pose3 = [a2_p3', a2_r3];
a2_pose4 = [a2_p4', a2_r4];
a2_pose5 = [a2_p5', a2_r5];
a2_pose6 = [a2_p6', a2_r6];


% arm 3 link midpoint positions (3 x 1)
a3_p1 = (b3 + Ra*[link1_length/2;0;0])/1000;
a3_p2 = (frames3(1:3,4,1) + (frames3(1:3,4,2) - frames3(1:3,4,1))*0.5)/1000;
a3_p3 = (frames3(1:3,4,2) + (frames3(1:3,4,3) - frames3(1:3,4,2))*0.5)/1000;
a3_p4 = (frames3(1:3,4,3) + (frames3(1:3,4,4) - frames3(1:3,4,3))*0.5)/1000;
a3_p5 = (frames3(1:3,4,4) + (frames3(1:3,4,5) - frames3(1:3,4,4))*0.5)/1000;
a3_p6 = (frames3(1:3,4,5) + (frames3(1:3,4,6) - frames3(1:3,4,5))*0.5)/1000;

% arm 3 link orientation vectors (3 x 1)
a3_r1 = flip(rotm2eul(frames3(1:3,1:3,1),'ZYX'));
a3_r2 = flip(rotm2eul(frames3(1:3,1:3,2),'ZYX'));
a3_r3 = flip(rotm2eul(frames3(1:3,1:3,3),'ZYX'));
a3_r4 = flip(rotm2eul(frames3(1:3,1:3,4),'ZYX'));
a3_r5 = flip(rotm2eul(frames3(1:3,1:3,5),'ZYX'));
a3_r6 = flip(rotm2eul(frames3(1:3,1:3,6),'ZYX'));

% arm 3 link poses (6 x 1)
a3_pose1 = [a3_p1', a3_r1];
a3_pose2 = [a3_p2', a3_r2];
a3_pose3 = [a3_p3', a3_r3];
a3_pose4 = [a3_p4', a3_r4];
a3_pose5 = [a3_p5', a3_r5];
a3_pose6 = [a3_p6', a3_r6];


a1_pose = [a1_pose1; a1_pose2; a1_pose3; a1_pose4; a1_pose5; a1_pose6];
a1_posestring = [];
a2_pose = [a2_pose1; a2_pose2; a2_pose3; a2_pose4; a2_pose5; a2_pose6];
a2_posestring = [];
a3_pose = [a3_pose1; a3_pose2; a3_pose3; a3_pose4; a3_pose5; a3_pose6];
a3_posestring = [];

for i=1:6
    s1 = "";
    s2 = "";
    s3 = "";
    for j=1:6
        s1 = s1 + compose("%.8f",a1_pose(i,j));
        s1 = s1 + " ";
        s2 = s2 + compose("%.8f",string(a2_pose(i,j)));
        s2 = s2 + " ";
        s3 = s3 + compose("%.8f",string(a3_pose(i,j)));
        s3 = s3 + " ";
    end
    a1_posestring = [a1_posestring; "<pose>" + s1 + "</pose>"];
    a2_posestring = [a2_posestring; "<pose>" + s2 + "</pose>"];
    a3_posestring = [a3_posestring; "<pose>" + s3 + "</pose>"];
end

wLink = 0.02;
linkDim = [auv_length/1e3, auv_width/1e3, auv_height/1e3;...
    0.115, wLink, wLink;...
    0.140, wLink, wLink;...
    0.115, wLink, wLink;...
    0.100, wLink, wLink;...
    0.115, wLink, wLink;...
    0.120, wLink, wLink];
rho = 1.25*(1e2)^3/1e3;
mLinks = [];
ILinks = [];
for i = 1:7
    if i == 1
        mLinks = [mLinks; 1];
    else
        mLinks = [mLinks; rho*prod(linkDim(i,:))];
    end
    Ixx = 1/12*mLinks(i)*(linkDim(i,2)^2 + linkDim(i,3)^2);
    Iyy = 1/12*mLinks(i)*(linkDim(i,1)^2 + linkDim(i,3)^2);
    Izz = 1/12*mLinks(i)*(linkDim(i,1)^2 + linkDim(i,2)^2);
    ILinks = [ILinks; Ixx, Iyy, Izz];
end

%% Creating SDF file based on link poses
filepathname = pwd;
idx = strfind(filepathname,'/');
pathname = strcat(filepathname(1:idx(end)),'sdf/');
templatefilename = strcat(pathname,'perch_3arm_template.sdf');
outputfilename = strcat(pathname,'perch_3arm_',configname,'.sdf');

fid = fopen(templatefilename,'r');
C=textscan(fid,'%q','delimiter','\n');
Cnew = C;
for i=1:6
    Cnew{1}{38+31*(i-1)} = a1_posestring(i);
    Cnew{1}{224+31*(i-1)} = a2_posestring(i);
    Cnew{1}{410+31*(i-1)} = a3_posestring(i);
end

fidt = fopen(outputfilename,'w');
for i=1:length(Cnew{1,1})
    fprintf(fidt,Cnew{1,1}{i});
    fprintf(fidt,'\n');
end
fclose('all');

