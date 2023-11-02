% close all;
clear all;
clc;
hold on;
axis equal;

% Force figure limits
zlim([0, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER
r = UR3e();
rModel = r.model;
rModel.teach;


t = 10;             % Total time (s)
deltaT = 0.02;      % Control frequency

T1 = rModel.fkine(zeros(1, rModel.n));         % Initial transform
T2 = transl(0.2,0.2,0.1) * trotx(pi) * trotz(pi/2);          % Arm Up position (trot to rotate the end-effector orient
T3 = SE3(T2)

RMRC(rModel,T1,T3,t,deltaT);

% 
% % Fill each deg2rad function with the 6 joint states add however many
% % necessary the first and last should be the same as they are robot initial
% % position ALSO please use waypoints or the robot will go crazy!!
% targetJointStates = [
%     [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8];
%     deg2rad([117,-35.5,66,-117,-90,22.5]);
%     deg2rad([147,-56,86,-117,-90,22.5]);
%     deg2rad([198,-30,52,-117,-90,22.5]);
%     deg2rad([228,-15,35.5,-106,-90,22.5]);
%     [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8]
% ]  

% folderName = 'data';
% centerpnt = [1.1, 0, 0];
% side = 1.5;
% plotOptions.plotFaces = true;
% 
% [TRI,PTS,DATA,~] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');
% [vertex, faces, faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2, plotOptions)
% % PTS = vertex
% % TRI or DATA.face.vertex_indices = faces
% vertex2 = PTS
% faces2 = TRI
% faces3 = DATA.face.vertex_indices
% faceNormals2 = [DATA.vertex.nx,DATA.vertex.ny,DATA.vertex.nz]
