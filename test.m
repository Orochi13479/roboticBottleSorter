close all;
clear all;
clc;

r = UR3eGripper;
rModel = r.model;
rModel.teach;

% qPath = jtraj([0, 0.03], [0, 0.074], 200)
% for i = 1:length(qPath)
%     rModel.animate(qPath(i, :))
%     drawnow();
%     pause(0)
% 
% end