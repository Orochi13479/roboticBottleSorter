close all;
clear all;
clc;

r = WidowX250Gripper;
rModel = r.model;
rModel.teach(zeros(r.model.n));

qPath = jtraj([0, 0.032, 0.032], [0, 0.04, 0.04], 50)
for i = 1:length(qPath)
    rModel.animate(qPath(i, :))
    drawnow();
    pause(0.1)

end