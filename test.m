close all;
clear all;
clc;

r = WidowX250Gripper;
rModel = r.model;
rModel.teach(zeros(r.model.n));

qPath = jtraj([0, 0.03], [0, 0.074], 200)
for i = 1:length(qPath)
    rModel.animate(qPath(i, :))
    drawnow();
    pause(0)

end