% close all;
clear all;
clc;
hold on;
axis equal;

% Force figure limits
zlim([0, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER
r = WidowX250();
rModel = r.model;
[armRotationMatrix1, armTranslationVector1] = tr2rt(rModel.base);
translationVector1 = [0.6, -0.6, 0];
rModel.base = rt2tr(armRotationMatrix1, translationVector1);

q1 = deg2rad([50, 0, 0, 0, 0, 0]);
q2 = deg2rad([-40, 0, 0, 0, 0, 0]);
folderName = 'data';

[cup, TRI, PTS, DATA] = PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0])

qMatrix = collisionFreeTraj(rModel, q1, q2, PTS, TRI)

input("Collision Free Traj Calced")

for j = 1:length(qMatrix)
    rModel.animate(qMatrix(j, :));
    drawnow();
    pause(0.1);
end