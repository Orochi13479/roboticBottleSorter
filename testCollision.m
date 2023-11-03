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
translationVector1 = [0.95, -0.6, 0.5];
rModel.base = rt2tr(armRotationMatrix1, translationVector1);
% rModel.teach()
q1 = deg2rad([40,90,20,45,90,0]);
q2 = deg2rad([-135, 90,20,-30,90,0]);
folderName = 'data';

[cup, TRI, PTS, DATA] = PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);

rModel.animate(rModel.getpos());

input("Enter for Position 1")
rModel.animate(q1);

input("Enter for Position 2")
rModel.animate(q2);

input("Enter to Start Calculation")
rModel.animate(q1);
pause(0.1);

disp("Calculating Traj Around Table Leg")
qMatrix = collisionFreeTraj(rModel, q1, q2, PTS, TRI);

input("Collision Free Traj Calced")

for j = 1:length(qMatrix)
    rModel.animate(qMatrix(j, :));
    drawnow();
    pause(0.1);
end