% close all;
clear all;
clc;
clf;
hold on;
axis equal;

% Force figure limits
zlim([0, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER
r = WidowX250();
rModel = r.model;
[armRotationMatrix1, armTranslationVector1] = tr2rt(rModel.base);
translationVector1 = [0, 0, 0];
rModel.base = rt2tr(armRotationMatrix1, translationVector1);

T1 = rModel.fkine(deg2rad([0, 0, 0, 0, 0, 0])).T;
% T2 = rModel.fkine(deg2rad([90, -45, 10, -60, -90, 0])).T; % UR3e
T2 = rModel.fkine(deg2rad([60, 0, 0, 0, 90, 0])).T; % WidowX250

qMatrix = RMRC(rModel, T1, T2);

for j = 1:length(qMatrix)
    rModel.animate(qMatrix(j, :));
    drawnow();
    pause(0.01);
end

% rModel.teach();
