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
translationVector1 = [0, 0, 0];
rModel.base = rt2tr(armRotationMatrix1, translationVector1);
% rModel.teach();
t = 10; % Total time (s)
deltaT = 0.02; % Control frequency

T1 = rModel.fkine(zeros(1, rModel.n)).T
T2 = rModel.fkine(deg2rad([90,30,0,0,60,0])).T
qMatrix = RMRC(rModel, T1, T2, t, deltaT)

length(qMatrix)
