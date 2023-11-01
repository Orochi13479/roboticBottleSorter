clear all;
clc;
clf;
hold on;
axis equal;

% Force figure limits
zlim([0, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER
disp('Figure Created');

% Robot Initialisations
% Initialise and Plot the UR3e object
UR3eRobot = UR3e;
UR3e = UR3eRobot.model;

% Initialise and Plot the WidowX250 object
X250Robot = WidowX250;
WidowX250 = X250Robot.model;

% Initialise and Plot the WidowX250 Gripper object
X250GripperL = WidowX250Gripper;
WidowX250GripperL = X250GripperL.model;
X250GripperR = WidowX250Gripper;
WidowX250GripperR = X250GripperR.model;

%Initialise and Plot the UR3e Gripper object
URGripperL = UR3eGripper;
UR3eGripperL = URGripperL.model;
URGripperR = UR3eGripper;
UR3eGripperR = URGripperR.model;

% Reduce lag
UR3e.delay = 0;
WidowX250.delay = 0;
WidowX250GripperL.delay = 0;
WidowX250GripperR.delay = 0;
UR3eGripperL.delay = 0;
UR3eGripperR.delay = 0;


disp('Robots Initialised');

% Manipulate the WidowX250 Base If custom position e.g. On top of a table
% Gather default rotation and translation matrices
[armRotationMatrix1, armTranslationVector1] = tr2rt(WidowX250.base);
[armRotationMatrix2, armTranslationVector2] = tr2rt(UR3e.base);

% Translate along each axis
% translationVector1 = [-0.3, 0, 0.5];
% translationVector2 = [0.3, 0, 0.5];

translationVector1 = [-0.3, -0.6, 0.5];
translationVector2 = [0.3, -0.6, 0.5];

% Specify the rotation angle in radians
angle = pi;

% Create a rotation matrix for the Z-axis rotation
Rz = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; ...
    0, 0, 1];

WidowX250.base = rt2tr(armRotationMatrix1, translationVector1);
UR3e.base = rt2tr(armRotationMatrix2, translationVector2);

% Set Base of WidowX250 Gripper to End effector
WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);

% Set Base of UR3e Gripper to End effector
UR3eGripperL.base = UR3e.fkine(UR3e.getpos).T*trotx(pi/2);
UR3eGripperR.base = UR3e.fkine(UR3e.getpos).T*trotz(pi)*trotx(pi/2);

% Assume starting position
UR3e.animate(UR3e.getpos());
WidowX250.animate(WidowX250.getpos());
WidowX250GripperL.animate([0, 0.03]);
WidowX250GripperR.animate([0, 0.03]);
UR3eGripperL.animate([0, 0, 0]);
UR3eGripperR.animate([0, 0, 0]);

% qMatrix = [];

q1 = [-pi / 4, 0, 0];
q2 = [pi / 4, 0, 0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1, q2, steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1, q2, steps);


disp('Robots Mounted');

%% Environment
folderName = 'data';

% Environment - Table dimensions
TableDimensions = [2.1, 1.4, 0.5]; %[Length, Width, Height]
wheeledTableDimensions = [0.75, 1.2, 0.52]; %[Length, Width, Height]
tableHeight = TableDimensions(3);

% Concrete floor
surf([-4.3, -4.3; 4.3, 4.3] ...
    , [-2.2, 2.2; -2.2, 2.2] ...
    , [0.01, 0.01; 0.01, 0.01] ...
    , 'CData', imread(fullfile(folderName, 'woodenFloor.jpg')), 'FaceColor', 'texturemap');

% Place objects in environment
PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [-0.4, -1, tableHeight]);
PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [0.2, -1, tableHeight]);
PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.2, -1.5, 0]);
% PlaceObject(fullfile(folderName, 'assembledFence.ply'), [0.25, 0.7, -0.97]);
PlaceObject(fullfile(folderName, 'wheeledTable.ply'), [-0.8, -0.75, 0]);
PlaceObject(fullfile(folderName, 'tableChair.ply'), [-1.6, -0.25, 0]);
PlaceObject(fullfile(folderName, 'wheelieBin.ply'), [1.2, 2, 0]);
PlaceObject(fullfile(folderName, 'cabinet.ply'), [0, 2, 0]);
PlaceObject(fullfile(folderName, 'cabinet.ply'), [-1, 2, 0]);

% PlaceObject('emergencyStopButton.ply', [0.96, 0.6, TableDimensions(3)]);

%% Place Movable objects
% Create Cups and Place Randomly
cupHeight = 0.1;

% 14 Cups to Start with
% X250 has 7 Cups

initCupArrayX250 = [; ...
    -0.299, -0.3, tableHeight; ...
    -0.55, -0.2, tableHeight; ...
    -0.31, 0.3, tableHeight; ...
    -0.31, 0.3, tableHeight + cupHeight; ...
    -0.5, 0.2, tableHeight; ...
    -0.5, 0.2, tableHeight + cupHeight; ...
    -0.4, 0.15, tableHeight; ...
    ];

for i = 1:length(initCupArrayX250)
    % Place the Cup using PlaceObject
    self.cupX250(i) = PlaceObject(fullfile(folderName, 'sodaCan.ply'), [initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3)]);
    % Convert Coords to Transforms
    self.initCupTrX250(:, :, i) = transl(initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3)+cupHeight) * troty(pi);
end

% UR3e has 7 Cups
initCupArrayUR3 = [; ...
    0.1, 0.2, tableHeight; ...
    0.3, -0.35, tableHeight; ...
    0.42, -0.325, tableHeight; ...
    0.4, -0.25, tableHeight; ...
    0.535, -0.1, tableHeight; ...
    0.225, -0.25, tableHeight; ...
    0.2, -0.2, tableHeight; ...
    ];

for i = 1:length(initCupArrayUR3)
    % Place the Cup using PlaceObject
    self.cupUR3(i) = PlaceObject(fullfile(folderName, 'plasticCup.ply'), [initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)]);
    % Convert Coords to Transforms
    self.initCupTrUR3(:, :, i) = transl(initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)+cupHeight) * troty(pi);
end


% Get Cup vertices from ply file
[tri, self.cupVertices] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');

% Hardcode Final Cup Locations
bin1x = 0.3;
bin2x = -0.3;
biny = -0.7;

finalCupArrayUR3 = [; ...
    bin1x, biny, tableHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    bin1x, biny, tableHeight + cupHeight; ...
    ];

finalCupArrayX250 = [; ...
    bin2x, biny, tableHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    bin2x, biny, tableHeight + cupHeight; ...
    ];

% Convert Final Coords to Transforms
for i = 1:length(finalCupArrayUR3)
    self.finalCupTrUR3(:, :, i) = transl(finalCupArrayUR3(i, 1), finalCupArrayUR3(i, 2), finalCupArrayUR3(i, 3)+cupHeight) * trotx(pi) * trotz(pi/2);
    self.finalCupTrX250(:, :, i) = transl(finalCupArrayX250(i, 1), finalCupArrayX250(i, 2), finalCupArrayX250(i, 3)+cupHeight) * trotx(pi) * trotz(pi/2);
end

disp('Plastic Cups Created');
disp('Setup is complete');

%% Begin operation
% SMOOTH SYNCRONOUS MOVEMENT OF BOTH ROBOTS ACHIEVED
steps = 200;
% WidowX250.teach()
% UR3e.teach()
% WidowX250GripperL.teach()
% WidowX250GripperR.teach()
input("Press Enter to See Beauty")

% TEMPORARY FOR DEMO VIDEO
% Calculate the desired end effector position and orientation
desiredPositionPickupX250 = [0, -0.1, tableHeight + cupHeight];
desiredPositionPickupUR3 = [0.1, 0.2, tableHeight + (3 * 0.03)];
desiredPositionDropoffX250 = [0, -0.1, tableHeight + (3 * 0.03)];
desiredPositionDropoffUR3 = [0, 0, tableHeight + (3 * 0.03)];
desiredOrientation = rotx(pi);

% Combine the desired position and orientation to form the transformation matrix
optimalEndEffectorPickupX250 = rt2tr(desiredOrientation, desiredPositionPickupX250);
optimalEndEffectorPickupUR3 = rt2tr(desiredOrientation, desiredPositionPickupUR3);

optimalEndEffectorDropoffX250 = rt2tr(desiredOrientation, desiredPositionDropoffX250);
optimalEndEffectorDropoffUR3 = rt2tr(desiredOrientation, desiredPositionDropoffUR3);

% Calculate the inverse kinematics solution for the desired end effector pose
qCommonPickupX250 = WidowX250.ikcon(optimalEndEffectorPickupX250);
qCommonPickupUR3 = UR3e.ikcon(optimalEndEffectorPickupUR3);

qCommonDropoffX250 = WidowX250.ikcon(optimalEndEffectorDropoffX250);
qCommonDropoffUR3 = UR3e.ikcon(optimalEndEffectorDropoffUR3);

% Gripper Trajectory Constant with all Uses
qOpenGripper = [0, 0.03];
qCloseGripper = [0, 0.05];
closeTraj = jtraj(qOpenGripper, qCloseGripper, steps/4);
openTraj = jtraj(qCloseGripper, qOpenGripper, steps/4);

UR3eqOpenGripper = [0, 0, 0];
UR3eqCloseGripper = deg2rad([30, 22.5, -52.5]);
UR3ecloseTraj = jtraj(UR3eqOpenGripper, UR3eqCloseGripper, steps/4);
UR3eopenTraj = jtraj(UR3eqCloseGripper, UR3eqOpenGripper, steps/4);

for i = 1:(length(finalCupArrayUR3))
    disp("Running...")
    % CollisionCheck(WidowX250, self.cupVertices);
    if i == 1
        % Initial Starting Position
        qStartX250 = zeros(1, WidowX250.n);
        qStartUR3 = zeros(1, UR3e.n);
    else
        qStartX250 = WidowX250.ikcon(self.finalCupTrX250(:, :, i-1), qCommonDropoffX250);
        qStartUR3 = UR3e.ikcon(self.finalCupTrUR3(:, :, i-1), qCommonDropoffUR3);
    end

    qInitialX250 = WidowX250.ikcon(self.initCupTrX250(:, :, i), qCommonPickupX250);
    qFinalX250 = WidowX250.ikcon(self.finalCupTrX250(:, :, i), qCommonDropoffX250);
    pickupTrajX250 = jtraj(qStartX250, qInitialX250, steps);
    dropoffTrajX250 = jtraj(qInitialX250, qFinalX250, steps);

    qInitialUR3 = UR3e.ikcon(self.initCupTrUR3(:, :, i), qCommonPickupUR3);
    qFinalUR3 = UR3e.ikcon(self.finalCupTrUR3(:, :, i), qCommonDropoffUR3);
    pickupTrajUR3 = jtraj(qStartUR3, qInitialUR3, steps);
    dropoffTrajUR3 = jtraj(qInitialUR3, qFinalUR3, steps);

    for j = 1:steps
        WidowX250.animate(pickupTrajX250(j, :));
        UR3e.animate(pickupTrajUR3(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T*trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T*trotz(pi)*trotx(pi/2);
        UR3eGripperR.animate(UR3eGripperR.getpos());
        drawnow();
    end

    

    for j = 1:steps / 4
        WidowX250GripperL.animate(closeTraj(j, :));
        WidowX250GripperR.animate(closeTraj(j, :));
        UR3eGripperL.animate(UR3ecloseTraj(j, :));
        UR3eGripperR.animate(UR3ecloseTraj(j, :));
        drawnow();
    end

    for j = 1:steps
        WidowX250.animate(dropoffTrajX250(j, :));
        UR3e.animate(dropoffTrajUR3(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T*trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T*trotz(pi)*trotx(pi/2);
        UR3eGripperR.animate(UR3eGripperR.getpos());
        drawnow();
    end

    for j = 1:steps / 4
        WidowX250GripperL.animate(openTraj(j, :));
        WidowX250GripperR.animate(openTraj(j, :));
        UR3eGripperL.animate(UR3eopenTraj(j, :));
        UR3eGripperR.animate(UR3eopenTraj(j, :));
        drawnow();
    end
end

disp("Finished")

