clear all;
clc;
clf;
hold on;
axis equal;

% Force figure limits
zlim([0, 2]);
xlim([-2, 2]);
ylim([-2, 2]);
disp('Figure Created');

% Robot Initialisations
% Initialise and Plot the UR3e object
UR3eRobot = UR3e;
UR3e = UR3eRobot.model;

% Initialise and Plot the WidowX250 object
X250Robot = WidowX250;
WidowX250 = X250Robot.model;

% Initialise and Plot the WidowX250 Gripper object
X250Gripper = WidowX250Gripper;
WidowX250Gripper = X250Gripper.model;

% Reduce lag
UR3e.delay = 0;
WidowX250.delay = 0;
WidowX250Gripper.delay = 0;

disp('Robots Initialised');

% Manipulate the WidowX250 Base If custom position e.g. On top of a table
% Gather default rotation and translation matrices
[armRotationMatrix1, armTranslationVector1] = tr2rt(WidowX250.base);
[armRotationMatrix2, armTranslationVector2] = tr2rt(UR3e.base);

% Translate along each axis
translationVector1 = [-0.3, 0, 0.5];
translationVector2 = [0.3, 0, 0.5];

% Specify the rotation angle in radians
angle = pi;

% Create a rotation matrix for the Z-axis rotation
Rz = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; ...
    0, 0, 1];

WidowX250.base = rt2tr(armRotationMatrix1, translationVector1);
UR3e.base = rt2tr(armRotationMatrix2, translationVector2);

% Set Base of Gripper to End effector
WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(pi) * transl(0, -0.233, 0);


% Assume starting position
UR3e.animate(UR3e.getpos());
WidowX250.animate(WidowX250.getpos());
WidowX250Gripper.animate(WidowX250Gripper.getpos());

disp('Robots Mounted');
disp('Setup is complete');

%% Environment
folderName = 'data';

% Environment - Table dimensions
TableDimensions = [2.1, 1.4, 0.5]; %[Length, Width, Height]

% Concrete floor
surf([-4.3, -4.3; 4.3, 4.3] ...
    , [-2.2, 2.2; -2.2, 2.2] ...
    , [0.01, 0.01; 0.01, 0.01] ...
    , 'CData', imread(fullfile(folderName, 'concrete.jpg')), 'FaceColor', 'texturemap');

% Place objects in environment
PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
PlaceObject(fullfile(folderName, 'plasticCup.ply'), [0.96, 0.6, TableDimensions(3)]);
PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.2, -1, 0]);
PlaceObject(fullfile(folderName, 'assembledFence.ply'), [0.25, 0.7, -0.97]);

% PlaceObject('emergencyStopButton.ply', [0.96, 0.6, TableDimensions(3)]);

%% Place Movable objects
% Create Bricks and Place Randomly
brickHeight = 0.034;
tableHeight = TableDimensions(3);

initBrickArray = [-0.4, -0.35, tableHeight; ...
    -0.52, -0.325, tableHeight; ...
    -0.7, -0.25, tableHeight; ...
    -0.62, -0.25, tableHeight; ...
    -0.735, -0.1, tableHeight; ...
    -0.815, 0, tableHeight + brickHeight; ...
    -0.815, 0, tableHeight; ...
    -0.8, 0.15, tableHeight; ...
    -0.65, 0.1, tableHeight];
self.initBrickArray = initBrickArray;

% Get brick vertices from ply file
[tri, self.brickVertices] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');

for i = 1:length(initBrickArray)
    % Place the brick using PlaceObject
    self.bricks(i) = PlaceObject(fullfile(folderName, 'plasticCup.ply'), [initBrickArray(i, 1), initBrickArray(i, 2), initBrickArray(i, 3)]);
    % Convert Coords to Transforms
    self.initBrickTr(:, :, i) = transl(initBrickArray(i, 1), initBrickArray(i, 2), initBrickArray(i, 3)+(brickHeight * 3)) * troty(pi);
end

disp('Plastic Cups Created');

%% Begin operation
% SMOOTH SYNCRONOUS MOVEMENT OF BOTH ROBOTS ACHIEVED
steps = 200;
% WidowX250.teach()
% UR3e.teach()
% WidowX250Gripper.teach()
input("Press Enter to See Beauty")
for i = 1:1
    if i == 1
        % Initial Starting Position
        qStart = zeros(1, WidowX250.n);
    else
        % qStart = WidowX250.ikcon(fin)
    end
    init1 = deg2rad([-15, 42.3, -5.9, -6.17, 52.4, -50.3]);
    fin1 = deg2rad([-180, -14.6, 26.9, 2.18, 76.3, -24.4]);
    init2 = deg2rad([0, -20, 40, -110, -90, 0]);
    fin2 = deg2rad([-180, -50, 90, -130, -90, 0]);
    % qInitial = WidowX250.fkine(init)
    % qFinal = WidowX250.fkine(fin)
    pickupTraj1 = jtraj(qStart, init1, steps);
    dropoffTraj1 = jtraj(init1, fin1, steps);
    pickupTraj2 = jtraj(qStart, init2, steps);
    dropoffTraj2 = jtraj(init2, fin2, steps);

    for j = 1:steps
        WidowX250.animate(pickupTraj1(j, :));
        UR3e.animate(pickupTraj2(j, :));
        WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(pi) * transl(0, -0.233, 0);
        WidowX250Gripper.animate(WidowX250Gripper.getpos());
        drawnow();
    end

    for j = 1:steps
        WidowX250.animate(dropoffTraj1(j, :));
        UR3e.animate(dropoffTraj2(j, :));
        WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(pi) * transl(0, -0.233, 0);
        WidowX250Gripper.animate(WidowX250Gripper.getpos());
        drawnow();
    end

end
