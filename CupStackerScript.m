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
UR3eGripperL.animate([0, 0.03, 0]);
UR3eGripperR.animate([0, 0.03, 0]);

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
tableHeight = TableDimensions(3);

% Concrete floor
surf([-4.3, -4.3; 4.3, 4.3] ...
    , [-2.2, 2.2; -2.2, 2.2] ...
    , [0.01, 0.01; 0.01, 0.01] ...
    , 'CData', imread(fullfile(folderName, 'concrete.jpg')), 'FaceColor', 'texturemap');

% Place objects in environment
PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [-0.08, -0.05, tableHeight]);
PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [-0.08, 0.07, tableHeight]);
PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.2, -1, 0]);
PlaceObject(fullfile(folderName, 'assembledFence.ply'), [0.25, 0.7, -0.97]);

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
wallx = 0;
wally = 0;

finalCupArrayUR3 = [; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    wallx, wally - 0.1, tableHeight + cupHeight; ...
    ];

finalCupArrayX250 = [; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
    wallx, wally + 0.05, tableHeight + cupHeight; ...
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

UR3eqOpenGripper = [0, 0.03, 0];
UR3eqCloseGripper = [0, 0.05, 0];
UR3ecloseTraj = jtraj(UR3eqOpenGripper, UR3eqCloseGripper, steps/4);
UR3eopenTraj = jtraj(UR3eqCloseGripper, UR3eqOpenGripper, steps/4);

for i = 1:(length(finalCupArrayUR3))
    disp("Running...")
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
        % CollisionCheck(WidowX250, self.cupVertices);
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

%% Collision Checker and Avoidance Function
function CollisionCheck(robot, cupVertices)
% Creating transform for every joint
q = robot.getpos();
tr = zeros(4, 4, robot.n+1);
tr(:, :, 1) = robot.base;
L = robot.links;
for i = 1:robot.n
    tr(:, :, i+1) = tr(:, :, i) * trotz(q(i)+L(i).offset) * transl(0, 0, L(i).d) * transl(L(i).a, 0, 0) * trotx(L(i).alpha);
end

for i = 1:size(tr, 3) - 1
    for vertexIndex = 1:size(cupVertices, 1)
        vertOnPlane = cupVertices(vertexIndex, :);
        [intersectP, check] = LinePlaneIntersection([0, 0, 1], vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP, cupVertices)
            plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
            disp('Collision');
        end
    end
end

% Go through until there are no step sizes larger than 1 degree
q1 = robot.getpos();
q2 = deg2rad([81.5, -12, 50, 51, 90, 0]);

steps = 2;

while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1, q2, steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1, q2, steps);

% Check each of the joint states in the trajectory to work out which ones are in collision.
% Return a logical vector of size steps which contains 0 = no collision (safe)
% and 1 = yes collision (Unsafe).

result = true(steps, 1);
for i = 1:steps
    % fprintf("Step: %d\n", i)
    result(i) = IsCollision(robot, qMatrix(i, :), cupVertices, false);
    % robot.animate(qMatrix(i,:));
    % drawnow();
    pause(0.02);
    if result(i) == true
        disp('UNSAFE: Object detected. Robot stopped')
        break
    end
end
end

%% IsCollision
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot, qMatrix, cupVertices, returnOnceFound)
if nargin < 4
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix, 1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex, :), robot);

    % Go through each link and also each cup vertex
    for i = 1:size(tr, 3) - 1
        for vertexIndex = 1:size(cupVertices, 1)
            vertOnPlane = cupVertices(vertexIndex, :);
            [intersectP, check] = LinePlaneIntersection([0, 0, 1], vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP, cupVertices)
                plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                disp('Collision');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end

%% Function for getting current link poses for Collision Checker
function [transforms] = GetLinkPoses(q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links)+1);
transforms(:, :, 1) = robot.base;

for i = 1:length(links)
    L = links(1, i);

    current_transform = transforms(:, :, i);

    current_transform = current_transform * trotz(q(1, i)+L.offset) * transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
    transforms(:, :, i+1) = current_transform;
end
end

%% FineInterpolation
function qMatrix = FineInterpolation(q1, q2, maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end

steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1, q2, steps))), 1))
    steps = steps + 1;
end
qMatrix = jtraj(q1, q2, steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely interpolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians, maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1:size(waypointRadians, 1) - 1
    qMatrix = [qMatrix; FineInterpolation(waypointRadians(i, :), waypointRadians(i + 1, :), maxStepRadians)]; %#ok<AGROW>
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is
% inside (result == 1) or
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)

u = triangleVerts(2, :) - triangleVerts(1, :);
v = triangleVerts(3, :) - triangleVerts(1, :);

uu = dot(u, u);
uv = dot(u, v);
vv = dot(v, v);

w = intersectP - triangleVerts(1, :);
wu = dot(w, u);
wv = dot(w, v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0) % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0) % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1; % intersectP is in Triangle
end
