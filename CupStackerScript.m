clear all;
clc;
clf;
hold on;
% axis equal;

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
[armRotationMatrix, armTranslationVector] = tr2rt(WidowX250.base);

% Translate along each axis
translationVector = [0.65, 0, 0];

% Specify the rotation angle in radians (90 degrees)
angle = pi / 2;

% Create a rotation matrix for the X-axis rotation
Rx = [1, 0, 0; ...
    0, cos(angle), -sin(angle); ...
    0, sin(angle), cos(angle)];

Rz = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; ...
    0, 0, 1];

% Place Base WidowX250 Up
% Apply the X-axis rotation
desiredBaseMatrix = rt2tr(armRotationMatrix, translationVector);
WidowX250.base = desiredBaseMatrix;

% Set Base of Gripper to End effector
WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T* trotx(pi) * transl(0,-0.236,0);


% Assume starting position
UR3e.animate(UR3e.getpos());
WidowX250.animate(WidowX250.getpos());
WidowX250Gripper.animate(WidowX250Gripper.getpos());

disp('Robots Mounted');
disp('Setup is complete');

%% Begin operation
% SMOOTH SYNCRONOUS MOVEMENT OF BOTH ROBOTS ACHIEVED
steps = 200;
% WidowX250.teach()
% UR3e.teach()
WidowX250Gripper.teach()
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
        WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T* trotx(pi) * transl(0,-0.236,0);
        WidowX250Gripper.animate(WidowX250Gripper.getpos());
        drawnow();
    end

    for j = 1:steps
        WidowX250.animate(dropoffTraj1(j, :));
        UR3e.animate(dropoffTraj2(j, :));
        WidowX250Gripper.base = WidowX250.fkine(WidowX250.getpos()).T* trotx(pi) * transl(0,-0.236,0);
        WidowX250Gripper.animate(WidowX250Gripper.getpos());
        drawnow();
    end

end
