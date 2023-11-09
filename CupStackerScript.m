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
translationVector1 = [-0.3, -0.6, 0.5];
translationVector2 = [0.3, -0.6, 0.5];

% Specify the rotation angle in radians
angle = pi;

% Create a rotation matrix for the Z-axis rotation
Rz = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; ...
    0, 0, 1];

WidowX250.base = rt2tr(Rz, translationVector1);
UR3e.base = rt2tr(armRotationMatrix2, translationVector2);

UR3e.animate(deg2rad([-180,-70,80,260,-90,0]));
WidowX250.animate(deg2rad([0,0,0,0,90,0]));

% Set Base of WidowX250 Gripper to End effector
WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);

% Set Base of UR3e Gripper to End effector
UR3eGripperL.base = UR3e.fkine(UR3e.getpos).T * trotx(pi/2);
UR3eGripperR.base = UR3e.fkine(UR3e.getpos).T * trotz(pi) * trotx(pi/2);

% Assume starting position
WidowX250GripperL.animate([0, 0.025]);
WidowX250GripperR.animate([0, 0.025]);
UR3eGripperL.animate([0, 0, 0]);
UR3eGripperR.animate([0, 0, 0]);

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
PlaceObject(fullfile(folderName, 'emergencyStopButton.ply'), [-1.5, -0.2, 0.6]);
PlaceObject(fullfile(folderName, 'fireExtinguisherElevated.ply'), [-1.25, 1.5, 0.45]);
PlaceObject(fullfile(folderName, 'rubbishBin.ply'), [-0.3, -1, tableHeight]);
PlaceObject(fullfile(folderName, 'rubbishBin.ply'), [0.3, -1, tableHeight]);
PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.35, -1.5, 0]);
PlaceObject(fullfile(folderName, 'wheeledTable.ply'), [-0.8, -0.75, 0]);
PlaceObject(fullfile(folderName, 'tableChair.ply'), [-1.6, -0.25, 0]);
PlaceObject(fullfile(folderName, 'wheelieBin.ply'), [1.2, 2, 0]);
PlaceObject(fullfile(folderName, 'cabinet.ply'), [0, 2, 0]);
PlaceObject(fullfile(folderName, 'cabinet.ply'), [-1, 2, 0]);

% Light Curtain Placements
PlaceObject(fullfile(folderName, 'lightCurtain.ply'), [1.2, -1.5, 0.85]);
PlaceObject(fullfile(folderName, 'lightCurtain.ply'), [1.2, 1, 0.85]);
PlaceObject(fullfile(folderName, 'lightCurtain.ply'), [-1.2, -1.5, 0.85]);
PlaceObject(fullfile(folderName, 'lightCurtain.ply'), [-1.2, 1, 0.85]);

[y1,z1] = meshgrid(-1.5:0.01:1, 0.1:0.01:1.5);  %setting location of meshgrid
x1 = zeros(size(y1)) - 1.2;
lightCurtain1 = surf(x1,y1,z1,'FaceAlpha',0.1,'EdgeColor','none');
hold on;

%% Place Movable objects
% Create Cups and Place Randomly
cupHeight = 0.14;
canHeight = 0.1;

% X250 has 7 Cans
initCupArrayX250 = [; ...
    -0.1, -0.35, tableHeight; ...
    -0.3, -0.3, tableHeight; ...
    -0.45, -0.31, tableHeight; ...
    -0.45, -0.31, tableHeight + canHeight; ...
    -0.55, -0.4, tableHeight; ...
    -0.6, -0.5, tableHeight; ...
    -0.4, -0.425, tableHeight; ...
    ];

for i = 1:length(initCupArrayX250)
    % Place the Cup using PlaceObject
    cupX250(i) = PlaceObject(fullfile(folderName, 'sodaCan.ply'), [initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3)]);
    % Convert Coords to Transforms
    initCupTrX250(:, :, i) = transl(initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3) + canHeight) * trotx(pi) * trotz(pi/2);
end

% UR3e has 7 Cups
initCupArrayUR3 = [; ...
    0, -0.4, tableHeight; ...
    0.1, -0.25, tableHeight; ...
    0.2, -0.3, tableHeight; ...
    0.3, -0.2, tableHeight; ...
    0.5, -0.5, tableHeight; ...
    0.5, -0.3, tableHeight; ...
    0.6, -0.5, tableHeight; ...
    ];

for i = 1:length(initCupArrayUR3)
    % Place the Cup using PlaceObject
    cupUR3(i) = PlaceObject(fullfile(folderName, 'plasticCup.ply'), [initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)]);
    % Convert Coords to Transforms
    initCupTrUR3(:, :, i) = transl(initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)+cupHeight) * trotx(pi) * trotz(pi/2);
end


% Get Cup and Can vertices from ply file
[tri, cupVertices] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');
[tri, canVertices] = plyread(fullfile(folderName, 'sodaCan.ply'), 'tri');


% Hardcode Final Cup Locations
bin1x = 0.3;
bin2x = -0.3;
biny = -1.0;
binHeight = 0.15;


finalCupArrayUR3 = [; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    bin1x, biny, tableHeight + binHeight; ...
    ];

finalCupArrayX250 = [; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    bin2x, biny, tableHeight + binHeight; ...
    ];

% Convert Final Coords to Transforms
for i = 1:length(finalCupArrayUR3)
    finalCupTrUR3(:, :, i) = transl(finalCupArrayUR3(i, 1), finalCupArrayUR3(i, 2), finalCupArrayUR3(i, 3)+cupHeight) * trotx(pi) * trotz(pi);
    finalCupTrX250(:, :, i) = transl(finalCupArrayX250(i, 1), finalCupArrayX250(i, 2), finalCupArrayX250(i, 3)+canHeight) * trotx(pi) * trotz(0);
end

pause(1); % Let environment Spawn in

disp('Plastic Cups Created');
disp('Setup is complete');

%% Begin operation
% SMOOTH SYNCRONOUS MOVEMENT OF BOTH ROBOTS ACHIEVED
steps = 150;
% UR3eGripperR.teach()
input("Press Enter to See Demo")

% Gripper Trajectory Constant with all Uses
qOpenGripper = [0, 0.025];
qCloseGripper = [0, 0.035];
closeTraj = jtraj(qOpenGripper, qCloseGripper, steps/4);
openTraj = jtraj(qCloseGripper, qOpenGripper, steps/4);

UR3eqOpenGripper = [0, 0, 0];
UR3eqCloseGripper = deg2rad([30, 20, -50]);
UR3ecloseTraj = jtraj(UR3eqOpenGripper, UR3eqCloseGripper, steps/4);
UR3eopenTraj = jtraj(UR3eqCloseGripper, UR3eqOpenGripper, steps/4);

for i = 1:(length(finalCupArrayUR3))
    disp("Running...")
    if i == 1
        % Initial Starting Position
        trStartX250 = initCupTrX250(:, :, i);
        trStartUR3 = initCupTrUR3(:, :, i);
    else
        trStartX250 = finalCupTrX250(:, :, i-1);
        trStartUR3 = finalCupTrUR3(:, :, i-1);
    end

    trWaypointUR3 = UR3e.fkine(deg2rad([-180,-70,80,260,-90,0])).T;
    trWaypointX250 = WidowX250.fkine(deg2rad([0,0,0,0,90,0])).T;

    trInitialX250 = initCupTrX250(:, :, i);
    trFinalX250 = finalCupTrX250(:, :, i);

    trInitialUR3 = initCupTrUR3(:, :, i);
    trFinalUR3 = finalCupTrUR3(:, :, i);
    
    pickupTrajUR3 = RMRC(UR3e, trWaypointUR3, trInitialUR3, UR3e.getpos());
    pickupTrajX250 = RMRC(WidowX250, trWaypointX250, trInitialX250, WidowX250.getpos());

    for j = 1:steps
        WidowX250.animate(pickupTrajX250(j, :));
        UR3e.animate(pickupTrajUR3(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T * trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T * trotz(pi) * trotx(pi/2);
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

    waypointUR31 = RMRC(UR3e, trInitialUR3, trWaypointUR3, UR3e.getpos());
    waypointX2501= RMRC(WidowX250, trInitialX250, trWaypointX250, WidowX250.getpos());
    
    movePLY(UR3e, cupUR3, cupVertices, i, -cupHeight)
    movePLY(WidowX250, cupX250, canVertices, i, -canHeight)
    drawnow();

    for j = 1:steps
        UR3e.animate(waypointUR31(j, :));
        WidowX250.animate(waypointX2501(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T * trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T * trotz(pi) * trotx(pi/2);
        UR3eGripperR.animate(UR3eGripperR.getpos());

        if mod(j, 2) == 0
            movePLY(UR3e, cupUR3, cupVertices, i, -cupHeight)
            movePLY(WidowX250, cupX250, canVertices, i, -canHeight)
        end
        drawnow();
    end
    
    dropoffTrajUR3 = RMRC(UR3e, trWaypointUR3, trFinalUR3, UR3e.getpos());
    dropoffTrajX250 = RMRC(WidowX250, trWaypointX250, trFinalX250, WidowX250.getpos());

    for j = 1:steps
        WidowX250.animate(dropoffTrajX250(j, :));
        UR3e.animate(dropoffTrajUR3(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T * trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T * trotz(pi) * trotx(pi/2);
        UR3eGripperR.animate(UR3eGripperR.getpos());
        if mod(j, 2) == 0
            movePLY(UR3e, cupUR3, cupVertices, i, -cupHeight)
            movePLY(WidowX250, cupX250, canVertices, i, -canHeight)
        end
        drawnow();
    end

    for j = 1:steps / 4
        WidowX250GripperL.animate(openTraj(j, :));
        WidowX250GripperR.animate(openTraj(j, :));
        UR3eGripperL.animate(UR3eopenTraj(j, :));
        UR3eGripperR.animate(UR3eopenTraj(j, :));
        drawnow();
    end
    
    movePLY(UR3e, cupUR3, cupVertices, i, -cupHeight - 0.14)
    movePLY(WidowX250, cupX250, canVertices, i, -canHeight - 0.1)

    waypointUR32 = RMRC(UR3e, trFinalUR3, trWaypointUR3, UR3e.getpos());
    waypointX2502 = RMRC(WidowX250, trFinalX250, trWaypointX250, WidowX250.getpos());

    for j = 1:steps
        WidowX250.animate(waypointX2502(j, :));
        UR3e.animate(waypointUR32(j, :));
        WidowX250GripperL.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
        WidowX250GripperL.animate(WidowX250GripperL.getpos());
        WidowX250GripperR.base = WidowX250.fkine(WidowX250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
        WidowX250GripperR.animate(WidowX250GripperR.getpos());
        UR3eGripperL.base = UR3e.fkine(UR3e.getpos()).T * trotx(pi/2);
        UR3eGripperL.animate(UR3eGripperL.getpos());
        UR3eGripperR.base = UR3e.fkine(UR3e.getpos()).T * trotz(pi) * trotx(pi/2);
        UR3eGripperR.animate(UR3eGripperR.getpos());
        drawnow();
    end
end

disp("Finished")
