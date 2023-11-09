clear all;
clc;
clf;
hold on;
axis equal;
set(gcf, 'color', 'g');
% Force figure limits
zlim([0, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER

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

% Manipulate the WidowX250 Base If custom position e.g. On top of a table
% Gather default rotation and translation matrices
[armRotationMatrix1, armTranslationVector1] = tr2rt(WidowX250.base);
[armRotationMatrix2, armTranslationVector2] = tr2rt(UR3e.base);

% Translate along each axis
translationVector1 = [-0.3, -0.6, 0.5];
translationVector2 = [0.3, -0.6, 0.5];

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
PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.5, -1.5, 0]);
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



%% Place Movable objects
% Create Cups and Place Randomly
cupHeight = 0.1;

% X250 has 7 Cups
initCupArrayX250 = [; ...
    -0.1, -0.25, tableHeight; ...
    -0.3, -0.3, tableHeight; ...
    -0.45, -0.3, tableHeight; ...
    -0.45, -0.3, tableHeight + cupHeight; ...
    -0.55, -0.4, tableHeight; ...
    -0.6, -0.5, tableHeight; ...
    -0.6, -0.5, tableHeight + cupHeight; ...
    ];

for i = 1:length(initCupArrayX250)
    % Place the Cup using PlaceObject
    self.cupX250(i) = PlaceObject(fullfile(folderName, 'sodaCan.ply'), [initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3)]);
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
    self.cupUR3(i) = PlaceObject(fullfile(folderName, 'plasticCup.ply'), [initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)]);
end

disp('Setup is complete');

%% Light Curtain Demo

% Importing the 3D model of a soda can
[f, v, data] = plyread(fullfile('data', 'sodaCan.ply'), 'tri');
canVertices = v; % Storing vertices of the soda can

[y1,z1] = meshgrid(-1.5:0.01:1, 0.1:0.01:1.5);  %setting location of meshgrid
x1 = zeros(size(y1)) - 1.2;
lightCurtain1 = surf(x1,y1,z1,'FaceAlpha',0.1,'EdgeColor','none');
hold on;

% Define initial and final positions for the soda can
Initial = [-1.5, 0, 0.5];
Final = [1.2, 0, 0.5];

steps = 30; % Define the number of steps for the demonstration

xCan = -1.5; % Initialise x-coordinate for the soda can
canHandles = []; % Initialise an array to store handles of the soda cans

input("Press Enter to Run Demo"); % Prompt user to start the demo

for j = 1:steps
    % Remove previously created soda cans
    if ~isempty(canHandles)
        delete(canHandles);
        canHandles = [];
    end

    % Create a new soda can and store its handle
    canHandle = PlaceObject(fullfile('data', 'sodaCan.ply'), [xCan, 0, 0.6]);
    canHandles = [canHandles, canHandle];

    pause(0.1); % Pause to display the movement

    xCan = xCan + 0.01; % Increment x-coordinate for the next soda can
    canVertices(:, 1) = canVertices(:, 1) + xCan; % Update vertices along x-axis
    drawnow; % Refresh the display

    % Display a warning when the soda can crosses a certain threshold
    if xCan >= -1.2
        disp("Light Curtain has been activated");
        lightCurtain1 = surf(x1, y1, z1, 'FaceAlpha', 0.1, 'FaceColor', 'red'); % Activate a red light curtain
        set(gcf, 'color', 'r'); % Change figure background color to red
    end
end
