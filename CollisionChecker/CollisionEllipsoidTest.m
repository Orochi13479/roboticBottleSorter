clc;
clf;
clear;
close all;

%% Robot
R = UR3e;
R.PlotAndColourRobot;

% UR3
centerPoints = [0.0, 0.0, 0.05; % Base 
                0.0, -0.01, 0.01; % Link 1 
                0.125, 0.0, 0.125; % Link 2 
                0.105, 0.0, 0.05; % Link 3 
                0.0, 0.0, 0.01; % Link 4 
                0.0, 0.0, 0.06; % Link 5 
                0.0, 0.0, 0.0;]; % end-effector

radii = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;]; 
     
meshPosition = [0.05, 0, 0.1];

% Create a grid of points manually within the specified region
xPoints = linspace(meshPosition(1) - 0.1, meshPosition(1) + 0.1, 50);
yPoints = linspace(meshPosition(2) - 0.1, meshPosition(2) + 0.1, 50);
zPoints = linspace(meshPosition(3) - 0.1, meshPosition(3) + 0.1, 50);

[xGrid, yGrid, zGrid] = meshgrid(xPoints, yPoints, zPoints);
meshPoints = [xGrid(:), yGrid(:), zGrid(:)];

% Initialize the CollisionCheck object
collision = CollisionCheck(R, centerPoints, radii);

% Plot ellipsoids
collision.plotEllipsoids();

% Perform collision checks with the manually created grid of points
collision.checkCollision(meshPoints);
