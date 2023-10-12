classdef CupStacker
    properties
        UR3e
        WidowX250
        WidowX250Gripper
    end

    methods
        % Constructor
        function self = CupStacker()
            clc;
            clf;
            hold on;
            % axis equal;

            % Environment Initialisations
            % Initialise and Plot objects 
            self.environment

            % Robot Initialisations
            % Initialise and Plot the UR3e object
            self.UR3e = UR3e;
            UR3 = self.UR3e.model;

            % Initialise the WidowX250 object
            self.WidowX250 = WidowX250;
            X250 = self.WidowX250.model;

            % Initialise the WidowX250Gripper
            self.WidowX250Gripper = WidowX250Gripper;
            Gripper = self.WidowX250Gripper.model;
            % LGripper.base = WidowX250.fkine(WidowX250.getpos).T*trotx(pi/2);
            % RGripper.base = WidowX250.fkine(WidowX250.getpos).T*trotz(pi)*trotx(pi/2);

            % Plot WidowX250 robot
            X250.plot(zeros(1, X250.n), 'workspace', self.UR3e.workspace,'nobase', 'noname', 'noraise', 'noshadow', 'notiles', 'nowrist', 'nojaxes', 'nojoints');

            % Reduce lag
            UR3.delay = 0;
            X250.delay = 0;

            disp('Robots Initialised');

            % Manipulate the WidowX250 Base If custom position e.g. On top of a table
            % Gather default rotation and translation matrices
            [armRotationMatrix, armTranslationVector] = tr2rt(X250.base);

            % Translate along each axis
            translationVector = [0.5, 0, 0];

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
            X250.base = desiredBaseMatrix;

            % Assume starting position
            UR3.animate(UR3.getpos());
            X250.animate(X250.getpos());

            disp('Robots Mounted');
            disp('Setup is complete');

            % Environment Setup
            % environment(self);

            operate(self);
        end


        function environment(self)
            folderName = 'data';

            % Environment - Table dimensions
            TableDimensions = [2.1, 1.4, 0.5]; %[Length, Width, Height]

            % Concrete floor
            surf([-4.3, -4.3; 4.3, 4.3] ...
                , [-2.2, 2.2; -2.2, 2.2] ...
                , [0.01, 0.01; 0.01, 0.01] ...
                , 'CData', imread(fullfile(folderName,'concrete.jpg')), 'FaceColor', 'texturemap');

            % Place objects in environment
            PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
            PlaceObject(fullfile(folderName, 'plasticCup.ply'), [0.96, 0.6, TableDimensions(3)]);
            PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.2, -1, 0]);
            PlaceObject(fullfile(folderName, 'assembledFence.ply'), [0.25, 0.7, -0.97]);

            % PlaceObject('emergencyStopButton.ply', [0.96, 0.6, TableDimensions(3)]);

        end

        function operate(self)
            % Begin operation
            self.UR3e.TestMoveJoints;
            self.WidowX250.TestMoveJoints;
        end

    end
end
