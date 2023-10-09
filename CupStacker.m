classdef CupStacker
    properties
        UR3e
        WidowX250
    end
    methods
        % Constructor
        function self = CupStacker()
            clc;
            clf;
            % hold on;
            % axis equal;

            % Robot Initialisations
            robotInit(self);

            % Environment Setup
            % environment(self);


        end

        function robotInit(self)
            % Initialise and Plot the UR3 object
            % self.UR3robot = UR3;
            self.UR3e = UR3e;
            UR3 = self.UR3e.model;

            % Initialise the WidowX250 object
            self.WidowX250 = WidowX250;
            X250 = self.WidowX250.model;

            % Plot WidowX250 robot
            X250.plot(zeros(1, X250.n), 'workspace', self.UR3e.workspace, 'nobase', 'noname', 'noraise', 'noshadow', 'notiles', 'nowrist');

            % Reduce lag
            UR3.delay = 0;
            X250.delay = 0;

            disp('Robots Initialised');

            % Manipulate the WidowX250 Base If custom position e.g. On top of a table
            % Gather default rotation and translation matrices
            [armRotationMatrix, armTranslationVector] = tr2rt(X250.base);

            % Translate along each axis
            translationVector = [0, 0, 0];

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
        end

        function environment(self)
            % Environment - Table dimensions
            TableDimensions = [2.1, 1.4, 0.5]; %[Length, Width, Height]

            % Concrete floor
            surf([-4.3, -4.3; 4.3, 4.3] ...
                , [-2.2, 2.2; -2.2, 2.2] ...
                , [0.01, 0.01; 0.01, 0.01] ...
                , 'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');

            % Place objects in environment
            PlaceObject('brownTable.ply', [0, 0, 0]);
        end

    end
end
