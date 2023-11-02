classdef CupStacker
    properties
        UR3e
        URGripperL
        URGripperR
        WidowX250
        X250GripperL
        X250GripperR
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

            % Initialise and Plot the WidowX250 Gripper object
            self.X250GripperL = WidowX250Gripper;
            WidowX250GripperL = self.X250GripperL.model;
            self.X250GripperR = WidowX250Gripper;
            WidowX250GripperR = self.X250GripperR.model;

            %Initialise and Plot the UR3e Gripper object
            self.URGripperL = UR3eGripper;
            UR3eGripperL = self.URGripperL.model;
            self.URGripperR = UR3eGripper;
            UR3eGripperR = self.URGripperR.model;

            % % Reduce lag
            UR3.delay = 0;
            X250.delay = 0;
            WidowX250GripperL.delay = 0;
            WidowX250GripperR.delay = 0;
            UR3eGripperL.delay = 0;
            UR3eGripperR.delay = 0;

            disp('Robots Initialised');

            % Manipulate the WidowX250 Base If custom position e.g. On top of a table
            % Gather default rotation and translation matrices
            [armRotationMatrix1, armTranslationVector1] = tr2rt(X250.base);
            [armRotationMatrix2, armTranslationVector2] = tr2rt(UR3.base);

            translationVector1 = [-0.3, -0.6, 0.5];
            translationVector2 = [0.3, -0.6, 0.5];

            % Specify the rotation angle in radians
            angle = pi;

            % Create a rotation matrix for the X-axis rotation
            Rx = [1, 0, 0; ...
                0, cos(angle), -sin(angle); ...
                0, sin(angle), cos(angle)];

            Rz = [cos(angle), -sin(angle), 0; ...
                sin(angle), cos(angle), 0; ...
                0, 0, 1];

            % Create a rotation matrix for the Z-axis rotation
            Rz = [cos(angle), -sin(angle), 0; ...
                sin(angle), cos(angle), 0; ...
                0, 0, 1];

            X250.base = rt2tr(armRotationMatrix1, translationVector1);
            UR3.base = rt2tr(armRotationMatrix2, translationVector2);

            % Set Base of WidowX250 Gripper to End effector
            WidowX250GripperL.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
            WidowX250GripperR.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);

            % Set Base of UR3e Gripper to End effector
            UR3eGripperL.base = UR3.fkine(UR3.getpos).T*trotx(pi/2);
            UR3eGripperR.base = UR3.fkine(UR3.getpos).T*trotz(pi)*trotx(pi/2);

            % Assume starting position
            UR3.animate(UR3.getpos());
            X250.animate(X250.getpos());
            WidowX250GripperL.animate([0, 0.03]);
            WidowX250GripperR.animate([0, 0.03]);
            UR3eGripperL.animate([0, 0, 0]);
            UR3eGripperR.animate([0, 0, 0]);

            q1 = [-pi / 4, 0, 0];
            q2 = [pi / 4, 0, 0];
            steps = 2;
            while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1, q2, steps)))),1))
                steps = steps + 1;
            end
            qMatrix = jtraj(q1, q2, steps);

            disp('Robots Mounted');

            % Environment Setup
            environment(self);

            % operate(self);
        end


        function environment(self)
            zlim([0, 2]);
            xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
            ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER

            folderName = 'data';
            hold on

            % Environment - Table dimensions
            TableDimensions = [2.1, 1.4, 0.5]; %[Length, Width, Height]
            % wheeledTableDimensions = [0.75, 1.2, 0.52]; %[Length, Width, Height]
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

        end

        function operate(self)
            % Begin operation
            UR3.TestMoveJoints;
            X250.TestMoveJoints;
        end

    end
end
