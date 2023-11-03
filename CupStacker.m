classdef CupStacker
    properties
        UR3e
        URGripperL
        URGripperR
        WidowX250
        X250GripperL
        X250GripperR
        cupX250
        cupUR3
        initCupTrX250
        initCupTrUR3
        cupVertices
        canVertices
        finalCupTrUR3
        finalCupTrX250
        eStop = false;
        resume = true;
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
            self.cupPlacement
            % self.LightCurtainDemo
            % self.operate

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
            cupPlacement(self);
            % operate(self);

        end

        %% Environment and Figure Initialisation and Creation
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
            PlaceObject(fullfile(folderName, 'emergencyStopButton.ply'), [0.65, -1.3, tableHeight]);
            PlaceObject(fullfile(folderName, 'fireExtinguisherElevated.ply'), [-1.25, 1.5, 0.45]);
            PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [-0.4, -1, tableHeight]);
            PlaceObject(fullfile(folderName, 'rubbishBin2.ply'), [0.2, -1, tableHeight]);
            PlaceObject(fullfile(folderName, 'brownTable.ply'), [0, 0, 0]);
            PlaceObject(fullfile(folderName, 'warningSign.ply'), [1.35, -1.5, 0]);
            % PlaceObject(fullfile(folderName, 'assembledFence.ply'), [0.25, 0.7, -0.97]);
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

        end
        %% Cup and Cans Initial and Final Placements and Transforms Made
        function cupPlacement(self)
            cupHeight = 0.1;
            tableHeight = 0.5;
            folderName = 'data';

            % 14 Cups to Start with
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
                % Convert Coords to Transforms
                self.initCupTrX250(:, :, i) = transl(initCupArrayX250(i, 1), initCupArrayX250(i, 2), initCupArrayX250(i, 3)+cupHeight) * troty(pi);
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
                % Convert Coords to Transforms
                self.initCupTrUR3(:, :, i) = transl(initCupArrayUR3(i, 1), initCupArrayUR3(i, 2), initCupArrayUR3(i, 3)+cupHeight) * troty(pi);
            end


            % Get Cup and Can vertices from ply file
            [tri, self.cupVertices] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');
            [tri, self.canVertices] = plyread(fullfile(folderName, 'sodaCan.ply'), 'tri');

            % Hardcode Final Cup Locations
            bin1x = 0.3;
            bin2x = -0.3;
            biny = -1.0;

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

            pause(1); % Let environment Spawn in

            disp('Plastic Cups Created');
            disp('Setup is complete');

        end
        %% Operation Function
        function operate(self)
            steps = 200;
            cupHeight = 0.1;
            tableHeight = 0.5;

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
            qCommonPickupX250 = X250.ikcon(optimalEndEffectorPickupX250);
            qCommonPickupUR3 = UR3.ikcon(optimalEndEffectorPickupUR3);

            qCommonDropoffX250 = X250.ikcon(optimalEndEffectorDropoffX250);
            qCommonDropoffUR3 = UR3.ikcon(optimalEndEffectorDropoffUR3);

            % Gripper Trajectory Constant with all Uses
            qOpenGripper = [0, 0.03];
            qCloseGripper = [0, 0.05];
            closeTraj = jtraj(qOpenGripper, qCloseGripper, steps/4);
            openTraj = jtraj(qCloseGripper, qOpenGripper, steps/4);

            UR3eqOpenGripper = [0, 0, 0];
            UR3eqCloseGripper = deg2rad([30, 22.5, -52.5]);
            UR3ecloseTraj = jtraj(UR3eqOpenGripper, UR3eqCloseGripper, steps/4);
            UR3eopenTraj = jtraj(UR3eqCloseGripper, UR3eqOpenGripper, steps/4);

            % while self.eStop == false || self.resume == true
                % for i = 1:(length(finalCupArrayUR3))
                %     disp("Running...")
                %     if i == 1
                %         % Initial Starting Position
                %         qStartX250 = zeros(1, X250.n);
                %         qStartUR3 = zeros(1, UR3.n);
                %     else
                %         qStartX250 = X250.ikcon(self.finalCupTrX250(:, :, i-1), qCommonDropoffX250);
                %         qStartUR3 = UR3.ikcon(self.finalCupTrUR3(:, :, i-1), qCommonDropoffUR3);
                %     end
                %     qInitialX250 = X250.ikcon(self.initCupTrX250(:, :, i), qCommonPickupX250);
                %     qFinalX250 = X250.ikcon(self.finalCupTrX250(:, :, i), qCommonDropoffX250);
                %     % pickupTrajX250 = jtraj(qStartX250, qInitialX250, steps);
                %     % dropoffTrajX250 = jtraj(qInitialX250, qFinalX250, steps);
                % 
                %     qInitialUR3 = UR3.ikcon(self.initCupTrUR3(:, :, i), qCommonPickupUR3);
                %     qFinalUR3 = UR3.ikcon(self.finalCupTrUR3(:, :, i), qCommonDropoffUR3);
                %     % pickupTrajUR3 = jtraj(qStartUR3, qInitialUR3, steps);
                %     % dropoffTrajUR3 = jtraj(qInitialUR3, qFinalUR3, steps);
                % 
                % 
                %     % RMRC ATTEMPT
                %     trStartX250 = X250.fkine(qStartX250).T;
                %     trInitialX250 = X250.fkine(qInitialX250).T;
                %     trFinalX250 = X250.fkine(qFinalX250).T;
                % 
                %     trStartUR3 = UR3.fkine(qStartUR3).T;
                %     trInitialUR3 = UR3.fkine(qInitialUR3).T;
                %     trFinalUR3 = UR3.fkine(qFinalUR3).T;
                % 
                %     pickupTrajX250 = RMRC(X250, trStartX250, trInitialX250);
                %     dropoffTrajX250 = RMRC(X250, trInitialX250, trFinalX250);
                % 
                %     pickupTrajUR3 = RMRC(UR3, trInitialUR3, trFinalUR3);
                %     dropoffTrajUR3 = RMRC(UR3, trFinalUR3, trFinalUR3);
                % 
                % 
                %     for j = 1:steps
                %         X250.animate(pickupTrajX250(j, :));
                %         UR3.animate(pickupTrajUR3(j, :));
                %         WidowX250GripperL.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
                %         WidowX250GripperL.animate(WidowX250GripperL.getpos());
                %         WidowX250GripperR.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
                %         WidowX250GripperR.animate(WidowX250GripperR.getpos());
                %         UR3eGripperL.base = UR3.fkine(UR3.getpos()).T * trotx(pi/2);
                %         UR3eGripperL.animate(UR3eGripperL.getpos());
                %         UR3eGripperR.base = UR3.fkine(UR3.getpos()).T * trotz(pi) * trotx(pi/2);
                %         UR3eGripperR.animate(UR3eGripperR.getpos());
                %         drawnow();
                %     end
                % 
                %     for j = 1:steps / 4
                %         WidowX250GripperL.animate(closeTraj(j, :));
                %         WidowX250GripperR.animate(closeTraj(j, :));
                %         UR3eGripperL.animate(UR3ecloseTraj(j, :));
                %         UR3eGripperR.animate(UR3ecloseTraj(j, :));
                %         drawnow();
                %     end
                % 
                %     for j = 1:steps
                %         X250.animate(dropoffTrajX250(j, :));
                %         UR3.animate(dropoffTrajUR3(j, :));
                %         WidowX250GripperL.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * troty(pi) * transl(0, 0.023, 0);
                %         WidowX250GripperL.animate(WidowX250GripperL.getpos());
                %         WidowX250GripperR.base = X250.fkine(X250.getpos()).T * trotx(-pi/2) * transl(0, 0.023, 0);
                %         WidowX250GripperR.animate(WidowX250GripperR.getpos());
                %         UR3eGripperL.base = UR3.fkine(UR3.getpos()).T * trotx(pi/2);
                %         UR3eGripperL.animate(UR3eGripperL.getpos());
                %         UR3eGripperR.base = UR3.fkine(UR3.getpos()).T * trotz(pi) * trotx(pi/2);
                %         UR3eGripperR.animate(UR3eGripperR.getpos());
                %         drawnow();
                %     end
                % 
                %     for j = 1:steps / 4
                %         WidowX250GripperL.animate(openTraj(j, :));
                %         WidowX250GripperR.animate(openTraj(j, :));
                %         UR3eGripperL.animate(UR3eopenTraj(j, :));
                %         UR3eGripperR.animate(UR3eopenTraj(j, :));
                %         drawnow();
                %     end
                % end
                % 
                % disp("Finished")

            % end

            if self.eStop
                save eStopVariablesSaved -regexp ^(?!(self)$). % saves local variables, except self
            end

        end

        %% Light Curtain Demo
        function LightCurtainDemo(self)

            [y1,z1] = meshgrid(-1.5:0.01:1, 0.1:0.01:1.5);  %location of meshgrid
            x1 = zeros(size(y1)) - 1.2;
            lightCurtain1 = surf(x1,y1,z1,'FaceAlpha',0.1,'EdgeColor','none');

            hold on;

            [f, v, data] = plyread(fullfile('data', 'sodaCan.ply'), 'tri');
            cV = v;

            steps = 30;

            xCan = -1.5;
            canHandles = [];

            for j = 1:steps
                % Delete previously created cans
                if ~isempty(canHandles)
                    delete(canHandles);
                    canHandles = [];
                end

                % Create a new soda can
                canHandle = PlaceObject(fullfile('data', 'sodaCan.ply'), [xCan, 0, 0.6]);
                canHandles = [canHandles, canHandle]; % Store the handle

                pause(0.2);

                xCan = xCan + 0.01;
                cV(:, 1) = cV(:, 1) + xCan;
                drawnow;

                if xCan >= -1.2
                    fprintf("Light Curtain has been activated\n");
                    lightCurtain1 = surf(x1, y1, z1, 'FaceAlpha', 0.1, 'FaceColor', 'red');
                    set(gcf, 'color', 'r');
                end
            end
        end

        function emergencyStop(self)
            self.eStop = true;
            self.resume = false;
        end

        function resumeButton(self)
            self.resume = true;
            self.eStop = false;
        end

    end
end