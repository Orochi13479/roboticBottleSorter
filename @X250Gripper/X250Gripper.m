classdef X250Gripper < RobotBaseClass

    %% X250Gripper created by a 

    properties (Access = public)
        plyFileNameStem = 'WidowX250';
    end

    methods

        %% Define robot Function
        function self = X250Gripper(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;

            %% UNCOMMENT OUT WHEN SOMEONE ADDS PLY FILES
            % self.PlotAndColourRobot();
        end

        %% Create the robot model (CHANGE ALL TO MAkE THE GRIPPER)
        function CreateModel(self)
            % Values from Diagram and Spec Sheet
            % https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/wx250s.html

            % Prismatic gripper joint DOESNT WORK YET PLEASE FIX
            % theta: -90, d: 0, a: 0, alpha: 0,  offset: ?
            link(7) = Link([-pi / 2, 0, 0, 0, 0]); % PRISMATIC Link
            % Prismatic joint limitations
            link(7).qlim = [0.03, 0.074];

            self.model = SerialLink(link, 'name', self.name);
        end

    end
end