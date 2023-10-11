classdef WidowX250Gripper < RobotBaseClass

    %% X250Gripper created by a 

    properties (Access = public)
        plyFileNameStem = 'WidowX250Gripper';
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
            self.PlotAndColourRobot();
            self.model;
        end

        %% Create the robot model (CHANGE ALL TO MAkE THE GRIPPER)
        function CreateModel(self)
            % Values from Diagram and Spec Sheet
            % https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/wx250s.html

            % Prismatic gripper joint DOESNT WORK YET PLEASE FIX
            % theta: -90, d: 0, a: 0, alpha: 0,  offset: ?
            % link(7) = Link([-pi / 2, 0, 0, 0, 0]); % PRISMATIC Link
            % Prismatic joint limitations
            % link(7).qlim = [0.03, 0.074];

            link(1) = Link([-pi / 2, 0, 0, 0, 0]);
            link(2) = Link([-pi / 2, 0, 0, 0, 0]);
            link(3) = Link([-pi / 2, 0, 0, 0, 0]);
            
            link(1).qlim = [-90 90]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            
            link(1).offset = -16*pi/180;
            link(2).offset = 60*pi/180;
            link(3).offset = 48*pi/180;

            self.model = SerialLink(link, 'name', self.name);
        end

    end
end