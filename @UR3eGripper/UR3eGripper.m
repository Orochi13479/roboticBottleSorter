classdef UR3eGripper < RobotBaseClass

    %% UR3eGripper created by 14264764

    properties (Access = public)
        plyFileNameStem = 'UR3eGripper';
    end

    methods

        %% Define robot Function
        function self = UR3eGripper(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;

            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)
            link(1) = Link([0      0      0.05      0      0]);
            link(2) = Link([0      0      0.045      0      0]);
            link(3) = Link([0      0      0.045       0      0]);

            link(1).qlim = [-90 90]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-90 90]*pi/180;

            link(1).offset = -16*pi/180;
            link(2).offset = 58*pi/180;
            link(3).offset = 48*pi/180;

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end