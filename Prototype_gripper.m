%% DANIEL GRIPPER
% Lab 2.4 modified
LinkG(1) = Link('d',0,'a',0.025,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
LinkG(2) = Link('d',0,'a',0.025,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
LinkG(3) = Link('d',0,'a',0.025,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);

LinkG(1).qlim = [-90 90]*pi/180;
LinkG(2).qlim = [-90 90]*pi/180;
LinkG(3).qlim = [-90 90]*pi/180;

LinkG(1).offset = -16*pi/180;
LinkG(2).offset = 60*pi/180;
LinkG(3).offset = 48*pi/180;

leftGripper = 'LeftGripper';
rightGripper = 'RightGripper';
gripperBase = eye(4);

LeftGripper = SerialLink(LinkG,'name',leftGripper,'base',gripperBase);
RightGripper = SerialLink(LinkG,'name',rightGripper,'base',gripperBase);

LeftGripper.base = armModel.fkine(armModel.getpos()).T*trotx(pi/2);
RightGripper.base = armModel.fkine(armModel.getpos()).T*trotz(pi)*trotx(pi/2);


%% Gripper/Finger plots
% LeftGripper.plot(zeros(1,LeftGripper.n),'noarrow','workspace',workspace);
% RightGripper.plot(zeros(1,RightGripper.n),'noarrow','workspace',workspace);

%% OROCHI GRIPPER
classdef Gripper < RobotBaseClass

    %% Gripper UR3 on a non-standard linear rail created by a student

    properties (Access = public)
        plyFileNameStem = 'Gripper';
    end

    methods

        %% Define robot Function
        function self = Gripper(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);

            % self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)

            link(1) = Link('d', 0, 'a', 0.04, 'alpha', 0, 'qlim', [-90, 90]*pi/180, 'offset', pi/180);
            link(2) = Link('d', 0, 'a', 0.04, 'alpha', 0, 'qlim', [-90, 90]*pi/180, 'offset', 40*pi/180);
            link(3) = Link('d', 0, 'a', 0.04, 'alpha', 0, 'qlim', [-90, 90]*pi/180, 'offset', 70*pi/180);

            self.model = SerialLink(link, 'name', self.name);
        end

    end
end
