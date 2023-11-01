classdef WidowX250Gripper < RobotBaseClass

    %% X250Gripper created by 14289692

    properties (Access = public)
        plyFileNameStem = 'X250Gripper';
    end

    methods

        %% Define robot Function
        function self = WidowX250Gripper(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end 
            self.model.base = self.model.base.T * baseTr;
            

            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)
            % Values from Diagram and Spec Sheet
            % https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/wx250s.html

            % Define DH parameters for the gripper finger
            a1 = 0.03; % Link length
            a2 = 0.074; % Link length

            link(1) = Link('d', 0.01, 'a', -0.04, 'alpha', pi/2, 'offset', pi/2); 
            link(2) = Link('theta', 0, 'a', a2-a1, 'alpha', pi/2, 'offset', -a2, 'prismatic', 'qlim', [a1, a2]);

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end