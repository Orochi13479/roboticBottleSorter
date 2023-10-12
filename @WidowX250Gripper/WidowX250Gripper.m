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
            
            % Prismatic gripper joint DOESNT WORK YET PLEASE FIX
            % theta: -90, d: 0, a: 0, alpha: 0,  offset: ?
            % link(7) = Link([-pi / 2, 0, 0, 0, 0]); % PRISMATIC Link
            % Prismatic joint limitations
            % link(7).qlim = [0.03, 0.074];

            %% DONT SEEM TO BE PRISMATIC JOINTS AND WILL HAVE TO MOST LIKELY
            %% DO TWO FINGERS INSTEAD OF ONE GRIPPER
        
            link(1) = Link([pi/2, 0, 0, 0, 0]);
            link(2) = Link([pi/2, 0, 0, 0, 0]);
            link(3) = Link([0, 0, 0, 0, 0]);
            

            link(1).qlim = [0.03, 0.074];
            link(2).qlim = [0.03, 0.074];
            link(3).qlim = [0.03, 0.074];
            

            link(1).offset = 0;
            link(2).offset = pi/2;
            link(3).offset = 0;
            

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end