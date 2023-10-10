classdef WidowX250 < RobotBaseClass

    %% WidowX250 created by a 14289692

    properties (Access = public)
        plyFileNameStem = 'WidowX250';
    end

    methods

        %% Define robot Function
        function self = WidowX250(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;

            %% UNCOMMENT OUT WHEN SOMEONE ADDS PLY FILES
            % self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)
            % Values from Diagram and Spec Sheet
            % https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/wx250s.html
            L1 = 0.11025;
            L2 = 0.25;
            L3 = 0.05;
            L4 = 0.17155;
            L5 = 0.07845;
            L6 = 0.065;
            L7 = 0.066;
            L8 = 0.25495;
            beta = asin(L3/L2);

            % Create the WidowX250 model
            link(1) = Link('d', L1, 'a', 0, 'alpha', -pi/2, 'qlim', [-180, 180]*pi/180, 'offset', 0); % STL files: 1,2
            link(2) = Link('d', 0, 'a', -L8, 'alpha', 0, 'qlim', [-108, 114]*pi/180, 'offset', pi/2+beta); % STL files: 3
            link(3) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-123, 92]*pi/180, 'offset', -beta); % STL files: NONE
            link(4) = Link('d', L4+L5, 'a', 0, 'alpha', -pi/2, 'qlim', [-100, 123]*pi/180, 'offset', 0); % STL files: 4, 5
            link(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-180, 180]*pi/180, 'offset', pi); % STL files: NONE
            link(6) = Link('d', L6+L7, 'a', 0, 'alpha', -pi/2, 'qlim', [-180, 180]*pi/180, 'offset', pi/2); % STL files: 6, 7

            % Prismatic gripper joint
            % theta: -90, d: 0, a: 0, alpha: 0,  offset: ?
            link(7) = Link([-pi / 2, 0, 0, 0, 0]); % PRISMATIC Link
            % Prismatic joint limitations
            link(7).qlim = [0.03, 0.074];

            self.model = SerialLink(link, 'name', self.name);
        end

    end
end