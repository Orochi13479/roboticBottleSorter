classdef WidowX250 < RobotBaseClass

    %% WidowX250 
    % Manufacturer Provided STL files converted and altered to ensure compatibility by 14289692 & 14289716
    % Manufacturer Provided Product of Exponentials converted to DH parameters by 14289692

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

            self.PlotAndColourRobot();
            % self.model.plot(zeros(1, self.model.n),'workspace', self.workspace);
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
            link(1) = Link('d', L1, 'a', 0, 'alpha', -pi/2, 'qlim', [-180, 180]*pi/180, 'offset', 0); % Blender files: 0,1 
            link(2) = Link('d', 0, 'a', -L8, 'alpha', 0, 'qlim', [-108, 114]*pi/180, 'offset', pi/2+beta); % Blender files: 2
            link(3) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-123, 92]*pi/180, 'offset', -beta); % Blender files: 3
            link(4) = Link('d', L4+L5, 'a', 0, 'alpha', -pi/2, 'qlim', [-100, 123]*pi/180, 'offset', 0); % Blender files: 4
            link(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-180, 180]*pi/180, 'offset', pi); % Blender files: 5
            link(6) = Link('d', L6+L7, 'a', 0, 'alpha', 0, 'qlim', [-180, 180]*pi/180, 'offset', -pi/2); % Blender files: 6

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end