classdef CollisionCheck < handle
    properties (Access = public)
        centerPoints;
        radii;
        robot;
    end
    methods
        function self = CollisionCheck(robot, centerPoints, radii)
            self.robot = robot;
            self.centerPoints = centerPoints;
            self.radii = radii;
        end

        %% Plot ellipsoids on robot
        function plotEllipsoids(self)
            for i = 1:self.robot.model.n
                [X, Y, Z] = ellipsoid(self.centerPoints(i,1), self.centerPoints(i,2), self.centerPoints(i,3), self.radii(i,1), self.radii(i,2), self.radii(i,3));
                self.robot.model.points{i} = [X(:),Y(:),Z(:)];

                self.robot.model.faces{i} = delaunay(self.robot.model.points{i});
            end

            self.robot.model.plot3d(self.robot.model.getpos());
        end

        %% Check for collisions
        function collision = checkCollision(self, meshPoints)
            collision = 0;
            q = self.robot.model.getpos();
            tr = zeros(4,4,self.robot.model.n+1);
            tr(:,:,1) = self.robot.model.base;
            L = self.robot.model.links;
            for i = 1:self.robot.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % Go through each ellipsoid
            for i = 1: size(tr,3)
                meshPointsAndOnes = [inv(tr(:,:,i)) * [meshPoints,ones(size(meshPoints,1),1)]']';
                updatedmeshPoints = meshPointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedmeshPoints, self.centerPoints(i,:), self.radii(i,:));
                pointsInside = find(algebraicDist < 1);
                collisionPoints = size(pointsInside,1);
                if collisionPoints > 0
                   collision = 1;
                    break
                end
            end
        end
    end
end