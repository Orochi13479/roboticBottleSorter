% r = WidowX250;
% rModel = r.model;
% rModel.teach;
% r.TestMoveJoints;

g = WidowX250Gripper.model;
g.teach;

% 
% qPath = jtraj(rModel.qlim(:,1)', rModel.qlim(:,2)', 200);
% for i = 1:length(qPath)
%     rModel.animate(qPath(i,:))
%     drawnow();
%     gModel.base = rModel.fkine(rModel.getpos());
%     drawnow();
%     pause(0)
% 
% end
