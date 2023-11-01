close all;
clear all;
clc;

r = UR3e;
rModel = r.model;
rModel.teach;

% Fill each deg2rad function with the 6 joint states add however many
% necessary the first and last should be the same as they are robot initial
% position ALSO please use waypoints or the robot will go crazy!!
targetJointStates = [
    [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8];
    deg2rad([23,23,23,23,23,23]);
    deg2rad([]);
    deg2rad([]);
    deg2rad([]);
    [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8]
]  