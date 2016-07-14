clear 
clc
close all
% Create the robot
A = [.4, .4, .4, .2];
D = [0, 0, 0, 0];
Alpha = [0, 0, 0, 0];
Qmin = -pi/2*ones(4,1);
Qmax = pi/2*ones(4,1);
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
robotplant = RobotPlant(robot, 'end_trans');

% Create the motion model
mus = [-ones(8, 1), ones(8,1)];
sigmas(:,:,1) = eye(8);
sigmas(:,:,2) = eye(8);
As(:,:,1) = eye(4);
As(:,:,2) = 3.*eye(4);
priors = [1,1];
motion_generator = MotionGenerator(robotplant, mus, sigmas, priors, As);

Simulation([.3;.2;.3;-.2], [-.9;0;.1],motion_generator);