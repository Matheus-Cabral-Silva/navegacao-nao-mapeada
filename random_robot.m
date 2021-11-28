clc;
clear all;

%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.01;               % Sample time [s]
tVec = 0:sampleTime:15;         % Time array

initPose = [0;0;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = false;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
wRef = 0;
for idx = 2:numel(tVec) 
    vRef = 1;
    changeDirection = rand;
    if changeDirection > 0.9
        wRef = 10*rand;
        randomAlternator = rand;
        if randomAlternator < 0.5
            wRef = -1*wRef;
        end
    end
        
    
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx))
    waitfor(r);
end