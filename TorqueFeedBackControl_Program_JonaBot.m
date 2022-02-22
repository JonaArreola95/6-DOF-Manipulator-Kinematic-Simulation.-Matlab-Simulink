
%% JonaBot TORQUE-CONTROLLED JOINT TRAJECTORY GENERATION & SIMULATION

% This program controls the JonaBot through the torque inputs. 
% The program uses a cubic polynomial function to generate a smooth 
% trajectory for the manipulator joint angles delta calculated.

%This program calls a Simulink model to perform the simulation.

clear
clc
% Import URDF File for manipulator
JonaBot = importrobot('Manipulator_URDF_3E\urdf\Manipulator_URDF_3E.urdf');
 
%% WAYPOINT DESCRIPTION

 % Provide the Home Position Transformation Matrix of the JonaBot
 HomePosTarget = [0   0   1   2330
                  0  -1   0   0
                  1   0   0   1600
                  0   0   0   1];

 %Initialize an empty cell arrays to store the waypoints and velocities
 waypoints = {};
 waypointVel = {};

 %Specify WayPoints the endefector needs to reach
 
 waypoints {1,1} = RzT(HomePosTarget,30,0);
 
 waypoints {1,2} = RzT(HomePosTarget,0,0);% Waypoint rotation (Frame, angles, 0=world rot or 1=local rot)
 waypoints {1,2}(1:4,4) = [2600 10 200 1]';



%% DATA MATRIXES CREATION
 %Based on the provided waypoints create the matrixes of data to be passed
 %on the simulink model to run the simulation.

 % Initialize empty matrix to store target frames and time intervals values.
 
 targets = zeros(size(waypoints,2)*4,4);
 posTimeIntervals = zeros(1,size(waypoints,2));
 angles = zeros (6,size(waypoints,2));

% Add 0 velocities to joints through the waypoints
 angTarVelocity = zeros(6,size(waypoints,2));

 for k= 1:size(waypoints,2) %iterate one time for each waypoint created

     % Targets Matrix.
     %Store target's Transformation matrix in a Nx4
     %matrix. (N=number of waypoints*3)
     targets((k*4)-3:k*4,1:4) = waypoints {1,k};
     
     
     % Time intervals Vector.
     %Create a 1xN vector containing the time intervals (2 sec) for each waypoint
     %starting in 0 and ending in N*2 eg: [0 2 4 ... N*2].
     posTimeIntervals (1,k) = 1+(k-1)*4;

     angles(1:6,k) = rad2deg(inverseKinematicsFuncOp(waypoints {1,k},k));

 end


 %% VISUALIZE MODEL IN SIMULINK
 % Open simulink model to visualize the JonaBot follow the trajectory
 fprintf('Opening Simulink. Please wait until Simulink is open...\n')  
 open("TorqueFeedBackControl_Simulation_JonaBot.slx");
