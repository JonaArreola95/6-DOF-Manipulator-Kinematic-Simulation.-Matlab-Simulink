
%% JonaBot JOINT-SPACE TRAJECTORY GENERATION & SIMULATION

% This program creates a joint-space trajectory for the JonaBot. The program
% uses a cubic polynomial function to generate a smooth trajectory for the
% manipulator joint angles
%Also,This program calls a Simulink model to perform the simulation.

clear
clc
 
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
 
 waypoints {1,1} = RzT(HomePosTarget,0,0);
 
 waypoints {1,2} = RzT(HomePosTarget,170,0);% Waypoint rotation (Frame, angles, 0=world rot or 1=local rot)
 waypoints {1,2} = RyT(waypoints {1,2},-30,0); % Other rotation
 waypoints {1,2} = RxT(waypoints {1,2},30,1); % And one more...
 waypoints {1,2} = RyT(waypoints {1,2},80,1); % One last one.
 %waypoints {1,2}(1:4,4) = [2600 -500 500 1]'; % Waypoint Position is given
 %by the Transformation rotations, so no need to specify it here.
 
 waypoints {1,3} = RyT(HomePosTarget,0,0);
 waypoints {1,3}(1:4,4) = [2600 -2300 1000 1]';

 waypoints {1,4} = RyT(HomePosTarget,0,0);
 waypoints {1,4}(1:4,4) = [1600 -2300 1000 1]';


%% DATA MATRIXES CREATION
 %Based on the provided waypoints create the matrixes of data to be passed
 %on the simulink model to run the simulation.

 % Initialize empty matrix to store target frames and time intervals values.
 
 targets = zeros(size(waypoints,2)*4,4);
 posTimeIntervals = zeros(1,size(waypoints,2));
 angles = zeros (6,size(waypoints,2));

% Add ramdom velocities to joints through the waypoints or choose zero
% velocity
 angTarVelocity = rand(6,size(waypoints,2));
%  angTarVelocity = zeros(6,size(waypoints,2));
 
 angTarVelocity(:,1) = zeros(6,1); % Set initial waypoint velocity to zero
 angTarVelocity(:,end) = zeros(6,1);% Set initial waypoint velocity to zero

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
%% VISUAL PATH DATA CREATION
 %Based on the provided inputs, create path visual information to display
 %during the simulation.


  tvec = 0:0.1:posTimeIntervals(end);%create a time vector
  traj = cubicpolytraj(angles,posTimeIntervals,tvec,"VelocityBoundaryCondition",rad2deg(angTarVelocity));
  
  eeTrajec = zeros(size(traj,2),3);
  
  for k=1:size(traj,2)
      transMat = forwardKinematicsFuncTCP(traj(1,k),traj(2,k),traj(3,k), ...
                                       traj(4,k),traj(5,k),traj(6,k));
      eeTrajec(k,1:3)= transMat(1:3,4)';
  end
  visualTraject = eeTrajec(12:end,:);


 %plot the trajectory to confirm how it will look in the simulation
 plot3(visualTraject(:,1),visualTraject(:,2),visualTraject(:,3))
 xlabel('X axis');
 ylabel('Y axis');
 zlabel('Z axis');
 title('Trajectory to be followed by the manipulator TCP');
 text(-5500,2000,3300,'Opening Simulink... Please run the simulink program once opened');
 shg;
 pause(3);

 %% VISUALIZE MODEL IN SIMULINK
 % Open simulink model to visualize the JonaBot follow the trajectory
 fprintf('Opening Simulink... Please run the simulink program once opened'); 
 open("JointSpace_Simulation_JonaBot.slx");
