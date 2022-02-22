
%% JonaBot TASK SPACE TRAJECTORY GENERATION & SIMULATION

%This program creates a task space trajectory for the JonaBot. The user can
%choose between cubic polynomial trajectory or a trapezoidal trajectory.
%Also, This program calls a Simulink model to perform the simulation.
clear
clc

%% TRAJECTORY CHOICE
%Uncomment the trajecotry of choice:

trajChoice = 'Trapezoidal';

%trajChoice = 'Cubic_Polynomial';
%Uncomment the cubic trajecotry waypoint speed of choice:
   wpSpeedChoice = 'zero';
   %wpSpeedChoice ='non-zero';


switch trajChoice

    case 'Trapezoidal'
        option = 0;
    
    case 'Cubic_Polynomial'
        option = 1;
       
end
 
%% WAYPOINT DESCRIPTION

 % Provide the Home Position Transformation Matrix of the JonaBot
 target = [0   0   1   2330
           0  -1   0   0
           1   0   0   1600
           0   0   0   1];

 %Initialize an empty cell arrays to store the waypoints and velocities
 waypoints = {};
 waypointVel = {};

 %Specify WayPoints the endefector will pass trhough
 waypoints {1,1} = RyT(target,0,0); % Waypoint Orientation (TransMat, angles, 0=local rot)
 waypoints {1,1}(1:4,4) = [2330 -500 500 1]'; % Waypoint Position [x y z 1]
 waypointVel {1,1} = [0 0 0]';% Waypoint Velocity [x y z]

 waypoints {1,2} = RyT(target,50,0);
 waypoints {1,2}(1:4,4) = [3000 0 1000 1]';
 waypointVel {1,2} = [1500 300 1500]'; %Velocity through this waypoint[x y z]

 waypoints {1,3} = RxT(target,20,0);
 waypoints {1,3}(1:4,4) = [1500 1000 2000 1]';
 waypointVel {1,3} = [1800 500 500]';

 waypoints {1,4} = RyT(target,50,0);
 waypoints {1,4}(1:4,4) = [1650 -1200 1300 1]';
 waypointVel {1,4} = [1300 350 1800]';

 waypoints {1,5} = RzT(target,90,0);
 waypoints {1,5}(1:4,4) = [2500 -1650 500 1]';
 waypointVel {1,5} = [500 0 0]';

 waypoints {1,6} = RxT(target,45,0); 
 waypoints {1,6}(1:4,4) = [2330 0 1600 1]';
 waypointVel {1,6} = [0 500 0]';


%% DATA MATRIXES CREATION
 %Based on the provided waypoints create the matrixes of data to be passed
 %on the simulink model to run the simulation.

 % Initialize empty matrixes to store position, orientation, velocities,
 % and time intervals values.
 
 orientation = zeros(3*size(waypoints,2),3);
 positions = zeros(3,size(waypoints,2));
 wpVelocities = zeros (3,size(waypoints,2));
 posTimeIntervals = zeros(1,size(waypoints,2));

 for i= 1:size(waypoints,2) %iterate one time for each waypoint created

     % Orientation Matrix.
     %Store rotation matrix portion of the Transformation matrix in a Nx3
     %matrix. (N=number of waypoints*3)
     orientation((i*3)-2:i*3,1:3) = waypoints {1,i}(1:3,1:3);
     
     
     % Position Matrix.
     %Assign the x,y,z values for each waypoint in a 3xN matrix (N= number of waypoints)
     positions(1,i) = waypoints {1,i}(1,4);
     positions(2,i) = waypoints {1,i}(2,4);
     positions(3,i) = waypoints {1,i}(3,4);
      
     switch wpSpeedChoice
       case 'zero'
       case 'non-zero'
         % Velocity Matrix.
         % Assign each velocity vector of x,y,z velocities to a 3xN matrix (N= number of waypoints).
         wpVelocities(1:3,i) = waypointVel {1,i};
     end

     % Time intervals Vector.
     %Create a 1xN vector containing the time intervals (2 sec) for each waypoint
     %starting in 0 and ending in N*2 eg: [0 2 4 ... N*2].
     posTimeIntervals (1,i) = (i-1)*2;

 end

%% VISUAL PATH DATA CREATION
 %Based on the provided inputs, create path visual information to display
 %during the simulation.

 switch trajChoice
    
     case 'Trapezoidal'
          tvec = 0:0.1:posTimeIntervals(end);%create a time vector
          trajectory = trapveltraj(positions,size(tvec,2));
          visualTraject = trajectory';

     case 'Cubic_Polynomial'
          tvec = 0:0.1:posTimeIntervals(end);%create a time vector
          trajectory = cubicpolytraj(positions,posTimeIntervals,tvec,"VelocityBoundaryCondition",wpVelocities);
          visualTraject = trajectory';
 end

 
 %plot the trajectory to confirm how it will look in the simulation
 plot3(visualTraject(:,1),visualTraject(:,2),visualTraject(:,3))
 xlabel('X axis');
 ylabel('Y axis');
 zlabel('Z axis');
 title('Trajectory generated for the manipulator');
 text(-5500,2000,3300,'Opening Simulink... Please run the simulink program once opened');
 shg;
 pause(3);

 %% VISUALIZE MODEL IN SIMULINK
 % Open simulink model to visualize the JonaBot follow the trajectory
 fprintf('Opening Simulink... Please run the simulink program once opened'); 
 open("TaskSpace_Simulation_JonaBot.slx");
