%% MOVES THE JONABOT IN A CIRCULAR TRAJECTORY
bot = importrobot("JonaBot_Manipulator\urdf\JonaBot_Manipulator.urdf");

 waypoint = [0    0   1   0
             0   -1   0   0
             1    0   0   0
             0    0   0   1];



%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.15:10)'; % Time vector
count = length(t); % count is the number of item in the vector "time"
center = [2500 0 1600]; %XYZ coordinates of the circle center
radius = 500; 
theta = t*(2*pi/t(end)); % Thetha vector. Divides 360 deg in the total time, each timestep (vector pos) angle increases 7.2 deg
points = center + radius*[zeros(size(theta)) cos(theta) sin(theta)]; %points mattrix of 3xcount. contains the points data XYZ for each time interval.
%the colums of the vector changes depending of the value of theta.


%% Pre define positions container
% Pre-allocate configuration solutions as a matrix |qs|.
q0 = homeConfiguration(bot); %copies home configuration cell array to use it later
ndof = length(q0);
qs = zeros(count, ndof); %Creates a cell array matrix for each timestep containing all of the angles solutions for each timestep.

%%
% Loop through the trajectory of points to trace the circle. Call the ik
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.

for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:); % assigns each point's XYZ values from the points matrix to this point vector. 
    frame = [waypoint(1:3,1:3) point';0 0 0 1];% assigns the same orientation to each point's XYZ values and adds the row 4 of the Tmat. 
    qan = inverseKinematicsFunc(frame);   
    qanr = deg2rad(qan);
    % Store the configuration
    qs(i,:) = qanr;

end

    %% Animate The Solution
    % Plot the robot for each frame of the solution using that specific robot 
    % configuration. Also, plot the desired trajectory.
    
    %%
    % Show the robot in the first configuration of the trajectory. Adjust the 
    % plot to show the 2-D plane that circle is drawn on. Plot the desired 
    % trajectory.

    %figure
    view([110 40]) %Azimuth and elevation of figure view
    qcell = num2cell(qs(1,:)); % Convert vetor to cell array
    config = homeConfiguration(bot);
    [config.JointPosition] = qcell{:}; %deals each cell array element to the cell array namaspace joint postion values.
    show(bot,config);

  %%
% Set up a <docid:robotics_ref.mw_9b7bd9b2-cebc-4848-a38a-2eb93d51da03 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circular trajectory shown.
framesPerSecond = 24;
r = rateControl(framesPerSecond);
% while true
    for i = 1:count
        
        qcell = num2cell(qs(i,:)); % convert vetor to cell array
        [config.JointPosition] = qcell{:}; %deals each cell array element to the cell array namaspace joint postion values.

        show(bot,config,'PreservePlot',true);
        drawnow
        waitfor(r);
    end
% end



