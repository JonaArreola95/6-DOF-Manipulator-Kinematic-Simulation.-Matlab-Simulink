%% INVERSE KINNEMATICS FOR REDUNDANT MANIPULATOR PROGRAM
% This program calculates the inverse kinematics given a certain target for
% the provided redundant manipulator. It provides the option to rotate or 
% change the position of the target. The program then prints out all the
% possible solutions and then it displays the robot model in all these 
% different solutions.

clear %Clear workspace
clc %Clear command window
%% Provide a sample target for the manipulator (target below shows home position)
 targetFrame =[0   0   1   2950 %2133 home
               1   0   0   1850 %1748 home
               0   1   0   0
               0   0   0   1];

 %rotate the target if desired. 
 % Arguments: (transformation matrix, rotation in deg, 0= world reference or 1= local reference)
  targetFrame = RzT(targetFrame,90,1);
  targetFrame = RxT(targetFrame,90,1);
  targetFrame = RyT(targetFrame,0,0);


%% 1.INITIALIZATION OF VARIABLES
% Allocate a variables for the robot angles (theta 1-6)
 th1 = 0;
 th2 = 0;
 d3 = 0;
 th4 = 0;
 th5 = 0;
 th6 = 0;

% Create the prismatic joint movement step vector
 d3Offset = (0:10:800)';

% Allocate empty matrixes to store all of the possible solution sets
 joints123n456 = zeros(length(d3Offset),6);
 joints123n4p5p6p = zeros(length(d3Offset),6);
 joints1p2p3pn4B5B6B = zeros(length(d3Offset),6);
 joints1p2p3pn4pB5pB6pB = zeros(length(d3Offset),6);



%% 2.FIND WRIST POS FOR THE MANIPULATOR
    
% Provide the transformation matrix from frame {6}(wrist) to {TCP}
 T6tcp = [1   0   0   0
          0   1   0   0
          0   0   1   547 % originally 22, it was scaled for simulation pruposes
          0   0   0   1];

% Equal the position of the TCP to the target one.
 Tp0tcp = targetFrame;

% Find the position of the wrist (T06') in the desired position. i.e.
% when the TCP is in the desired position (equaled to the target frame)
% using the equation T06' = T0tcp' * (T6tcp)^-1

 Ttcp6 = transMatTranspose(T6tcp);
 Tp06 = Tp0tcp * Ttcp6;

% In the manipulator configuration chosen, the first three joints
% provide the position for the TCP frame, so the program will only require
% the position vector from the Transformation Matrix T06. (wPos = P'06).

% Extract the position vector of the transformation Matrix  
 wPos = Tp06 (1:4,4);
    
    
 %% PERFORM THE CALCULATIONS FOR EVERY STEP OF THE PRISMATIC JOINT
 for i = d3Offset'

     k = (i/10)+1; %cretae a counter of step 1
     d3link = 518; %provide the link lenght when d3 = 0
     d3len = d3link + i; %increase d3 lenght in each iteration

     % Provide the length of the link 1, 2
     l1 = 1748; % originally 412, it was scaled for simulation pruposes
     l2 = 1068 + d3len; % originally 267, it was scaled for simulation pruposes
            
    %% 3. FIND THETA 2 FOR THE MANIPULATOR.
      
    %check if the desired position is in reach of the manipulator before doing 
    %the calculation
     if ((l1+(l2)) > (sqrt((wPos(1)^2)+(wPos(2)^2))))
       %if it is in reach, calculate theta 2 using the law of cosines function
        th2 = angleFromLawOfCosines(l1,l2,wPos(1),wPos(2),1)*-1;
    
       %find the other possible solution of theta 3: theta3 prime = (mirror of theta3)
        th2p = -th2;
     else
          th2 = nan;
          th2p = nan;
     end
      
    %% 4. FIND THETA 1 FOR THE MANIPULATOR.
    if  isnan(th2)
        th1 = nan;
        th1p = nan;
    
    else
        %find gamma and beta angles.
         y = angleFromLawOfCosines(l1,l2,wPos(1),wPos(2),2);
        %  b = atan2d(wPos16(3),abs(wPos16(1)-a2));
         b = atand(wPos(2)/(abs(wPos(1))));
        
        %decide whether gamma should be summed or substracted based on the sign
        % (position) of theta 2
         if (th2 > 0)
            th1 = b - y;
         else
            th1 = b + y;
         end
        
        %find the second solution of theta 1
        
        %  if (th2p > 0)
        %     th1p = b - y;
        %  else
        %     th1p = b + y;
        %  end
        % or
        % with a faster calculation:
         th1p = (b*2) - th1;
    end



    %% Store all of the solutions for the first three joints (normal and prime)
    if isnan(th2)
        joints123n456(k,1:3) = [th1 th2 nan];
        joints123n4p5p6p(k,1:3) = [th1 th2 nan];
        joints1p2p3pn4B5B6B(k,1:3) = [th1p th2p nan];
        joints1p2p3pn4pB5pB6pB(k,1:3) = [th1p th2p nan];
    else
        joints123n456(k,1:3) = [th1 th2 i];
        joints123n4p5p6p(k,1:3) = [th1 th2 i];
        joints1p2p3pn4B5B6B(k,1:3) = [th1p th2p i];
        joints1p2p3pn4pB5pB6pB(k,1:3) = [th1p th2p i];
    end
%% 5. FIND THETA 4,5,6 FOR THE MANIPULATOR.

%Once the position is set, the rest of the angles calculate the
%orientation of the end effector. For this, we need to know what is the
%orientation of frame 4 when the position is set. Then the rest of the
%angles are calculated from the euler angles equations.

% **** A. Calculations for th1,th2,th3 *****

% Calculate T46 (frame 6 seen in 4) when in the desired position
 th4 = 0; %Set theta 4 to zero
%Calculate the posotion of each frame with the angle values obtain so
%far (theta 1,theta 2,theta 3, and set theta 4 = 0)
 tmatarray = forwardKinematicsFuncRedundant(th1,th2,d3,th4,th5,th6); 
 [Tp01,Tp12,Tp23,Tp34,Tp45,Tp56,Tp6tcp] = tmatarray{:};

%Calculate frame 4' in 0 using the angle values known so far.
 Tp24 = Tp23*Tp34;
 Tp14 = Tp12*Tp24;
 Tp04 = Tp01*Tp14;

%Calculate frame {6'} represented in {4'} when theta 4 = 0.
 Tp40 = transMatTranspose(Tp04); 
 Tp46 = Tp40* Tp06;
    
%Calculate angle theta 4, theta 5, and theta 6 for 0<=Th5<=180

%(Round2Zero creates a real zero value. If for example Cos(p1/2) is
%calculated, mathlab provides a really small number instead of a true 0 that
%can create erratic degree results when these are input in atan2 function).
 th4 = atan2d(round2Zero(Tp46(2,3)),round2Zero(Tp46(1,3)));
 th5 = atan2d(round2Zero(sqrt(Tp46(1,3)^2 + Tp46(2,3)^2 )),round2Zero(Tp46(3,3)));
 th6 = atan2d(round2Zero(Tp46(3,2)),round2Zero(-Tp46(3,1)));

%Calculate angle theta 4, theta 5, and theta 6 for -180<=Th5<=0
th4p = atan2d(round2Zero(-Tp46(2,3)),round2Zero(-Tp46(1,3)));
th5p = atan2d(round2Zero(-sqrt(Tp46(1,3)^2 + Tp46(2,3)^2 )),round2Zero(Tp46(3,3)));
th6p = atan2d(round2Zero(-Tp46(3,2)),round2Zero(Tp46(3,1)));


% **** B. Calculations for th1p,th2p,th3p (prime angles)*****

% Calculate T46 (frame 6 seen in 4) when in the desired position
 th4B = 0; %Set theta 4 to zero
%Calculate the posotion of each frame with the angle values obtain so
%far (theta 1',theta 2' ,theta 3' , and set theta 4 = 0)
 tmatarray = forwardKinematicsFuncRedundant(th1p,th2p,d3,th4B,th5,th6); 
 [Tp01B,Tp12B,Tp23B,Tp34B,Tp45B,Tp56B,Tp6tcpB] = tmatarray{:};

%Calculate frame 4' in 0 using the angle values known so far.
 Tp24B = Tp23B*Tp34B;
 Tp14B = Tp12B*Tp24B;
 Tp04B = Tp01B*Tp14B;

%Calculate frame {6'} represented in {4'} when theta 4 = 0.
 Tp40B = transMatTranspose(Tp04B); 
 Tp46B = Tp40B* Tp06;
    
%Calculate angle theta 4, theta 5, and theta 6 for 0<=Th5<=180

%(Round2Zero creates a real zero value. If for example Cos(p1/2) is
%calculated, mathlab provides a really small number instead of a true 0 that
%can create erratic degree results when these are input in atan2 function).
 th4B = atan2d(round2Zero(Tp46B(2,3)),round2Zero(Tp46B(1,3)));
 th5B = atan2d(round2Zero(sqrt(Tp46B(1,3)^2 + Tp46B(2,3)^2 )),round2Zero(Tp46B(3,3)));
 th6B = atan2d(round2Zero(Tp46B(3,2)),round2Zero(-Tp46B(3,1)));

%Calculate angle theta 4, theta 5, and theta 6 for -180<=Th5<=0
th4pB = atan2d(round2Zero(-Tp46B(2,3)),round2Zero(-Tp46B(1,3)));
th5pB = atan2d(round2Zero(-sqrt(Tp46B(1,3)^2 + Tp46B(2,3)^2 )),round2Zero(Tp46B(3,3)));
th6pB = atan2d(round2Zero(-Tp46B(3,2)),round2Zero(Tp46B(3,1)));


%% Add offset to theta2
% th2 = th2 + 90;
% th2p = th2p + 90;

    joints123n456(k,2) = joints123n456(k,2) + 90;
    joints123n4p5p6p(k,2) = joints123n4p5p6p(k,2) + 90;
    joints1p2p3pn4B5B6B(k,2) = joints1p2p3pn4B5B6B(k,2) + 90;
    joints1p2p3pn4pB5pB6pB(k,2) = joints1p2p3pn4pB5pB6pB(k,2) + 90;

%% Store all of the solutions for the last three joints (normal and prime)
   
    joints123n456(k,4:6) = [th4 th5 th6];
    joints123n4p5p6p(k,4:6) = [th4p th5p th6p];
    joints1p2p3pn4B5B6B(k,4:6) = [th4B th5B th6B];
    joints1p2p3pn4pB5pB6pB(k,4:6) = [th4pB th5pB th6pB];


 end

%% Obtain only valid solutions from for the solution sets

joints123n456 = rmmissing(joints123n456);
joints123n4p5p6p = rmmissing(joints123n4p5p6p);
joints1p2p3pn4B5B6B = rmmissing(joints1p2p3pn4B5B6B);
joints1p2p3pn4pB5pB6pB = rmmissing(joints1p2p3pn4pB5pB6pB);

if isempty(joints123n456)
   %if the position is out of reach, display error message.
   msg = 'Error occurred. Target is out of reach of the manipulator workspace (Singularity Position)';
   error(msg)
end

%% 5. Return results

if 0==0 % OR 1 to avoid printing all of the time.

    fprintf('MANIPULATOR SOLUTIONS\n')
%Solutions sets for theta 1
    %Solution sets for theta 1,2
        %Solution sets for theta 4,5,6 variations
         solution11 = joints123n456;
         fprintf(['Solution Set 1.1 :\nth1= %4.2f to %4.2f , th2= %4.2f to %4.2f  , ' ...
             'd3= %4.2f to %4.2f , th4= %4.2f, th5= %4.2f to %4.2f , th6= %4.2f to %4.2f \n'] ...
             ,joints123n456(1,1),joints123n456(end,1),joints123n456(1,2),joints123n456(end,2) ...
             ,joints123n456(1,3),joints123n456(end,3),joints123n456(end,4),joints123n456(1,5), ...
             joints123n456(end,5),joints123n456(1,6),joints123n456(end,6));
       
         %Solution sets for theta 4',5',6; (prime) variations
         solution12 = joints123n4p5p6p;
         fprintf(['Solution Set 1.2 :\nth1= %4.2f to %4.2f , th2= %4.2f to %4.2f  , ' ...
             'd3= %4.2f to %4.2f , th4= %4.2f, th5= %4.2f to %4.2f , th6= %4.2f to %4.2f \n'] ...
             ,joints123n4p5p6p(1,1),joints123n4p5p6p(end,1),joints123n4p5p6p(1,2),joints123n4p5p6p(end,2) ...
             ,joints123n4p5p6p(1,3),joints123n4p5p6p(end,3),joints123n4p5p6p(end,4),joints123n4p5p6p(1,5), ...
             joints123n4p5p6p(end,5),joints123n4p5p6p(1,6),joints123n4p5p6p(end,6));

    %Solution sets for theta 1',2' (prime)
        %Solution sets for theta 4,5,6 variations
         solution21 = joints1p2p3pn4B5B6B;
         fprintf(['Solution Set 2.1 :\nth1= %4.2f to %4.2f , th2= %4.2f to %4.2f  , ' ...
             'd3= %4.2f to %4.2f , th4= %4.2f, th5= %4.2f to %4.2f , th6= %4.2f to %4.2f \n'] ...
             ,joints1p2p3pn4B5B6B(1,1),joints1p2p3pn4B5B6B(end,1),joints1p2p3pn4B5B6B(1,2),joints1p2p3pn4B5B6B(end,2) ...
             ,joints1p2p3pn4B5B6B(1,3),joints1p2p3pn4B5B6B(end,3),joints1p2p3pn4B5B6B(end,4),joints1p2p3pn4B5B6B(1,5), ...
             joints1p2p3pn4B5B6B(end,5),joints1p2p3pn4B5B6B(1,6),joints1p2p3pn4B5B6B(end,6));
        
         
         %Solution sets for theta 4',5',6' (prime) variations
         solution22 = joints1p2p3pn4pB5pB6pB;
         fprintf(['Solution Set 2.2 :\nth1= %4.2f to %4.2f , th2= %4.2f to %4.2f  , ' ...
             'd3= %4.2f to %4.2f , th4= %4.2f, th5= %4.2f to %4.2f , th6= %4.2f to %4.2f \n'] ...
             ,joints1p2p3pn4pB5pB6pB(1,1),joints1p2p3pn4pB5pB6pB(end,1),joints1p2p3pn4pB5pB6pB(1,2),joints1p2p3pn4pB5pB6pB(end,2) ...
             ,joints1p2p3pn4pB5pB6pB(1,3),joints1p2p3pn4pB5pB6pB(end,3),joints1p2p3pn4pB5pB6pB(end,4),joints1p2p3pn4pB5pB6pB(1,5), ...
             joints1p2p3pn4pB5pB6pB(end,5),joints1p2p3pn4pB5pB6pB(1,6),joints1p2p3pn4pB5pB6pB(end,6));

solutionSet = {solution11,solution12,solution21,solution22};
end
 
%% SHOW ROBOT
%ImportJonaBot(solution111); %Call JonaBot Function to show the robot with the chosen configuration

shg; %Bring the plot window to the front

% Show SolutionSet 1.1
for i = 1:size(joints123n456,1)

    ImportRedundant(joints123n456(i,:),'Solution Set 1.1');
end

% Show SolutionSet 2.1
for i = 1:size(joints123n456,1)

    ImportRedundant(joints1p2p3pn4B5B6B(i,:),'Solution Set 2.2');
end
