
%% CALCULATES THE INVERSE KINEMATICS OF A GIVEN TARGET FOR THE JONABOT
% This program calculates the inverse kinematics given a certain target for
% the JonaBot. It provides the option to rotate or change the position of
% the target. The program then prints out all the possible solutions and
% then it displays the robot model in all these different solutions.


function solution = inverseKinematicsFunc (targetFrame)
%% 1.INITIALIZATION OF VARIABLES
% Allocate a variables for the robot angles (theta 1-6)
 th1 = 0;
 th2 = 0;
 th3 = 0;
 th4 = 0;
 th5 = 0;
 th6 = 0;

% Provide the length of the link 1, 2 and the offset a2 of the robot
 l2 = 1600;
 l3 = 1350; 
 a2 = 650;
    
% Provide the transformation matrix from frame {6}(wrist) to {TCP}
 T6tcp = [1   0   0   0
          0   1   0   0
          0   0   1   330
          0   0   0   1];

%% 2.FIND THETA 1 FOR THE MANIPULATOR
% Equal the position of the TCP to the target one.
 Tp0tcp = targetFrame;

% Find the position of the wrist (T06') in the desired position. i.e.
% when the TCP is in the desired position (equaled to the target frame)
% using the equation T06' = T0tcp' * (T6tcp)^-1

 Ttcp6 = transMatTranspose(T6tcp);
 Tp06 = Tp0tcp * Ttcp6;

% In the manipulator configuration chosen, the first three angles
% provide the position for the TCP frame, so we will only require the
% position vector from the Transformation Matrix. (wPos = P'06).

% Extract the position vector of the transformation Matrix  
 wPos = Tp06 (1:4,4);

%Calculate the theta1 angle by usign Atan2 function using the position
%values (Y,X) of the position vector
 th1= atan2d(wPos(2),wPos(1));

%Find the second solution for theta 1
 th1p= th1 - 360; 

%% 3. FIND THETA 3 FOR THE MANIPULATOR.

%Represent the position vector P'06 in the new (rotated by th1 degrees)
%frame {1'}. Formula = T1'0 * P'06' = P1'6'. This will provide the correct
% X,Y cordinates of the wrist position in the new plane created by link2
% and link3 when the rotation of theta 1 takes place. If this is not done, 
% the coordinates of the wrist will only be valid when th1=0.

%Calculate the posotion of each frame respect to frame {0} with the angle
%values obtain so far (theta 1)
 tmatarray = forwardKinematicsFunc(th1,th2,th3,th4,th5,th6);
 [Tp01,Tp12,Tp23,Tp34,Tp45,Tp56,Tp6tcp] = tmatarray{:};
   
% Position of wrist when TCP is equalled to target represented in the new 
% frame {1'}(roatetd by theta1).
 Tp10 = transMatTranspose(Tp01);
 Pp16 = Tp10 * Tp06;
% Extract the Position vector of wrist from the transfromation matrix
 wPos16 = Pp16 (1:3,4);

%check if the desired position is in reach of the manipulator before doing 
%the calculation
 if ((l2+l3)> (sqrt(((wPos16(1)-a2)^2)+ (wPos16(3)^2))))
   %if it is in reach, calculate theta 3 using the law of cosines function
    th3 = angleFromLawOfCosines(l2,l3,wPos16(1)-a2,wPos16(3),1)*-1;

   %find the other possible solution of theta 3: theta3 prime = (mirror of theta3)
    th3p = -th3;
 else
    %if the position is out of reach, display error message.
    msg = 'Error occurred. Target is out of reach of the manipulator workspace (Singularity Position)';
    error(msg)
 end
  
%% 4. FIND THETA 2 FOR THE MANIPULATOR.

%find gamma and beta angles.
 y = angleFromLawOfCosines(l2,l3,wPos16(1)-a2,wPos16(3),2);
 b = atan2d(wPos16(3),abs(wPos16(1)-a2));

%decide whether gamma should be summed or substracted based on the sign
% (position) of theta 3
 if (th3 > 0)
    th2 = b - y;
 else
    th2 = b + y;
 end

%find the second solution of theta 2
 if (th3p > 0)
    th2p = b - y;
 else
    th2p = b + y;
 end
    
    
%% 5. FIND THETA 4,5,6 FOR THE MANIPULATOR.

%Once the position is set, the rest of the angles calculate the
%orientation of the end effector. For this, we need to know what is the
%orientation of frame 4 when the position is set. Then the rest of the
%angles are calculated from the euler angle equations.


% Calculate T46 (frame 6 seen in 4) when in the desired position
 th4 = 0; %Set theta 4 to zero

%Calculate the posotion of each frame with the angle values obtain so
%far (theta 1,theta 2,theta 3, and set theta 4 = 0)
 tmatarray = forwardKinematicsFunc(th1,th2,th3,th4,th5,th6); 
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
%calculated, mathlab provide a really small number instead of a true 0 that
%can create erratic degree results when input in atan2 function).
 th4 = atan2d(round2Zero(Tp46(2,3)),round2Zero(Tp46(1,3)));
 th5 = atan2d(round2Zero(sqrt(Tp46(1,3)^2 + Tp46(2,3)^2 )),round2Zero(Tp46(3,3)));
 th6 = atan2d(round2Zero(Tp46(3,2)),round2Zero(-Tp46(3,1)));

%Calculate angle theta 4, theta 5, and theta 6 for -180<=Th5<=0
th4p = atan2d(round2Zero(-Tp46(2,3)),round2Zero(-Tp46(1,3)));
th5p = atan2d(round2Zero(-sqrt(Tp46(1,3)^2 + Tp46(2,3)^2 )),round2Zero(Tp46(3,3)));
th6p = atan2d(round2Zero(-Tp46(3,2)),round2Zero(Tp46(3,1)));

%% Add offset to theta3
th3 = th3 +90;
th3p = th3p +90;

%% 5. Return results

if 0==1 %to avoid printing all of the time!!
%Solutions sets for theta 1
    %Solution sets for theta 2,3
        %Solution sets for theta 4,5,6 variations
         solution111 = [th1,th2,th3,th4,th5,th6];
         fprintf('Solution Set 1.1.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2,th3,th4,th5,th6);
        %Solution sets for theta 4',5',6; (prime) variations
         solution112 = [th1,th2,th3,th4p,th5p,th6p];
         fprintf('Solution Set 1.1.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2,th3,th4p,th5p,th6p);

    %Solution sets for theta 2',3' (prime)
        %Solution sets for theta 4,5,6 variations
         solution121 = [th1,th2p,th3p,th4,th5,th6];
         fprintf('Solution Set 1.2.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2p,th3p,th4,th5,th6);
        %Solution sets for theta 4',5',6' (prime) variations
         solution122 = [th1,th2p,th3p,th4p,th5p,th6p];
         fprintf('Solution Set 1.2.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2p,th3p,th4p,th5p,th6p);


%Solutions sets for theta 1' (prime)
    %Solution sets for theta 2,3
        %Solution sets for theta 4,5,6 variations
         solution211 = [th1p,th2,th3,th4,th5,th6];
         fprintf('Solution Set 2.1.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2,th3,th4,th5,th6);
        %Solution sets for theta 4',5',6; (prime) variations
         solution212 = [th1p,th2,th3,th4p,th5p,th6p];
         fprintf('Solution Set 2.1.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2,th3,th4p,th5p,th6p);

    %Solution sets for theta 2',3' (prime)
        %Solution sets for theta 4,5,6 variations
         solution221 = [th1p,th2p,th3p,th4,th5,th6];
         fprintf('Solution Set 2.2.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2p,th3p,th4,th5,th6);
        %Solution sets for theta 4',5',6' (prime) variations
         solution222 = [th1p,th2p,th3p,th4p,th5p,th6p];
         fprintf('Solution Set 2.2.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2p,th3p,th4p,th5p,th6p);

solutionSet = {solution111,solution112,solution121,solution122,solution211,solution212,solution221,solution222};
end


solution111 = [th1,th2,th3,th4,th5,th6];
fprintf('Solution Set 1.1.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2,th3,th4,th5,th6);
solution = solution111;

end