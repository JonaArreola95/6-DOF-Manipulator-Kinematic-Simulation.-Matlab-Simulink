%% INVERSE KINEMATICS FUNCTION

% This function calculates the inverse kinematics given a certain target 
% for the JonaBot. The program calculates all the possible solutions. It
% provides the option to choose from the 8 different solutions as the
% arguments. It returns a vector of 1X6 containing the angle solution of the
% JonaBot for the provided target.

function solution = inverseKinematicsFuncOp (targetFrame, solNum)


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
%angles are calculated from the euler angles equations.

% **** A. Calculations for th1,th2,th3 *****

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
 tmatarray = forwardKinematicsFunc(th1p,th2p,th3p,th4B,th5,th6); 
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


%% Add offset to theta3
th3 = th3 +90;
th3p = th3p +90;

%% 5. Return results


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
         solution121 = [th1,th2p,th3p,th4B,th5B,th6B];
         fprintf('Solution Set 1.2.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2p,th3p,th4B,th5B,th6B);
        %Solution sets for theta 4',5',6' (prime) variations
         solution122 = [th1,th2p,th3p,th4pB,th5pB,th6pB];
         fprintf('Solution Set 1.2.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1,th2p,th3p,th4pB,th5pB,th6pB);


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
         solution221 = [th1p,th2p,th3p,th4B,th5B,th6B];
         fprintf('Solution Set 2.2.1 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2p,th3p,th4B,th5B,th6B);
        %Solution sets for theta 4',5',6' (prime) variations
         solution222 = [th1p,th2p,th3p,th4pB,th5pB,th6pB];
         fprintf('Solution Set 2.2.2 :\nth1= %4.2f, th2= %4.2f, th3= %4.2f, th4= %4.2f, th5= %4.2f, th6= %4.2f\n\n',th1p,th2p,th3p,th4pB,th5pB,th6pB);

solutionSet = {solution111,solution112,solution121,solution122,solution211,solution212,solution221,solution222};

%% Return Solution

 solution = deg2rad(solutionSet{1,solNum});

end

%% FUNCTIONS

%% FORDWARD KINEMATICS FUNCITON
%THIS FUNCTION CALCULATES THE forward kinematics of a given set of angles for the JonaBot DH parameters
function result = forwardKinematicsFunc (t1,t2,t3,t4,t5,t6)
%declare the manipulator DH table as a Matrix

 dhpar =[0,    0,    0, t1;
        90,  650,    0, t2;
         0, 1600,    0, t3+90;
        90,    0, 1350, t4;
       -90,    0,    0, t5;
        90,    0,    0, t6;
         0,    0,  330,  0];

%%calculate the Transformation matrix for each Link of the manipulator

%initialize each tranformation matrix (empty) where the values will be stored
t01 = zeros (4,4);
t12 = zeros (4,4);
t23 = zeros (4,4);
t34 = zeros (4,4);
t45 = zeros (4,4);
t56 = zeros (4,4);
t6tcp = zeros (4,4);

%arrange all of the matrixes as a single cell array
result = {t01,t12,t23,t34,t45,t56,t6tcp};

for i=1:7
%obtain the four values from the DH parameters matrix one link at a time
alpha = deg2rad(dhpar(i,1));
a = dhpar(i,2);
d = dhpar(i,3);
theta = deg2rad(dhpar(i,4));

%perform the transformation matrix operation
tmat =   [cos(theta),           -sin(theta),            0,            a;
          sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),  -sin(alpha)*d;
          sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),   cos(alpha)*d;
          0,                     0,                     0,            1];

%assign the ith link trans matrix result to the ith element of the cell array    
result {1,i} = tmat;

end

end
