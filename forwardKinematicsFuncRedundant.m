%% FORDWARD KINEMATICS REDUNDANT MANIPULATOR
% This function calculates the forward kinematics for a given set of angles
% for the JonaBot DH parameters. The function returns a set of 6
% Transformation matrixes from frame {0} to frame {tcp} t01,t12,t23,t34,t45,t56,t6tcp

function result = forwardKinematicsFuncRedundant (t1,t2,d3,t4,t5,t6)
%% Initialize the manipulator DH table as a Matrix

 dhpar =[0,    0,    0, t1;
         0, 1748,    0, t2+90;
        90,    0,   d3,  0;
         0,    0, 1068, t4;
       -90,    0,    0, t5;
        90,    0,    0, t6;
         0,    0,  547,  0];


%% calculate the Transformation matrix for each Link of the manipulator

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
result (1,i) = {tmat};

end

end
