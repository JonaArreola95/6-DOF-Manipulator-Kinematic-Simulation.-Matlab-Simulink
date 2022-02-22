%% FORWARD KINEMATICS AND TRANSFORMATION MATRIX VERIFICATION FOR THE PROVIDED
%% REDUNDANT MANIPULATOR
% This program calculates the transformation matrixes for the provided
% redundant manipulator. Then it evaluates them using a set of angles to 
% obtain the T06 and the T0tcp Frames.

clear
clc

%% Initialize values

% theta variables as symbols
syms t1 t2 d3 t4 t5 t6;

% Provided redundant DH table as a Matrix

 DHTbl =[0,    0,    0, t1;
         0,  412,    0, t2;
        90,    0,   d3,  0;
         0,    0,  267, t4;
       -90,    0,    0, t5;
        90,    0,    0, t6;
         0,    0,   22,  0];

%% Calculate the Transformation matrix for each Link of the manipulator

%initialize each tranformation matrix (empty) where the values will be stored
t01 = zeros (4,4);
t12 = zeros (4,4);
t23 = zeros (4,4);
t34 = zeros (4,4);
t45 = zeros (4,4);
t56 = zeros (4,4);
t6tcp = zeros (4,4);

%arrange all of the matrixes as a single cell array
tMatArray = {t01,t12,t23,t34,t45,t56,t6tcp};

%this for loop calculates the values for each link
for i=1:7
    
%obtain the four values from the DH parameters matrix one link at a time
alpha = deg2rad(DHTbl(i,1));
a = DHTbl(i,2);
d = DHTbl(i,3);
theta = DHTbl(i,4);

%perform the transformation matrix operation
tmat =   [cos(theta),           -sin(theta),            0,            a;
          sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),  -sin(alpha)*d;
          sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),   cos(alpha)*d;
          0,                     0,                     0,            1];

%assign the ith link trans matrix result to the ith element of the cell array    
tMatArray (1,i) = {tmat};
end

%tMatArray = transmat(dhpar);

%create separate matrixes with the array values.
[t01,t12,t23,t34,t45,t56,t6tcp] = tMatArray{:};

%display each of the Transformation matrixes.

display(t01);
display(t12);
display(t23);
display(t34);
display(t45);
display(t56);
display(t6tcp);

%calculate the general expresions of the Transform matrixes of frame
%seen in the previos one.

t46 = t45*t56;
t36 = t34*t46;
t26 = t23*t36;
t16 = t12*t26;
t06 = t01*t16;
t0tcp = t06*t6tcp;

%display each of the Transformation matrixes.
display(t46);
display(t36);
display(t26);
display(t16);
display(t06);
display(t0tcp);

%assign values to the theta angles   
t1 = deg2rad(90);
t2 = deg2rad(0);
d3 = 200;
t4 = deg2rad(0);
t5 = deg2rad(0);
t6 = deg2rad(0);

%evaluate function in these values
fprintf('T06 and T0tcp evaluated in:\ntheta1= %d\ntheta2= %d\ntheta3= %d\ntheta4= %d\ntheta5= %d\ntheta6= %d\n',t1,t2,d3,t4,t5,t6);

%display the Transformation matrix evaluated in theta values.

fprintf('\n T06 =');
subs(t06)   

fprintf('\n T0tcp =');
subs(t0tcp)   
