%% FORDWARD KINEMATICS
% This function calculates the forward kinematics of a given set of angles
% (Degrees) for the JonaBot DH parameters. The function returns the 
% transformation matrix of frame {tcp} represented in frame {0} (t0tcp).

function t0tcp = forwardKinematicsFuncTCP (t1,t2,t3,t4,t5,t6)


%% declare the manipulator DH table as a Matrix

 dhpar =[0,    0,    0, t1;
        90,  650,    0, t2;
         0, 1600,    0, t3;
        90,    0, 1350, t4;
       -90,    0,    0, t5;
        90,    0,    0, t6;
         0,    0,  330,  0];

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
        result {1,i} = tmat;
    
    end

%Create true zeros for the matrixes
for i=1:6
    for j=1:4
        for k = 1:4
            result{1,i}(j,k) = round2Zero(result{1,i}(j,k));
        end
    end
end

%create separate matrixes with the array values.
[t01,t12,t23,t34,t45,t56,t6tcp] = result{:};


%calculate the general expresions of the Transform matrixes of frame
%seen in the previos one.

t46 = t45*t56;
t36 = t34*t46;
t26 = t23*t36;
t16 = t12*t26;
t06 = t01*t16;
t0tcp = t06*t6tcp;