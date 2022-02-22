function result = RyT(matrix,an,option)

an = deg2rad(an);
if option == 0
   
   Ry =   [cos(an)    0     sin(an)    0
           0          1     0          0
           -sin(an)   0     cos(an)    0
           0          0     0          1];
   result = Ry * matrix;
   
elseif option == 1
 
   Ry =   [cos(an)    0     sin(an)
           0          1     0      
           -sin(an)   0     cos(an)];

   rotMat = matrix(1:3,1:3);
   posVect = matrix(1:3,4);      
   rotYMat = Ry * rotMat;
   
   rotYMat = [rotYMat ; zeros(1,3)];
   posVect = [posVect ;1];
   result = [rotYMat posVect];
 
end