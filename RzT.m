function result = RzT(matrix,an,option)

 
an = deg2rad(an);
if option == 0
   
 Rz = [cos(an)  -sin(an)   0   0
       sin(an)   cos(an)   0   0
       0          0        1   0
       0          0        0   1];
 
   result = Rz * matrix;
   
elseif option == 1
 
 Rz = [cos(an)  -sin(an)   0
       sin(an)   cos(an)   0
       0          0        1 ];
 

   rotMat = matrix(1:3,1:3);
   posVect = matrix(1:3,4);      
   rotZMat = Rz * rotMat;
   
   rotZMat = [rotZMat ; zeros(1,3)];
   posVect = [posVect ;1];
   result = [rotZMat posVect];
 
end