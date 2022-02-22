function result = RxT(matrix,an,option)

 
an = deg2rad(an);
if option == 0
   
   Rx = [1    0         0         0
         0    cos(an)  -sin(an)   0
         0    sin(an)   cos(an)   0
         0    0         0         1];
   
   result = Rx * matrix;
   
elseif option == 1
 
  Rx = [1    0         0       
        0    cos(an)  -sin(an)
        0    sin(an)   cos(an) ];

   rotMat = matrix(1:3,1:3);
   posVect = matrix(1:3,4);      
   rotXMat = Rx * rotMat;
   
   rotXMat = [rotXMat ; zeros(1,3)];
   posVect = [posVect ;1];
   result = [rotXMat posVect];
 

 
end