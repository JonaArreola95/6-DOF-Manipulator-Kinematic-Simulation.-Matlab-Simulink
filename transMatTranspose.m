%% THIS FUNCTION CALCULATES THE TRANSPOSE OF A TRANSFORMATION MATRIX
function result = transMatTranspose(tmatrix)
%%
%obtain the rotation matrix and the translation vector of the
%transformation matrix
rotMat = tmatrix(1:3,1:3);
posVect = tmatrix(1:3,4);

%obtain the transpose of the rotatio matrix
rotMatTrans = transpose(rotMat);
%obtain the inverse of the position vector 
posVectTrans = rotMatTrans * posVect * -1;

%add the last row of zeros to the new rotation matrix
result = [rotMatTrans ; zeros(1,3)];
%add the last 1 to the new position vector
posVectTrans = [posVectTrans ;1];
%integrate the rotation matrix and the position vector to form the new
%transformation matrix
result = [result posVectTrans];

end