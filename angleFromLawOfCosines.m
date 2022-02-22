%This function calculates the angle of a triangle given two of its sides
function angle = angleFromLawOfCosines(side1,side2,hipCatAdj,hipCatOp,option)

if option == 1
    cosAlpha = ((hipCatAdj)^2  + (hipCatOp)^2 - (side1)^2 - (side2)^2) / (2*side1*side2);
    alpha = acosd (cosAlpha);
    angle = alpha;
elseif option == 2
    cosGamma = ((hipCatAdj)^2  + (hipCatOp)^2 + (side1)^2 - (side2)^2) / (2*side1*(sqrt((hipCatAdj)^2  + (hipCatOp)^2)));
    gamma = acosd (cosGamma);
    angle = gamma;

end


