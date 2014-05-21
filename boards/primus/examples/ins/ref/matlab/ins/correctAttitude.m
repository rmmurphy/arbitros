function [ DCMout] = correctAttitude( DCMin, dAError )
    sSMAE(1,1) = 1;
    sSMAE(1,2) = dAError(3);
    sSMAE(1,3) = -dAError(2);
    sSMAE(2,1) = -dAError(3);    
    sSMAE(2,2) = 1;     
    sSMAE(2,3) = dAError(1);
    sSMAE(3,1) = dAError(2);
    sSMAE(3,2) = -dAError(1);
    sSMAE(3,3) = 1;
    DCMout = sSMAE*DCMin;
end

