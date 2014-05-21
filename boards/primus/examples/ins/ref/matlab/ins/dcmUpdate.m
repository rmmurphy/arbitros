function [DCM] = dcmUpdate( DCM, gyroX, gyroY, gyroZ, dt)

    %Calculate the skew-symmetric matrix of angular rate eq 5.7
    skewSemAngMat(1,1) = 1;
    skewSemAngMat(1,2) = -gyroZ*dt;
    skewSemAngMat(1,3) = gyroY*dt;
    skewSemAngMat(2,1) = gyroZ*dt;
    skewSemAngMat(2,2) = 1;
    skewSemAngMat(2,3) = -gyroX*dt;
    skewSemAngMat(3,1) = -gyroY*dt;
    skewSemAngMat(3,2) = gyroX*dt;
    skewSemAngMat(3,3) = 1;    
    
    %Update the DCM based on eq 5.6 and 5.7
    DCM = DCM*skewSemAngMat;

end

