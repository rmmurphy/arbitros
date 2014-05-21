function [roll,pitch,yaw] = eulerAngles( DCM)
    %Calculate the euler angles using eq 2.17
    pitch = -atan2( DCM(3,1), sqrt(1 - DCM(3,1)*DCM(3,1)));
    roll = atan2( DCM(3,2),DCM(3,3));
    yaw = atan2( DCM(2,1), DCM(1,1));
end
