function [roll, pitch] = accRollPitch( accX, accY, accZ )

    pitch = atan2( -accX, sqrt(accY*accY + accZ*accZ));
    roll = atan2( accY, accZ);

end

