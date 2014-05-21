function [DCM] = dcmNormalize( DCM)
    %Normalize the DCM according to eq 18, 19, 20, and 21 of [4]
    
    error = DCM(1,:)*DCM(2,:)';
    Xorth = DCM(1,:)' - (error/2)*DCM(2,:)';
    Yorth = DCM(2,:)' - (error/2)*DCM(1,:)';
    Zorth = cross(Xorth,Yorth);
    
    DCM(1,:) = .5*(3-Xorth'*Xorth)*Xorth;
    DCM(2,:) = .5*(3-Yorth'*Yorth)*Yorth;
    DCM(3,:) = .5*(3-Zorth'*Zorth)*Zorth;
    if( max(DCM(1,:)) > 1)
        x = 1;
    end
    
end

