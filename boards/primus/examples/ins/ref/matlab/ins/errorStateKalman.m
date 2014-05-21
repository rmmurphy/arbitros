function [ dAError, dGBias, dGScale] = errorStateKalman( b_initFlag, deltaZ, gBias, gScale, DCM, dt, G, bAccel, res, rVar)
    persistent H00 H Q Qatt phi phi01 R P;
    
    if b_initFlag
                
        %==================================================================
        % Initalize the attitude error portion of the system noise (process 
        % noise) matrix. The value .38 was pulled from the ITG-3200 
        % datasheet
        % [1] eq 12.67
        %==================================================================         
        Qatt = zeros(3); %attitude system noise is a 3x3 matrix
        Qatt(1,1) = (.38*pi/180)*dt; %gyro random noise x axis in rad/sec
        Qatt(2,2) = (.38*pi/180)*dt; %gyro random noise y axis in rad/sec
        Qatt(3,3) = (.38*pi/180)*dt; %gyro random noise z axis in rad/sec
      
        %==================================================================
        % Initialize the error covariance 9x9 matrix to all zeros since
        % this is an error-state not total-state Kalman filter.
        % [1] section 3.2.1 eq 3.5
        %==================================================================     
        P = [ zeros(3), zeros(3), zeros(3);
              zeros(3), zeros(3), zeros(3);
              zeros(3), zeros(3), zeros(3)];
     
        %==================================================================
        % Inialize the measurement noise 3x3 matrix. The diagonals
        % represent the noise on the 'x' axis accelerometer, 'y' axis
        % accelerometer, and 'y' axis magnetometer referenced in the
        % navigation frame. The values could be determined dynamically
        % by measuring the variance on each one of these parameters,
        % however they were set experimentally.
        %==================================================================         
        R = zeros(3);
        R(1,1) = .05; %Accelerometer x error
        R(2,2) = .05; %Accelerometer y error
        R(3,3) = .05; %Magnetometer y axis error

    else

        %##################################################################
        % Begin propagation phase...
        %##################################################################
        
        %==================================================================
        % Update attitude system (process) noise. This step corrects for
        % Kalman tracking errors by increasing the roll, pitch and yaw 
        % process noise whenever a large error has been detected.
        %==================================================================
        if( res(1) > 5*rVar(1))
           Qatt(1,1) = .2;
        else
           Qatt(1,1) = (.38*pi/180)*dt;
        end
        
        if( res(2) > 5*rVar(2))
           Qatt(2,2) = .2;
        else
           Qatt(2,2) = (.38*pi/180)*dt;
        end
            
        if( res(3) > rVar(3))
           Qatt(3,3) = .2;
        else
           Qatt(3,3) = (.38*pi/180)*dt;
        end 

        %==================================================================
        % Update gyro bias and gyro scale factor system (process) noise. 
        % The value is set to 10% of the current bias and scale factor
        % estimate.
        % [1] section 4.4.1
        %==================================================================
        QgBias = zeros(3);
        QgBias(1,1) = abs(gBias(1))*.1*dt; 
        QgBias(2,2) = abs(gBias(2))*.1*dt; 
        QgBias(3,3) = abs(gBias(3))*.1*dt;
        
        if( QgBias(1,1) < 1/32768)
            QgBias(1,1) = 1/32768;
        end
        
        if( QgBias(2,2) < 1/32768)
            QgBias(2,2) = 1/32768;
        end
        
        if( QgBias(3,3) < 1/32768)
            QgBias(3,3) = 1/32768;
        end
        
        QgScale = zeros(3);   
        QgScale(1,1) = gScale(1)*.1*dt;
        QgScale(2,2) = gScale(2)*.1*dt; 
        QgScale(3,3) = gScale(3)*.1*dt;
        
        if( QgScale(1,1) < 1/32768)
            QgScale(1,1) = 1/32768;
        end
        
        if( QgScale(2,2) < 1/32768)
            QgScale(2,2) = 1/32768;
        end
        
        if( QgScale(3,3) < 1/32768)
            QgScale(3,3) = 1/32768;
        end
        
        Q = [Qatt,     zeros(3), zeros(3);
             zeros(3), QgBias,   zeros(3);
             zeros(3), zeros(3), QgScale];
        
        %==================================================================
        % Update the transition matrix
        %          |I3 DCM*dt -DCM*G*dt |
        % phiINS = |0  I3     0         |
        %          |0  0      I3        |
        % [1] eq 12.39 with v, r, and ba removed and gyro scale factor
        % added
        % [2] eq 10 with the rotation rate removed and attitude propagation
        % term negated
        % [3] Section 10.5.3 with xa removed Fg = 0, gyro scale rate added
        % and attitude propagation term negated.
        %==================================================================        
        phi01 = DCM*dt;
        phi02 = -DCM*[G(1), 0,    0;
                     0,    G(2), 0;
                     0,    0,    G(3)]*dt;
                 
        phi = [eye(3),   phi01,    phi02;
               zeros(3), eye(3),   zeros(3);
               zeros(3), zeros(3), eye(3)];
           
        %==================================================================
        % Propagate the error covariance matrix
        % [1] eq 3.11
        %==================================================================         
        P = phi*P*phi' + Q;

        %##################################################################
        % End propagation phase...
        %################################################################## 

        %##################################################################
        % Begin correction phase...
        %##################################################################

        gyMag = norm( G);
        %Hard limit the amount of rotation and acceleration
        if( gyMag > 1)
          gyMag = 1;
        end     

        if( bAccel > 1)
           bAccel = 1;
        end

        %==================================================================
        % Update the measurement noise estimate. Wheenver there is a large
        % amount of rotation or accleration we want to ignore correcting
        % the DCM since the accelerometer is measuring specific force
        % instead of gravity. Additionally, the magnetometer is inaccurate
        % with a high degree of rotation - corrections based on it need to
        % be ignored as well.
        % [3] eq 10.61
        %==================================================================         
        if (bAccel > .1)%.1g's       
           R(1,1) = .05 + .95*(bAccel/1);
           R(2,2) = .05 + .95*(bAccel/1);
        else
           R(1,1) = .05;
           R(2,2) = .05;       
        end
   
        if (gyMag > .17)  %10 degress/sec          
           R(3,3) = .05 + .95*(gyMag/1);           
        else
           R(3,3) = .05;            
        end

        %==================================================================
        % Calcualte the measurement matrix
        % [2] eq 17 with the magnetometer yaw term added
        % [3] eq's 10.42 and 10.45 are combined with the Rbn term removed 
        % and ge/me terms negated.
        %==================================================================         
        H00 = [0,-1, 0;
               1, 0, 0;
               0, 0,-1];
        H = [H00, zeros(3), zeros(3)];

        %==================================================================
        % Since the off diagonal elements of H*P*H' are small, they are
        % ignored in order to avoid a matrix inverse
        %==================================================================         
        T = H*P*H'+ R;
        T1 = zeros(3);
        T1(1,1) = 1/T(1,1);
        T1(2,2) = 1/T(2,2);
        T1(3,3) = 1/T(3,3);

        %==================================================================
        % Update the Kalman gain
        % [1] eq 3.15
        %==================================================================         
        K = (P*H')*T1;%/(H*P*H'+ R);

        %==================================================================
        % Update the state vector estimate ignoring the previous xk term
        % since this is an error-state Kalman filter
        % [1] eq 3.16 and section 3.2.6
        %==================================================================         
        xAhrs = K*deltaZ;
        dAError = xAhrs(1:3);
        dGBias = xAhrs(4:6);
        dGScale = xAhrs(7:9);

        %==================================================================
        % Update the error covariance matrix
        % [1] eq 3.17
        %==================================================================         
        P = (eye(9)-K*H)*P;

        %##################################################################
        % End correction phase...
        %##################################################################
    end

end


