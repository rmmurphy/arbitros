%##########################################################################
% Error-state Kalman Filter AHRS Simulation
% Programmer: Ryan M Murphy 
% Date: September 13, 2012
% References: 1. Principles of GNSS, INERTIAL, AND MULTISENSOR INTEGRATED
%                NAVIGATION SYSTEMS - Paul D. Groves
%             2. Effective Kalman Filter for MEMS-IMU/Magnetometers 
%                Integrated Attitude and Heading Reference Systems - Wei Li
%                and Jinling Wang
%             3. Aided Navigation - Jay A Farrell
%             4. Direction Cosine Matrix IMU: Theory = William Premerlani 
%                and Paul Bizard
%##########################################################################

%==========================================================================
% Sparkfun 9DOF sensor data where each row triplet contains magnetometer,
% gyro, and accelerometer data with their respective x,y, and z samples
% arranges along the columns.
%==========================================================================
ahrsSamples;

%==========================================================================
% The magnetomter data is stored every 3rd row starting with the first row
%==========================================================================
mag = smp(1:3:end,:);

%==========================================================================
% The gyro data is stored every 3rd row starting with the second row
%==========================================================================
gyr = smp(2:3:end,:);

%==========================================================================
% The accelerometer data is stored every 3rd row starting with the third row
%==========================================================================
acc = smp(3:3:end,:);

%==========================================================================
% The DCM update rate
%==========================================================================
dt = .05; %seconds

%==========================================================================
% The conversion from degrees/sec to rad/sec 
%==========================================================================
gyroFullScaleRange = 2000;  %+/- 2000 deg/sec
gyroFullScaleValue = 32767; %A value of 32767 fxpnt represents 2000 deg/sec
gyroConversion = (gyroFullScaleRange / gyroFullScaleValue)*pi/180; %(rad/sec)/fxpnt
    
%==========================================================================
% Initialize the DCM using leveling (i.e. accelerometer and magnetometer
%data). In order to increase the accuracy of the estimate the acceleromter
%and magnetometer x,y, and z data is averaged over a period of 10 seconds -
%when the data stored in 'smp' was collected the 9DOF sensor stick was held
%still for the first 10 seconds...
% [3] eq's 10.13 and 10.17
%==========================================================================
numSampIn10Sec = 10/dt;
avrAccX = mean(acc(1:numSampIn10Sec,1));
avrAccY = mean(acc(1:numSampIn10Sec,2));
avrAccZ = mean(acc(1:numSampIn10Sec,3));
avrMagX = mean(mag(1:numSampIn10Sec,1));
avrMagY = mean(mag(1:numSampIn10Sec,2));
avrMagZ = mean(mag(1:numSampIn10Sec,3));

%==========================================================================
% Store the estimated gravity magnetude in g's - it is assumed that the
% platform is not moving at this time.
%==========================================================================
grav1G = sqrt(avrAccX*avrAccX + avrAccY*avrAccY + avrAccZ*avrAccZ);

%==========================================================================
% Determine the bias on each gyro axis
%==========================================================================
gBias = [mean(gyr(1:numSampIn10Sec,1))*gyroConversion;
         mean(gyr(1:numSampIn10Sec,2))*gyroConversion;
         mean(gyr(1:numSampIn10Sec,3))*gyroConversion];
     
%==========================================================================
% Initialize accelerometer, magnetometer bias', and gyro scale factor error
% which we don't know at this point...unless we have performed the
% ellipsoid fit algorithm
%==========================================================================
aBias = zeros(3,1);
mBias = zeros(3,1);
gScale = ones(3,1);

%==========================================================================
% Get the initial roll and pitch
% [3] eq's 10.14 and 10.15 in radians
%==========================================================================
[roll, pitch] = accRollPitch( floor(avrAccX) - aBias(1), floor(avrAccY) - aBias(2), floor(avrAccZ) - aBias(3));

%==========================================================================
% Calculate the pitch and roll corrected magnetic heading
% [1] eq 10.6 in radians
%==========================================================================
yaw = magHeading( floor(avrMagX) - mBias(1), floor(avrMagY) - mBias(2), floor(avrMagZ) - mBias(3), roll, pitch);

%==========================================================================
% Estimate the strength of the magnetic field
%==========================================================================
magStr = sqrt( avrMagX*avrMagX + avrMagY*avrMagY + avrMagZ*avrMagZ);

%==========================================================================
% Populate the DCM matrix
% [1] eq 2.15
%==========================================================================   
DCM(1,1) = cos(pitch)*cos(yaw);
DCM(1,2) = -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
DCM(1,3) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
DCM(2,1) = cos(pitch)*sin(yaw);
DCM(2,2) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
DCM(2,3) = -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
DCM(3,1) = -sin(pitch);
DCM(3,2) = sin(roll)*cos(pitch);
DCM(3,3) = cos(roll)*cos(pitch);

roll = zeros( length(acc), 1);
pitch = zeros( length(acc), 1);
yaw = zeros( length(acc), 1);
roll1 = zeros( length(acc), 1);
pitch1 = zeros( length(acc), 1);
yaw1 = zeros( length(acc), 1);

%==========================================================================
% Initialze the error-state Kalman filter 'state vector'
%         | dAError | attitude error 3x1
% xAHRS = | dGBias  | gyro bias error 3x1
%         | dGScale | gyro scale error 3x1
%==========================================================================
dAError = zeros(3,1);
dGBias = zeros(3,1);
dGScale = zeros(3,1);

%==========================================================================
% Initialze the error-state Kalman filter 'measurement innovation vector'
%          | deltaRoll  | roll error 1x1
% deltaZ = | deltaPitch | pitch error 1x1
%          | deltaYaw   | yaw error 1x1
% [1] eq 3.7
%========================================================================== 
deltaZ = zeros(3,1);

%==========================================================================
% Initialize the error-state Kalman filter
%========================================================================== 
errorStateKalman( 1, deltaZ, gBias, gScale, DCM, dt, gyr(1,:), 0);

bodyAccel = 0;

rollRes = zeros( length(acc), 1);
pitchRes = zeros( length(acc), 1);
yawRes = zeros( length(acc), 1);

rollResVar = 0;
pitchResVar = 0;
yawResVar = 0;
DCM1 = zeros( 3,3,length(acc));

i_count = 0;
%==========================================================================
% Perform simulation
%==========================================================================
for T=201:length(acc)
    
    %======================================================================
    % Update the gyro bias and scale factor parameters based on the error
    % calculated by the Kalman filter
    %======================================================================
    gBias = gBias + dGBias;
    gScale = gScale + dGScale; 
    
    %======================================================================
    % Apply calibration parameters
    %======================================================================
    acc(T,1) = aBias(1) + (acc(T,1) / grav1G);
    acc(T,2) = aBias(2) + (acc(T,2) / grav1G);
    acc(T,3) = aBias(3) + (acc(T,3) / grav1G);
    gyr(T,1) = ((gyr(T,1)*gyroConversion) - gBias(1))*gScale(1);
    gyr(T,2) = ((gyr(T,2)*gyroConversion) - gBias(2))*gScale(2);
    gyr(T,3) = ((gyr(T,3)*gyroConversion) - gBias(3))*gScale(3);
    mag(T,:) = mag(T,:) / magStr;

    %======================================================================
    % Perform DCM correction
    % [1] eq 12.7
    %======================================================================   
    DCM = correctAttitude( DCM, dAError);
    
    %======================================================================
    % Integrate the DCM and normalize...
    % [1] eq's 5.6 and 5.7
    %======================================================================    
    DCM = dcmUpdate( DCM, gyr(T,1), gyr(T,2), gyr(T,3), dt);
    
    %======================================================================
    % Normalize the DCM
    % [4] eq's 18, 19, 20, and 21
    %======================================================================     
    DCM = dcmNormalize( DCM);
    
    %======================================================================
    % Get the 'uncorrected' accelerometer roll and pitch. 
    % [3] eq's 10.14 and 10.15 in radians
    %======================================================================
    [roll(T), pitch(T)] = accRollPitch( acc(T,1), acc(T,2), acc(T,3));

    %======================================================================
    % Calculate the pitch and roll corrected magnetic heading
    % [1] eq 10.6 in radians
    %======================================================================   
    yaw(T) = magHeading( mag(T,1), mag(T,2), mag(T,3), roll(T), pitch(T));
    
    %======================================================================
    % Get the corrected Euler angles from the DCM
    % [1] eq 2.17
    %======================================================================   
    [roll1(T),pitch1(T),yaw1(T)] = eulerAngles( DCM);
    
    %======================================================================
    % Average the accelerometer specific force in g's
    %====================================================================== 
    specificForce = abs(sqrt(acc(T,1)*acc(T,1) + acc(T,2)*acc(T,2) + acc(T,3)*acc(T,3)) - 1);
    bodyAccel = .8*bodyAccel + .2*specificForce;  
    
    %======================================================================
    % Form measurements based on the gravity vector and magnetic field 
    % vector, where deltaZ is the attitude error measurement..
    % [3] unlabeled eq's in section 10.5.1.2 and 10.5.1.3
    %======================================================================
    grav = DCM*acc(T,:)'; %Get the gravity vector in the nav frame
    magN = DCM*mag(T,:)'; %Get the magnetic field vector in the nav frame
    deltaZ(1) = -grav(1); %Roll error is based on the 'x' grav component
    deltaZ(2) = -grav(2); %Pitch error is based on the 'y' grav component    
    deltaZ(3) = -magN(2); %Yaw error is based on the 'y' mag component
 
    %======================================================================
    % Compare the corrected attitude to the uncorrected for use in kalman
    % error monitoring
    %======================================================================    
    res(1) = abs(phaseError(roll1(T), roll(T)));
    res(2) = abs(phaseError(pitch1(T), pitch(T)));
    res(3) = abs(phaseError( yaw1(T), yaw(T)));
    
    %======================================================================
    % Average the residuals using a recursive filter with an alpha = 1 - 
    % dt/T, where T = 5sec and dt = .05sec
    %======================================================================     
    rollResVar = .99*rollResVar + .01*res(1);
    rollRes(T) = rollResVar;     
    pitchResVar = .99*pitchResVar + .01*res(2);
    pitchRes(T) = pitchResVar;
    yawResVar = .99*yawResVar + .01*res(3);
    yawRes(T) = yawResVar;
    
    rVar(1) = rollResVar;
    rVar(2) = pitchResVar;
    rVar(3) = yawResVar;
   
res1(T-200, 1) = rVar(1);
res1(T-200, 2) = rVar(2);
res1(T-200, 3) = rVar(3);

    [dAError, dGBias, dGScale] = errorStateKalman( 0, deltaZ, gBias, gScale, DCM, dt, gyr(T,:), bodyAccel, res, rVar);
    i_count = i_count + 1;
    if i_count == 689
        i_count = 698;
    end
end
hold all;
plot( roll*180/pi);
plot( roll1*180/pi);
%plot( res1(1:end,1)*180/pi);
%plot( pitch*32767/pi);
%plot( pitch1*32767/pi);
%plot( yaw*180/pi);
%plot( yaw1*180/pi);
%plot( yawRes*180/pi);
%plot( pitchRes*180/pi);
%plot( rollRes*180/pi);


