% test ellipsoid fit

% create the test data:
% radii

H = [-449.163365595193,-338.5756037712,-46.6866344048066;
-446.181763877395,-356.076496605567,-64.9182361226048;
-443.507142674936,-370.572185619891,-77.0928573250635;
-278.613636073762,89.577552237254,253.263636073762;
237.114267402997,165.061201128918,-97.2142674029979;
-179.950053840624,171.77871555019,324.350053840624;
-207.804148826078,-320.19082746035,-424.045851173921;
191.492315730202,146.499648122771,-163.342315730202;
-247.96130944789,-315.241079992044,-44.8886905521098;
-82.877741954886,-187.961859378465,-383.972258045114;
-440.211724139181,-251.071139599365,26.6117241391812;
-147.062139468195,-166.218325856979,-396.537860531805;
-133.674837840403,-338.04527368531,-320.925162159597;
-465.082413153505,-188.315412769059,30.7324131535054;
-353.725811435707,66.5965818486912,120.875811435707;
-173.725180106471,186.274404564514,325.625180106471;
-452.432286887658,-141.823141906043,48.3322868876584;
-149.168199591132,215.442559288459,343.818199591132;
-274.714772466386,167.889628253664,212.614772466386;
-409.549927574777,-153.313627100325,99.6999275747777;
-430.45068516986,-222.9636450472,49.1006851698602;
-427.89370465452,-229.150829382582,45.2937046545207;
-431.415151263927,-236.398673889744,41.0651512639274]

H= fix(H);

roll = pi/4;
pitch  = 0;%-pi/4;
yaw   = 0;%pi/4;

calEnv = 4096;

%Used for giving the readings some more distortion in order to test out the
%performance of the algorithm.
ROT = [cos(roll)*cos(yaw) -cos(pitch)*sin(yaw) + sin(pitch)*sin(roll)*cos(yaw) sin(pitch)*sin(yaw)+cos(pitch)*sin(roll)*cos(yaw);
       cos(roll)*sin(yaw) cos(pitch)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw) -sin(pitch)*cos(yaw)+cos(pitch)*sin(roll)*sin(yaw);
       -sin(roll)         sin(pitch)*cos(roll)                                cos(pitch)*cos(roll)];
   
%The axis scale factors as calculated by the least mean squares estimator
% needed to translate the axis of the ellipsoid back onto a sphere of
% radius 1..

S = [    5.65514                        0                         0
                         0    12.96103                        0
                         0                         0    22.78584];

%The rotation and scale factors...
Winv = [0.508 0.711 0.487
-0.693 0.000905 0.721
0.512 -0.704 0.493]';

%The bias as calculated by the least mean squares estimator...
B = [-97.12036 33.46997 -10.94434];

Hnew = H;%H(1:32:end,:);

%Apply corrections in order to turn the ellipse back into a sphere...

x = Hnew(:,1)-B(1);
y = Hnew(:,2)-B(2);
z = Hnew(:,3)-B(3);
Hnew = [x y z];
%Rotate so that the principal axis' are in line with x y z. This is accomplished using
%Winv. Once properly aligned the scaling vector S is applied. Finally,
%rotate back to the original position.
Hnew = (Hnew*Winv*S)*Winv';
x = Hnew(:,1);
y = Hnew(:,2);
z = Hnew(:,3);

% do the fitting
[ center, radii, evecs, v ] = ellipsoid_fit( [x y z ],0);
fprintf( 'Ellipsoid center: %.3g %.3g %.3g\n', center );
fprintf( 'Ellipsoid radii : %.3g %.3g %.3g\n', radii );
fprintf( 'Ellipsoid evecs :\n' );
fprintf( '%.3g %.3g %.3g\n%.3g %.3g %.3g\n%.3g %.3g %.3g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form  :\n' );
fprintf( '%.3g ', v );
fprintf( '\n' );

maxX = max(abs(x));
maxY = max(abs(y));
maxZ = max(abs(z));
maxd = max([maxX maxY maxZ]);

% draw data
plot3( x, y, z, '.r');
xlabel('x')
ylabel('y')
zlabel('z')
axis([-maxd maxd -maxd maxd -maxd maxd]);
grid on;
hold on;
line(evecs(1,3),evecs(2,3),evecs(3,3));
%draw fit
maxd = maxd*1.5;
step = maxd / 50;
[ x, y, z ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd );

Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
          2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
          2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
p = patch( isosurface( x, y, z, Ellipsoid, 1 ) );
set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
view( -25, 15.5 );
axis vis3d;

roll  = atan2( evecs(2,3),evecs(3,3));
pitch = -asin( evecs(1,3));
yaw   = atan2(evecs(1,2),evecs(1,1));

camlight;
lighting phong;
