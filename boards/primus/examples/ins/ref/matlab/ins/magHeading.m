function [heading] = magHeading( magX, magY, magZ, roll, pitch)
   %Calculate the magnetic heading according to eq 10.6
   heading = atan2( -magY*cos(roll) + magZ*sin(roll), magX*cos(pitch) + magY*sin(roll)*sin(pitch)+ magZ*cos(roll)*sin(pitch));
end

