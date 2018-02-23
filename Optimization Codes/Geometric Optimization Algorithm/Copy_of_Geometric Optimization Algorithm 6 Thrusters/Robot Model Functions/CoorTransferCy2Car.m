function [x_Cyl,phi,r] = CoorTransferCy2Car(x_Car,y,z)

% This function transfers the coordinates in Cartesian frame to coordinates
% in cylindrical frame

% assume the angle is between [-pi,pi] so that it corresponds to 

% the x-coordinate should be within the range of the robot body i.e.
% within the range [-l_H,l_H]

% The x-Coordinate does not 

x_Cyl=x_Car;
phi=atan2(z,y);
r=sqrt(y^2+z^2);

end

