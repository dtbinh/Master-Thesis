function [x_Car,y_Car,z_Car] = CoorTransferCar2Cy(x_Cyl,phi,r)

% This function transfer the coordinates in cylindrical frame to
% coordinates in Cartesian frame

% assume the angle is between [0,2*pi]
% note the frame definition method for our udnerwater robot
% north-east-down(NED)

% The positive direction of the angle phi is the counterclockwise rotation

x_Car=x_Cyl;
y_Car=r*cos(phi);
z_Car=r*sin(phi);






end

