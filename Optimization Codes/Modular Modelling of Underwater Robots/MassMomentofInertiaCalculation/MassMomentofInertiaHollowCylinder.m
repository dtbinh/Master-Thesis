function [Mrb_hc,I_hc] = MassMomentofInertiaHollowCylinder(r_o,r_i,h,rho)
% Estimating the mass and moment of inertia for the hollow cylinder
% r_o is the outer radius, r_i is the inner radius, h is the length

% The mass of the robot
m=rho*pi*(r_o^2-r_i^2)*h;

% inertia term of the matrix 

I_x=0.5*m*(r_o^2+r_i^2);

I_y=0.25*m*(r_o^2+r_i^2+h^2/3);

I_z=I_y;

Mrb_hc=m*eye(3);

I_hc=diag([I_x,I_y,I_z]);



end

