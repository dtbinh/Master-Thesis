%% fin configuration matrix calculation using symbolic toolbox

% the angle of attack of hydrofoils can be approximated by the mechanical
% angle of rotation of the hydrofoils

% angle of attack is treated as control input

% small drift angle

% slow manoeuring assumption 

syms alpha

syms C_L

syms C_D

% C_L=0.3;
% 
% C_D=0.3;

C_L_alpha=C_L*alpha;

C_D_alpha=C_D*alpha^2;

% The fin position is characterized by two parameters

% define the position of fin with cylindrical coordinate frame

% the postion of fin middle point in body frame {b}

syms x_F

% rotation angle 

syms gama

% hull diameter

syms d_H

syms a_F b_F

% the fin size approximated as rectangle

r_F=[x_F;0.5*(d_H+a_F)*sin(gama);0.5*(d_H+a_F)*cos(gama)];

% r_F X

r_F_cross=vp(r_F); 

% the inflow velocity

syms u

% surrounding fluid density

syms rho_f

q_u=0.5*rho_f*u^2;


% lift and drag force 

F_L=[0;C_L*a_F*b_F*q_u*cos(gama)*alpha;-C_L*a_F*b_F*q_u*sin(gama)*alpha];

F_D=[C_D*a_F*b_F*q_u*alpha^2;0;0];

% lift and drag moment calculation 

M_L=r_F_cross*F_L;

M_D=r_F_cross*F_D;

% let us construct the configuration matrix for the lift

% we take extract the term lift force 


B_F_L=[0;C_L*a_F*b_F*q_u*cos(gama);-C_L*a_F*b_F*q_u*sin(gama)];

B_F_D=[C_D*a_F*b_F*q_u;0;0];

B_M_L=r_F_cross*B_F_L;

B_M_D=r_F_cross*B_F_D;

B_L=simplify([B_F_L;B_M_L]);

B_D=simplify([B_F_D;B_M_D]);
