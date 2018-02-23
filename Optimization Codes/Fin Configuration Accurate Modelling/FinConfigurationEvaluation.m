% evaluate the function

% water density 
rho=1033;
% first specify the velocity vector
u_f=2;
v_f=0;
w_f=0;
p_f=0;
q_f=0;
r_f=0;

% specify the position vector 
r_f_x=0.3;
r_f_y=0;
r_f_z=0.1;

% specify the rotation vector
a_f=0;

% specify the input
delta=0.3;

% delta=0.2;

% in this setting, since the robot is only moving in the x direction
% then the angle of attack is exactly equal to mechanical rotation angle
% the mechanical angle delta should be equal to the angle of attack alpha

% we should first evaluate the angle of attack alpha

alpha_check=eval(alpha)

