function D = DragHull(r,l)

% calculate the drag matrix of hull
% approximate the hull as cylinder

% input: 1.r: radius
%        2.l: length

% drag coefficient of bullet shape
% from nasa, assume it is constant
Cd=0.295;

rho=1033;

% surge damping, consider only viscous damping, neglect the skin friction
% frontal area: A_frontal=pi*r^2;
X_u_u_norm=-0.5*rho*Cd*(pi*r^2);

% sway damping
Y_v_v_norm=-0.5*rho*Cd*(2*r*l);

% heave damping 
Z_w_w_norm=-0.5*rho*Cd*(2*r*l);

% roll damping 
% the robot body is rotation symmetric, the skin friction is neglected
% thus, the roll damping is equal to zero
K_p_p_norm=0;

% pitch damping
M_q_q_norm=-1/12*rho*(Cd*r*(l^4));

% yaw damping
N_r_r_norm=-1/12*rho*(Cd*r*(l^4));


% The complete drag matrix 
% The drrag matrix is from ();
D=diag([-X_u_u_norm -Y_v_v_norm -Z_w_w_norm -K_p_p_norm -M_q_q_norm -N_r_r_norm]);


end

